// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       point_gimbal_behavior.cpp
 *  \brief      Point Gimbal behavior implementation file.
 *  \authors    Pedro Arias-Perez, Rafael Perez-Segui
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "point_gimbal_behavior.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/frame_utils.hpp"

namespace point_gimbal_behavior
{

PointGimbalBehavior::PointGimbalBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::PointGimbal>("PointGimbalBehavior", options),
  tf_handler_(this)
{
  // Gimbal name to publish commands
  this->declare_parameter<std::string>("gimbal_name", "gimbal");
  this->get_parameter("gimbal_name", gimbal_name_);

  gimbal_control_pub_ = this->create_publisher<as2_msgs::msg::GimbalControl>(
    "platform/" + gimbal_name_ + "/gimbal_command", 10);

  // Gimbal frame ids to get state
  this->declare_parameter<std::string>("gimbal_base_frame_id", "gimbal");
  this->get_parameter("gimbal_base_frame_id", gimbal_base_frame_id_);
  this->declare_parameter<std::string>("gimbal_frame_id", "gimbal");
  this->get_parameter("gimbal_frame_id", gimbal_frame_id_);

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  gimbal_base_frame_id_ = as2::tf::generateTfName(this, gimbal_base_frame_id_);
  gimbal_frame_id_ = as2::tf::generateTfName(this, gimbal_frame_id_);

  // Gimbal orientation threshold
  this->declare_parameter<double>("gimbal_threshold", 0.01);
  this->get_parameter("gimbal_threshold", gimbal_threshold_);

  // TF threshold
  double tf_timeout_threshold;
  this->declare_parameter<double>("tf_timeout_threshold", 0.5);
  this->get_parameter("tf_timeout_threshold", tf_timeout_threshold);
  tf_timeout_threshold_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(tf_timeout_threshold));

  // Gimbal limits (default no limits)
  this->declare_parameter<double>("roll_range.min", -4 * M_PI);
  this->get_parameter("roll_range.min", gimbal_roll_min_);
  this->declare_parameter<double>("roll_range.max", 4 * M_PI);
  this->get_parameter("roll_range.max", gimbal_roll_max_);
  this->declare_parameter<double>("pitch_range.min", -4 * M_PI);
  this->get_parameter("pitch_range.min", gimbal_pitch_min_);
  this->declare_parameter<double>("pitch_range.max", 4 * M_PI);
  this->get_parameter("pitch_range.max", gimbal_pitch_max_);
  this->declare_parameter<double>("yaw_range.min", -4 * M_PI);
  this->get_parameter("yaw_range.min", gimbal_yaw_min_);
  this->declare_parameter<double>("yaw_range.max", 4 * M_PI);
  this->get_parameter("yaw_range.max", gimbal_yaw_max_);

  RCLCPP_INFO(this->get_logger(), "Roll range: %f to %f", gimbal_roll_min_, gimbal_roll_max_);
  RCLCPP_INFO(this->get_logger(), "Pitch range: %f to %f", gimbal_pitch_min_, gimbal_pitch_max_);
  RCLCPP_INFO(this->get_logger(), "Yaw range: %f to %f", gimbal_yaw_min_, gimbal_yaw_max_);

  // Timeout
  this->declare_parameter<double>("behavior_timeout", 10.0);
  double behavior_timeout;
  this->get_parameter("behavior_timeout", behavior_timeout);
  behavior_timeout_ = rclcpp::Duration::from_seconds(behavior_timeout);
  RCLCPP_INFO(this->get_logger(), "Behavior timeout: %f", behavior_timeout_.seconds());

  // Set internal variables frame ids
  current_goal_position_.header.frame_id = gimbal_frame_id_;

  RCLCPP_INFO(
    this->get_logger(), "PointGimbalBehavior created for gimbal name %s in frame %s with base %s",
    gimbal_name_.c_str(), gimbal_frame_id_.c_str(), gimbal_base_frame_id_.c_str());
}

bool PointGimbalBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> goal)
{
  if (goal->follow_mode) {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: follow mode on not supported");
    return false;
  }
  if (goal->control.control_mode != as2_msgs::msg::GimbalControl::POSITION_MODE) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: control mode %d not supported",
      goal->control.control_mode);
    return false;
  }

  // Process goal

  // Check frame id
  desired_goal_position_.header.frame_id = goal->control.target.header.frame_id;
  if (desired_goal_position_.header.frame_id == "") {
    desired_goal_position_.header.frame_id = base_link_frame_id_;
    RCLCPP_INFO(
      this->get_logger(), "Goal frame id not set, using base_link frame id %s",
      desired_goal_position_.header.frame_id.c_str());
  }
  desired_goal_position_.point.x = goal->control.target.vector.x;
  desired_goal_position_.point.y = goal->control.target.vector.y;
  desired_goal_position_.point.z = goal->control.target.vector.z;

  // Convert goal point to gimbal frame
  if (!tf_handler_.tryConvert(
      desired_goal_position_, gimbal_base_frame_id_, tf_timeout_threshold_))
  {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: could not convert goal point from %s to frame %s",
      desired_goal_position_.header.frame_id.c_str(), gimbal_base_frame_id_.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "PointGimbalBehavior: desired goal point in %s frame: x=%f, y=%f, z=%f",
    desired_goal_position_.header.frame_id.c_str(), desired_goal_position_.point.x,
    desired_goal_position_.point.y, desired_goal_position_.point.z);

  // Get current gimbal orientation
  if (!update_gimbal_state()) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: could not get current gimbal orientation");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: current gimbal orientation in %s frame: roll=%f, pitch=%f, yaw=%f",
    gimbal_angles_current_.header.frame_id.c_str(), gimbal_angles_current_.vector.x,
    gimbal_angles_current_.vector.y, gimbal_angles_current_.vector.z);
  RCLCPP_INFO(
    this->get_logger(), "PointGimbalBehavior: current goal position in %s frame: x=%f, y=%f, z=%f",
    current_goal_position_.header.frame_id.c_str(), current_goal_position_.point.x,
    current_goal_position_.point.y, current_goal_position_.point.z);

  // Set gimbal control command

  // Point to look at:
  Eigen::Vector3d point_to_look_at(
    desired_goal_position_.point.x, desired_goal_position_.point.y, desired_goal_position_.point.z);
  point_to_look_at.normalize();

  // Angles to look at point
  double roll = 0.0;
  double pitch = -asin(point_to_look_at.z());
  double yaw = atan2(point_to_look_at.y(), point_to_look_at.x());

  // Wrap angles
  roll = as2::frame::wrapAngle0To2Pi(roll);
  pitch = as2::frame::wrapAngle0To2Pi(pitch);
  yaw = as2::frame::wrapAngle0To2Pi(yaw);

  if (!check_gimbal_limits(roll, pitch, yaw)) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: desired gimbal orientation out of limits");
    return false;
  }

  geometry_msgs::msg::Vector3 gimbal_angles_desired;
  gimbal_angles_desired.x = roll;
  gimbal_angles_desired.y = pitch;
  gimbal_angles_desired.z = yaw;

  // Set gimbal control command
  gimbal_control_msg_.control_mode = as2_msgs::msg::GimbalControl::POSITION_MODE;
  gimbal_control_msg_.target.header.frame_id = gimbal_base_frame_id_;
  gimbal_control_msg_.target.vector = gimbal_angles_desired;

  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: desired gimbal orientation in %s frame: roll=%f, pitch=%f, yaw=%f",
    gimbal_control_msg_.target.header.frame_id.c_str(), gimbal_control_msg_.target.vector.x,
    gimbal_control_msg_.target.vector.y, gimbal_control_msg_.target.vector.z);

  goal_init_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool PointGimbalBehavior::on_modify(std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Goal modified not available for this behavior");
  return false;
}

bool PointGimbalBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior cancelled");
  return true;
}

bool PointGimbalBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior paused");
  return true;
}

bool PointGimbalBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior resumed");
  goal_init_time_ = this->now();
  return true;
}

as2_behavior::ExecutionStatus PointGimbalBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> & goal,
  std::shared_ptr<as2_msgs::action::PointGimbal::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::PointGimbal::Result> & result_msg)
{
  // Check timeout
  auto behavior_time = this->now() - goal_init_time_;
  if (behavior_time.seconds() > behavior_timeout_.seconds()) {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: goal timeout");
    result_msg->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (!update_gimbal_state()) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (check_finished()) {
    result_msg->success = true;
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
  gimbal_control_pub_->publish(gimbal_control_msg_);

  // Feedback
  feedback_msg->gimbal_attitude.header.stamp = this->now();
  feedback_msg->gimbal_attitude.header.frame_id = gimbal_angles_current_.header.frame_id;
  feedback_msg->gimbal_attitude.vector = gimbal_angles_current_.vector;
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PointGimbalBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior execution ended");
}

bool PointGimbalBehavior::check_gimbal_limits(
  const double roll,
  const double pitch,
  const double yaw)
{
  double roll_w = as2::frame::wrapAnglePiToPi(roll);
  double pitch_w = as2::frame::wrapAnglePiToPi(pitch);
  double yaw_w = as2::frame::wrapAnglePiToPi(yaw);

  // Unoptimized for debugging purposes
  if (roll_w > gimbal_roll_max_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: roll %f greater than limits %f", roll_w,
      gimbal_roll_max_);
    return false;
  } else if (roll_w < gimbal_roll_min_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: roll %f less than limits %f", roll_w,
      gimbal_roll_min_);
    return false;
  } else if (pitch_w > gimbal_pitch_max_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: pitch %f greater than limits %f", pitch_w,
      gimbal_pitch_max_);
    return false;
  } else if (pitch_w < gimbal_pitch_min_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: pitch %f less than limits %f", pitch_w,
      gimbal_pitch_min_);
    return false;
  } else if (yaw_w > gimbal_yaw_max_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: yaw %f greater than limits %f", yaw_w,
      gimbal_yaw_max_);
    return false;
  } else if (yaw_w < gimbal_yaw_min_) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: yaw %f less than limits %f", yaw_w,
      gimbal_yaw_min_);
    return false;
  }

  // bool roll_in_range = (roll >= gimbal_roll_min_) && (roll <= gimbal_roll_max_);
  // bool pitch_in_range = (pitch >= gimbal_pitch_min_) && (pitch <= gimbal_pitch_max_);
  // bool yaw_in_range = (yaw >= gimbal_yaw_min_) && (yaw <= gimbal_yaw_max_);
  return true;
}

bool PointGimbalBehavior::update_gimbal_state()
{
  // Get current goal position in gimbal frame
  current_goal_position_.header.frame_id = gimbal_base_frame_id_;
  current_goal_position_.header.stamp = this->now();
  current_goal_position_.point.x = desired_goal_position_.point.x;
  current_goal_position_.point.y = desired_goal_position_.point.y;
  current_goal_position_.point.z = desired_goal_position_.point.z;
  if (!tf_handler_.tryConvert(current_goal_position_, gimbal_frame_id_, tf_timeout_threshold_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not convert current goal point from %s to frame %s",
      desired_goal_position_.header.frame_id.c_str(), gimbal_frame_id_.c_str());
    return false;
  }

  // Get current gimbal orientation for feedback
  geometry_msgs::msg::QuaternionStamped current_gimbal_orientation;
  try {
    // tf2::TimePoint time = tf2::TimePointZero;
    rclcpp::Time time = this->now();
    current_gimbal_orientation = tf_handler_.getQuaternionStamped(
      gimbal_base_frame_id_, gimbal_frame_id_, time, tf_timeout_threshold_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not get current gimbal orientation from %s to %s: %s",
      gimbal_frame_id_.c_str(), gimbal_base_frame_id_.c_str(), e.what());
    return false;
  }

  as2::frame::quaternionToEuler(
    current_gimbal_orientation.quaternion, gimbal_angles_current_.vector.x,
    gimbal_angles_current_.vector.y, gimbal_angles_current_.vector.z);

  gimbal_angles_current_.header.frame_id = gimbal_base_frame_id_;
  gimbal_angles_current_.vector.x = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.x);
  gimbal_angles_current_.vector.y = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.y);
  gimbal_angles_current_.vector.z = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.z);
  return true;
}

bool PointGimbalBehavior::check_finished()
{
  // Check angle between vectors:
  // - desired_goal_position (in gimbal frame)
  // - current_goal_position (in gimbal frame)

  Eigen::Vector3d desired_goal_position = Eigen::Vector3d(1.0, 0.0, 0.0);
  desired_goal_position.normalize();

  Eigen::Vector3d current_goal_position = Eigen::Vector3d(
    current_goal_position_.point.x, current_goal_position_.point.y, current_goal_position_.point.z);
  current_goal_position.normalize();

  double angle = abs(acos(current_goal_position.dot(desired_goal_position)));
  if (angle < gimbal_threshold_) {
    RCLCPP_INFO(
      this->get_logger(), "PointGimbalBehavior: goal reached, angle between vectors %f", angle);
    return true;
  }

  return false;
}

}  // namespace point_gimbal_behavior
