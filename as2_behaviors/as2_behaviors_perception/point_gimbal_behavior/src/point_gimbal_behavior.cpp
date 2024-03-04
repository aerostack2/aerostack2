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

PointGimbalBehavior::PointGimbalBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::PointGimbal>("PointGimbalBehavior"),
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
  this->declare_parameter<double>("gimbal_orientation_threshold", 0.01);
  this->get_parameter("gimbal_orientation_threshold", gimbal_orientation_threshold_);

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
  goal_point_.header.frame_id = goal->control.target.header.frame_id;
  if (goal_point_.header.frame_id == "") {
    goal_point_.header.frame_id = base_link_frame_id_;
    RCLCPP_INFO(
      this->get_logger(), "Goal frame id not set, using base_link frame id %s",
      goal_point_.header.frame_id.c_str());
  }
  goal_point_.point.x = goal->control.target.vector.x;
  goal_point_.point.y = goal->control.target.vector.y;
  goal_point_.point.z = goal->control.target.vector.z;

  // Convert goal point to gimbal frame
  if (!tf_handler_.tryConvert(goal_point_, gimbal_base_frame_id_)) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: could not convert goal point to gimbal frame %s",
      gimbal_base_frame_id_.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "PointGimbalBehavior: goal point in gimbal frame: x=%f, y=%f, z=%f",
    goal_point_.point.x, goal_point_.point.y, goal_point_.point.z);

  // Get current gimbal orientation
  if (!update_gimbal_angles()) {
    RCLCPP_ERROR(
      this->get_logger(), "PointGimbalBehavior: could not get current gimbal orientation");
    return false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: current gimbal orientation: roll=%f, pitch=%f, yaw=%f",
    gimbal_angles_current_.x, gimbal_angles_current_.y, gimbal_angles_current_.z);

  // Set gimbal control command

  // Point to look at:
  Eigen::Vector3d point_to_look_at(goal_point_.point.x, goal_point_.point.y, goal_point_.point.z);
  point_to_look_at.normalize();

  double roll = 0.0;
  double pitch = -asin(point_to_look_at.z());
  double yaw = atan2(point_to_look_at.y(), point_to_look_at.x());

  gimbal_angles_desired_.x = roll;
  gimbal_angles_desired_.y = pitch;
  gimbal_angles_desired_.z = yaw;

  // Set gimbal control command
  gimbal_control_msg_.control_mode = as2_msgs::msg::GimbalControl::POSITION_MODE;
  gimbal_control_msg_.target.header.frame_id = gimbal_base_frame_id_;
  gimbal_control_msg_.target.vector = gimbal_angles_desired_;

  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: desired gimbal orientation: roll=%f, pitch=%f, yaw=%f",
    gimbal_angles_desired_.x, gimbal_angles_desired_.y, gimbal_angles_desired_.z);

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
  return true;
}

as2_behavior::ExecutionStatus PointGimbalBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> & goal,
  std::shared_ptr<as2_msgs::action::PointGimbal::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::PointGimbal::Result> & result_msg)
{
  if (!update_gimbal_angles()) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (compare_attitude(
      gimbal_angles_desired_, gimbal_angles_current_, gimbal_orientation_threshold_))
  {
    result_msg->success = true;
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
  gimbal_control_pub_->publish(gimbal_control_msg_);

  // Feedback
  feedback_msg->gimbal_attitude.header.stamp = this->now();
  feedback_msg->gimbal_attitude.header.frame_id = gimbal_base_frame_id_;
  feedback_msg->gimbal_attitude.vector = gimbal_angles_current_;
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PointGimbalBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior execution ended");
}

bool PointGimbalBehavior::update_gimbal_angles()
{
  geometry_msgs::msg::QuaternionStamped current_gimbal_orientation;
  try {
    current_gimbal_orientation =
      tf_handler_.getQuaternionStamped(gimbal_base_frame_id_, gimbal_frame_id_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not get current gimbal orientation from %s to %s: %s",
      gimbal_frame_id_.c_str(), gimbal_base_frame_id_.c_str(), e.what());
    return false;
  }

  as2::frame::quaternionToEuler(
    current_gimbal_orientation.quaternion, gimbal_angles_current_.x, gimbal_angles_current_.y,
    gimbal_angles_current_.z);
  return true;
}

bool PointGimbalBehavior::compare_attitude(
  const geometry_msgs::msg::Vector3 & attitude1,
  const geometry_msgs::msg::Vector3 & attitude2,
  const double threshold)
{
  double roll_diff = fabs(attitude1.x - attitude2.x);
  double pitch_diff = fabs(attitude1.y - attitude2.y);
  double yaw_diff = fabs(attitude1.z - attitude2.z);

  RCLCPP_INFO(
    this->get_logger(), "Desired 1: roll=%f, pitch=%f, yaw=%f", attitude1.x, attitude1.y,
    attitude1.z);
  RCLCPP_INFO(
    this->get_logger(), "Current 2: roll=%f, pitch=%f, yaw=%f", attitude2.x, attitude2.y,
    attitude2.z);
  RCLCPP_INFO(
    this->get_logger(), "Diff     : roll=%f, pitch=%f, yaw=%f", roll_diff, pitch_diff, yaw_diff);
  RCLCPP_INFO(this->get_logger(), "Threshold: %f", threshold);

  return (roll_diff < threshold) && (pitch_diff < threshold) && (yaw_diff < threshold);
}

bool PointGimbalBehavior::compare_attitude(
  const geometry_msgs::msg::Quaternion & attitude1,
  const geometry_msgs::msg::Quaternion & attitude2,
  const double threshold)
{
  // Check if the difference between the two quaternions is less than the tolerance
  // TODO(RPS98): Improve this comparison
  double w_diff = attitude1.w - attitude2.w;
  double x_diff = attitude1.x - attitude2.x;
  double y_diff = attitude1.y - attitude2.y;
  double z_diff = attitude1.z - attitude2.z;

  RCLCPP_INFO(
    this->get_logger(), "Desired 1: w=%f, x=%f, y=%f, z=%f", attitude1.w, attitude1.x, attitude1.y,
    attitude1.z);
  RCLCPP_INFO(
    this->get_logger(), "Current 2: w=%f, x=%f, y=%f, z=%f", attitude2.w, attitude2.x, attitude2.y,
    attitude2.z);

  RCLCPP_INFO(
    this->get_logger(), "Attitude difference: w=%f, x=%f, y=%f, z=%f", w_diff, x_diff, y_diff,
    z_diff);

  return (fabs(w_diff) < threshold) && (fabs(x_diff) < threshold) && (fabs(y_diff) < threshold) &&
         (fabs(z_diff) < threshold);
}
