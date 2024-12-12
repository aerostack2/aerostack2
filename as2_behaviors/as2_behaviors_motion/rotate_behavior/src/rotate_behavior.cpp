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
 *  \file       rotate_behavior.cpp
 *  \brief      Rotate behavior implementation file.
 *  \authors    Pedro Arias-Pérez, Tomás Sánchez Villauenga
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "rotate_behavior.hpp"

namespace rotate_behavior
{

RotateBehavior::RotateBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::Rotate>("RotateBehavior", options),
  tf_handler_(this)
{
  // Rotation angle (yaw)
  try {
    this->declare_parameter<double>("rotation_angle");
  } catch (const rclcpp::ParameterTypeException & e) {
    // RCLCPP_FATAL(this->get_logger(),
    //  "Launch argument <rotation_angle> not defined or "
    //  "malformed: %s",
    //  e.what());
    this->~RotateBehavior();
  }
  // Rotation speed
  try {
    this->declare_parameter<double>("rotation_speed");
  } catch (const rclcpp::ParameterTypeException & e) {
    // RCLCPP_FATAL(this->get_logger(),
    //  "Launch argument <rotation_speed> not defined or "
    //  "malformed: %s",
    //  e.what());
    this->~RotateBehavior();
  }

  // Behavior name to publish commands
  this->declare_parameter<std::string>("behavior_name", "rotate_behavior");
  this->get_parameter("behavior_name", behavior_name_);

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  // Set internal variables frame ids
  // current_goal_position_.header.frame_id = base_link_frame_id_;

  // Angle threshold
  this->declare_parameter<double>("angle_threshold", 0.1);
  this->get_parameter("angle_threshold", angle_threshold);

  // RCLCPP_INFO(
  // this->get_logger(), "RotateBehavior created for behavior name %s in frame %s",
  // behavior_name_.c_str(), base_link_frame_id_.c_str());
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
    std::bind(&RotateBehavior::update_rotation_angle, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&RotateBehavior::update_rotation_speed, this, std::placeholders::_1));
}

bool RotateBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal)
{
  // Process goal
  // if (desired_goal_position_.header.frame_id == "") {
  //   desired_goal_position_.header.frame_id = base_link_frame_id_;
  //   // RCLCPP_INFO(
  //   // this->get_logger(), "Goal frame id not set, using base_link frame id %s",
  //   // desired_goal_position_.header.frame_id.c_str());
  // }

  if (goal->speed < 0.0f) {
    // RCLCPP_ERROR(this->get_logger(), "RotateBehavior: Invalid rotation speed");
    return false;
  }

  // desired_goal_position_.yaw = goal->yaw;
  // desired_goal_position_.speed = goal->speed;

  // RCLCPP_INFO(
  // this->get_logger(),
  // "RotateBehavior: desired yaw=%f and speed=%f",
  // desired_goal_position_.yaw, desired_goal_position_.speed);

  goal_init_time_ = this->now();
  // RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool RotateBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal)
{
  // RCLCPP_INFO(this->get_logger(), "Goal modified not available for this behavior");
  return false;
}

bool RotateBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "RotateBehavior cancelled");
  return true;
}

bool RotateBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "RotateBehavior paused");
  return true;
}

bool RotateBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "RotateBehavior resumed");
  goal_init_time_ = this->now();
  return true;
}

as2_behavior::ExecutionStatus RotateBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Rotate::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Rotate::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Rotate::Result> & result_msg)
{
  // Check timeout
  auto behavior_time = this->now() - goal_init_time_;
  if (behavior_time.seconds() > behavior_timeout_.seconds()) {
    // RCLCPP_ERROR(this->get_logger(), "RotateBehavior: goal timeout");
    result_msg->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (!update_rotation_state()) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (check_finished()) {
    result_msg->success = true;
    // RCLCPP_INFO(this->get_logger(), "Rotate goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // Feedback
  feedback_msg->header.stamp = this->now();
  feedback_msg->header.frame_id = base_link_frame_id_;    // base_link_frame_id_
  feedback_msg->current_yaw = current_rotation_position_;
  feedback_msg->current_speed = current_rotation_speed_;

  return as2_behavior::ExecutionStatus::RUNNING;
}


void RotateBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  // RCLCPP_INFO(this->get_logger(), "RotateBehavior execution ended");
}


bool RotateBehavior::update_rotation_state()
{
  try {
    // tf2::TimePoint time = tf2::TimePointZero;
    rclcpp::Time time = this->now();
  } catch (const std::exception & e) {
    // RCLCPP_ERROR(
    // this->get_logger(),
    // "RotateBehavior: could not get current rotation information");
    return false;
  }
}

void RotateBehavior::update_rotation_angle(const geometry_msgs::msg::PoseStamped & msg)
{
  // TODO(pariaspe): non-sense, converto to yaw
  current_rotation_position_ = msg.pose.orientation.x;
}

void RotateBehavior::update_rotation_speed(const geometry_msgs::msg::TwistStamped & msg)
{
  current_rotation_speed_ = msg.twist.angular.z;
}

bool RotateBehavior::check_finished()
{
  // Check angle between vectors:
  // - desired_goal_position (in gimbal frame)
  // - current_goal_position (in gimbal frame)

  // if (desired_goal_position_.yaw - current_rotation_position_ < angle_threshold &&
  //   desired_goal_position_.yaw - current_rotation_position_ > -angle_threshold)
  // {
  //   // RCLCPP_INFO(
  //   // this->get_logger(), "RotateBehavior: goal reached, angle between vectors %f",
  // desired_goal_position_.z - current_rotation_position_ );
  //   return true;
  // }

  return false;
}

}  // namespace rotate_behavior
