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
#include "as2_core/names/topics.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"

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
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <rotation_angle> not defined or malformed: %s",
      e.what());
    this->~RotateBehavior();
  }
  // Rotation speed
  try {
    this->declare_parameter<double>("rotation_speed");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <rotation_speed> not defined or malformed: %s",
      e.what());
    this->~RotateBehavior();
  }
  // Angle threshold
  try {
    this->declare_parameter<double>("angle_threshold");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <angle_threshold> not defined or malformed: %s",
      e.what());
    this->~RotateBehavior();
  }

  default_angle_ = this->get_parameter("rotation_angle").as_double();
  default_speed_ = this->get_parameter("rotation_speed").as_double();
  angle_threshold_ = this->get_parameter("angle_threshold").as_double();
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
    std::bind(&RotateBehavior::update_rotation_angle, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&RotateBehavior::update_rotation_speed, this, std::placeholders::_1));

  position_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::PositionMotion>(this);
}

bool RotateBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal)
{
  desired_goal_ = *goal;

  if (desired_goal_.speed < 0.0f) {
    RCLCPP_ERROR(this->get_logger(), "RotateBehavior: Invalid rotation speed");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "RotateBehavior: desired yaw=%.3f and speed=%.3f",
    desired_goal_.yaw, desired_goal_.speed);

  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool RotateBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal)
{
  desired_goal_ = *goal;
  RCLCPP_INFO(
    this->get_logger(),
    "RotateBehavior: Goal modified to yaw=%.3f and speed=%.3f",
    desired_goal_.yaw, desired_goal_.speed);
  return false;
}

bool RotateBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return true;
}

bool RotateBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return true;
}

bool RotateBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return true;
}

as2_behavior::ExecutionStatus RotateBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Rotate::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Rotate::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Rotate::Result> & result_msg)
{
  if (check_finished()) {
    result_msg->success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  if (!position_motion_handler_->sendPositionCommandWithYawAngle(
      base_link_frame_id_, 0.0, 0.0, 0.0, desired_goal_.yaw,
      "earth", 0.0, 0.0, 0.0))
  {
    RCLCPP_ERROR(this->get_logger(), "Rotate behavior: Error sending rotate command");
    result_msg->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Feedback
  feedback_msg->header.stamp = this->now();
  feedback_msg->header.frame_id = base_link_frame_id_;
  feedback_msg->current_yaw = current_status_.current_yaw;
  feedback_msg->current_speed = current_status_.current_speed;

  return as2_behavior::ExecutionStatus::RUNNING;
}


void RotateBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  RCLCPP_INFO(this->get_logger(), "RotateBehavior execution ended");
}

void RotateBehavior::update_rotation_angle(const geometry_msgs::msg::PoseStamped & msg)
{
  current_status_.current_yaw = as2::frame::getYawFromQuaternion(msg.pose.orientation);
}

void RotateBehavior::update_rotation_speed(const geometry_msgs::msg::TwistStamped & msg)
{
  current_status_.current_speed = msg.twist.angular.z;
}

bool RotateBehavior::check_finished()
{
  if (abs(desired_goal_.yaw - current_status_.current_yaw) < angle_threshold_) {
    RCLCPP_INFO(
      this->get_logger(), "RotateBehavior: goal reached, angle between vectors %f",
      desired_goal_.yaw - current_status_.current_yaw);
    return true;
  }

  return false;
}

}  // namespace rotate_behavior
