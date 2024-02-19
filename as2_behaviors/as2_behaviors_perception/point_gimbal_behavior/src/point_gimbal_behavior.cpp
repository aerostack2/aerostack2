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
 *  \authors    Pedro Arias-Perez
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "point_gimbal_behavior.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/frame_utils.hpp"

PointGimbalBehavior::PointGimbalBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::FollowReference>("PointGimbalBehavior")
{
  this->declare_parameter<std::string>("gimbal_name", "gimbal");
  this->get_parameter("gimbal_name", gimbal_name_);
  this->declare_parameter<std::string>("gimbal_control_mode", "position");
  this->get_parameter("gimbal_control_mode", gimbal_control_mode_);

  gimbal_control_pub_ = this->create_publisher<as2_msgs::msg::GimbalControl>(
    "platform/" + gimbal_name_ + "/gimbal_command", 10);

  gimbal_orientation_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "sensor_measurements/" + gimbal_name_ + "/attitude",
    as2_names::topics::sensor_measurements::qos,
    std::bind(&PointGimbalBehavior::gimbal_orientation_callback, this, std::placeholders::_1));

  gimbal_twist_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "sensor_measurements/" + gimbal_name_ + "/twist", as2_names::topics::sensor_measurements::qos,
    std::bind(&PointGimbalBehavior::gimbal_twist_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "PointGimbalBehavior created for gimbal %s in mode %s",
    gimbal_name_.c_str(), gimbal_control_mode_.c_str());
}

bool PointGimbalBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
{
  if (goal->target_pose.header.frame_id != "") {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: target frame_id must be empty");
    return false;
  }

  gimbal_control_msg_.control_mode = as2_msgs::msg::GimbalControl::POSITION_MODE;
  gimbal_control_msg_.control.header.stamp = this->now();
  gimbal_control_msg_.control.header.frame_id = "";  // FIXME
  gimbal_control_msg_.control.vector.x = goal->target_pose.point.x;
  gimbal_control_msg_.control.vector.y = goal->target_pose.point.y;
  gimbal_control_msg_.control.vector.z = goal->target_pose.point.z;
  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool PointGimbalBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
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
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior paused not available for this behavior");
  return false;
}

bool PointGimbalBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior resumed not available for this behavior");
  return false;
}

as2_behavior::ExecutionStatus PointGimbalBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> & goal,
  std::shared_ptr<as2_msgs::action::FollowReference::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::FollowReference::Result> & result_msg)
{
  if (compareAttitude(goal->target_pose.point, gimbal_status_.orientation)) {
    result_msg->follow_reference_success = true;
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  gimbal_control_pub_->publish(gimbal_control_msg_);
  // FIXME: this is not the actual speed
  feedback_msg->actual_speed =
    gimbal_status_.twist.x + gimbal_status_.twist.y + gimbal_status_.twist.z;
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PointGimbalBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior execution ended");
}

void PointGimbalBehavior::gimbal_orientation_callback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  double roll, pitch, yaw;
  as2::frame::quaternionToEuler(msg->quaternion, roll, pitch, yaw);

  gimbal_status_.orientation.x = roll;
  gimbal_status_.orientation.y = pitch;
  gimbal_status_.orientation.z = yaw;
}

void PointGimbalBehavior::gimbal_twist_callback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  gimbal_status_.twist = msg->vector;
}

bool PointGimbalBehavior::compareAttitude(
  const geometry_msgs::msg::Point & attitude1,
  const geometry_msgs::msg::Vector3 & attitude2)
{
  return fabs(attitude1.x - attitude2.x) < 0.01 && fabs(attitude1.y - attitude2.y) < 0.01 &&
         fabs(attitude1.z - attitude2.z) < 0.01;
}
