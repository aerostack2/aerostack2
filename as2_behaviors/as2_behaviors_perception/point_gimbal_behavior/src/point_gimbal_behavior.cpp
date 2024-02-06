/*!*******************************************************************************************
 *  \file       point_gimbal_behavior.cpp
 *  \brief      Point Gimbal behavior implementation file.
 *  \authors    Pedro Arias-Perez
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "point_gimbal_behavior.hpp"

PointGimbalBehavior::PointGimbalBehavior()
    : as2_behavior::BehaviorServer<as2_msgs::action::FollowReference>("point_gimbal_behavior") {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior created");

  // Add parameter with gimbal name
  // Add parameter with gimbal control mode
}

void PointGimbalBehavior::setup() {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior setup");
  gimbal_control_pub_ =
      this->create_publisher<as2_msgs::msg::GimbalControl>("platform/cam/gimbal_command", 10);
}

bool PointGimbalBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) {
  this->setup();
  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool PointGimbalBehavior::on_modify(
    std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Goal modified");
  return true;
}

bool PointGimbalBehavior::on_deactivate(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior cancelled");
  return true;
}

bool PointGimbalBehavior::on_pause(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior paused");
  return true;
}

bool PointGimbalBehavior::on_resume(const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior resumed");
  return true;
}

as2_behavior::ExecutionStatus PointGimbalBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> &goal,
    std::shared_ptr<as2_msgs::action::FollowReference::Feedback> &feedback_msg,
    std::shared_ptr<as2_msgs::action::FollowReference::Result> &result_msg) {
  as2_msgs::msg::GimbalControl cmd;
  cmd.control_mode            = as2_msgs::msg::GimbalControl::POSITION_MODE;
  cmd.control.header.stamp    = this->now();
  cmd.control.header.frame_id = "base_link";
  cmd.control.vector.x        = goal->target_pose.point.x;
  gimbal_control_pub_->publish(cmd);
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PointGimbalBehavior::on_execution_end(const as2_behavior::ExecutionStatus &status) {
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior execution ended");
  return;
}
