/*!*******************************************************************************************
 *  \file       goto_plugin_position.cpp
 *  \brief      This file contains the implementation of the go to behaviour position plugin
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
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

#include "goto_base.hpp"
#include "motion_reference_handlers/position_motion.hpp"

namespace goto_plugin_position {
class Plugin : public goto_base::GotoBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    return true;
  }

  bool on_pause(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto paused");
    sendHover();
    return true;
  }

  bool on_resume(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto resumed");
    return true;
  }

  bool own_activate(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto goal accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to position: %f, %f, %f", goal->target_pose.point.x,
                goal->target_pose.point.y, goal->target_pose.point.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to speed: %f", goal->max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to angle: %f", goal->yaw.angle);
    return true;
  }

  bool own_modify(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto goal modified");
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to position: %f, %f, %f", goal->target_pose.point.x,
                goal->target_pose.point.y, goal->target_pose.point.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to speed: %f", goal->max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto to angle: %f", goal->yaw.angle);
    return true;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.goto_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
            "earth", goal_.target_pose.point.x, goal_.target_pose.point.y,
            goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed, goal_.max_speed,
            goal_.max_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "GOTO PLUGIN: Error sending position command");
      result_.goto_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goto end");
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      // Leave the drone in the last position
      if (position_motion_handler_->sendPositionCommandWithYawAngle(
              "earth", goal_.target_pose.point.x, goal_.target_pose.point.y,
              goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed, goal_.max_speed,
              goal_.max_speed))
        return;
    }
    sendHover();
    return;
  }

private:
  bool checkGoalCondition() {
    if (distance_measured_) {
      if (fabs(feedback_.actual_distance_to_goal) < params_.goto_threshold) return true;
    }
    return false;
  }

};  // Plugin class
}  // namespace goto_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(goto_plugin_position::Plugin, goto_base::GotoBase)
