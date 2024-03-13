/*!*******************************************************************************************
 *  \file       position_motion.cpp
 *  \brief      This file contains the implementation of the take off behavior position plugin
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
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

#include <geometry_msgs/msg/point.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_motion_reference_handlers/position_motion.hpp"
#include "takeoff_behavior/takeoff_base.hpp"

namespace takeoff_plugin_position {

class Plugin : public takeoff_base::TakeoffBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool own_activate(as2_msgs::action::Takeoff::Goal& _goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff accepted");
    takeoff_position_.x = actual_pose_.pose.position.x;
    takeoff_position_.y = actual_pose_.pose.position.y;
    takeoff_position_.z = actual_pose_.pose.position.z + _goal.takeoff_height;
    takeoff_angle_      = as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff to position: %f, %f, %f", takeoff_position_.x,
                takeoff_position_.y, takeoff_position_.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with angle: %f", takeoff_angle_);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with speed: %f", _goal.takeoff_speed);
    return true;
  }

  bool own_modify(as2_msgs::action::Takeoff::Goal& _goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff accepted");
    takeoff_position_.x = actual_pose_.pose.position.x;
    takeoff_position_.y = actual_pose_.pose.position.y;
    takeoff_position_.z = actual_pose_.pose.position.z + _goal.takeoff_height;
    takeoff_angle_      = as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff to position: %f, %f, %f", takeoff_position_.x,
                takeoff_position_.y, takeoff_position_.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with angle: %f", takeoff_angle_);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with speed: %f", _goal.takeoff_speed);
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string>& message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff canceled, set to hover");
    sendHover();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus& state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      // Leave the drone in the last position
      if (position_motion_handler_->sendPositionCommandWithYawAngle(
              "earth", takeoff_position_.x, takeoff_position_.y, takeoff_position_.z,
              takeoff_angle_, "earth", goal_.takeoff_speed, goal_.takeoff_speed,
              goal_.takeoff_speed))
        return;
    }
    sendHover();
    return;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.takeoff_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
            "earth", takeoff_position_.x, takeoff_position_.y, takeoff_position_.z, takeoff_angle_,
            "earth", goal_.takeoff_speed, goal_.takeoff_speed, goal_.takeoff_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Take Off PLUGIN: Error sending speed command");
      result_.takeoff_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  geometry_msgs::msg::Point takeoff_position_;
  double takeoff_angle_;

  bool checkGoalCondition() {
    if (localization_flag_) {
      if (fabs(goal_.takeoff_height - feedback_.actual_takeoff_height) < params_.takeoff_threshold)
        return true;
    }
    return false;
  }

};  // Plugin class
}  // namespace takeoff_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_position::Plugin, takeoff_base::TakeoffBase)
