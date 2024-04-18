/*!*******************************************************************************************
 *  \file       go_to_plugin_position.cpp
 *  \brief      This file contains the implementation of the go to behavior position plugin
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

#include "as2_motion_reference_handlers/position_motion.hpp"
#include "go_to_behavior/go_to_base.hpp"

namespace go_to_plugin_position {
class Plugin : public go_to_base::GoToBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool own_activate(as2_msgs::action::GoToWaypoint::Goal& _goal) override {
    if (!computeYaw(_goal.yaw.mode, _goal.target_pose.point, actual_pose_.pose.position,
                    _goal.yaw.angle)) {
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo goal accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to position: %f, %f, %f", _goal.target_pose.point.x,
                _goal.target_pose.point.y, _goal.target_pose.point.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to speed: %f", _goal.max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to angle: %f", _goal.yaw.angle);
    return true;
  }

  bool own_modify(as2_msgs::action::GoToWaypoint::Goal& _goal) override {
    if (!computeYaw(_goal.yaw.mode, _goal.target_pose.point, actual_pose_.pose.position,
                    _goal.yaw.angle)) {
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo goal modified");
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to position: %f, %f, %f", _goal.target_pose.point.x,
                _goal.target_pose.point.y, _goal.target_pose.point.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to speed: %f", _goal.max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo to angle: %f", _goal.yaw.angle);
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string>& message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string>& message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string>& message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo resumed");
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus& state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "GoTo end");
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

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.go_to_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
            "earth", goal_.target_pose.point.x, goal_.target_pose.point.y,
            goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed, goal_.max_speed,
            goal_.max_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "GOTO PLUGIN: Error sending position command");
      result_.go_to_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  bool checkGoalCondition() {
    if (localization_flag_) {
      if (fabs(feedback_.actual_distance_to_goal) < params_.go_to_threshold) return true;
    }
    return false;
  }

  inline float getActualYaw() {
    return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
  }

  bool computeYaw(const uint8_t yaw_mode,
                  const geometry_msgs::msg::Point& target,
                  const geometry_msgs::msg::Point& actual,
                  float& yaw) {
    switch (yaw_mode) {
      case as2_msgs::msg::YawMode::PATH_FACING: {
        Eigen::Vector2d diff(target.x - actual.x, target.y - actual.y);
        if (diff.norm() < 0.1) {
          RCLCPP_WARN(node_ptr_->get_logger(),
                      "Goal is too close to the current position in the plane, setting yaw_mode to "
                      "KEEP_YAW");
          yaw = getActualYaw();
        } else {
          yaw = as2::frame::getVector2DAngle(diff.x(), diff.y());
        }
      } break;
      case as2_msgs::msg::YawMode::FIXED_YAW:
        RCLCPP_INFO(node_ptr_->get_logger(), "Yaw mode FIXED_YAW");
        break;
      case as2_msgs::msg::YawMode::KEEP_YAW:
        RCLCPP_INFO(node_ptr_->get_logger(), "Yaw mode KEEP_YAW");
        yaw = getActualYaw();
        break;
      case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
        RCLCPP_INFO(node_ptr_->get_logger(), "Yaw mode YAW_FROM_TOPIC, not supported");
        return false;
        break;
      default:
        RCLCPP_ERROR(node_ptr_->get_logger(), "Yaw mode %d not supported", yaw_mode);
        return false;
        break;
    }
    return true;
  }

};  // Plugin class
}  // namespace go_to_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(go_to_plugin_position::Plugin, go_to_base::GoToBase)