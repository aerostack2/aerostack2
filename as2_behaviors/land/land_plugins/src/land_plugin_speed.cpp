/*!*******************************************************************************************
 *  \file       land_plugin_speed.cpp
 *  \brief      This file contains the implementation of the land behaviour speed plugin
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

#include "as2_behavior/behavior_server.hpp"
#include "land_base.hpp"
#include "motion_reference_handlers/speed_motion.hpp"

namespace land_plugin_speed {

class Plugin : public land_base::LandBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::SpeedMotion> speed_motion_handler_ = nullptr;

public:
  void ownInit() {
    speed_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::SpeedMotion>(node_ptr_);
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land canceled, set to hover");
    sendHover();
    return true;
  }

  bool on_pause(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land paused");
    sendHover();
    return true;
  }

  bool on_resume(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land resumed");
    time_ = node_ptr_->now();
    return true;
  }

  bool own_activate(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", goal->land_speed);
    time_ = node_ptr_->now();
    return true;
  }

  bool own_modify(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land goal modified");
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", goal->land_speed);
    time_ = node_ptr_->now();
    return true;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.land_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!speed_motion_handler_->sendSpeedCommandWithYawSpeed("earth", 0.0, 0.0, goal_.land_speed,
                                                             0.0)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "LAND PLUGIN: Error sending speed command");
      result_.land_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land end");
    if (state != as2_behavior::ExecutionStatus::SUCCESS) {
      sendHover();
    }
    return;
  }

private:
  rclcpp::Time time_;
  bool checkGoalCondition() {
    if (fabs(feedback_.actual_land_speed) < 0.05) {
      if ((node_ptr_->now() - this->time_).seconds() > 2) {
        return true;
      }
    } else {
      time_ = node_ptr_->now();
    }
    return false;
  }

};  // Plugin class
}  // namespace land_plugin_speed

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(land_plugin_speed::Plugin, land_base::LandBase)
