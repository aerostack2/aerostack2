/*!*******************************************************************************************
 *  \file       takeoff_plugin_platform.cpp
 *  \brief      This file contains the implementation of the take off behavior platform plugin
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

#include <chrono>
#include <std_srvs/srv/set_bool.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/services.hpp"
#include "takeoff_behavior/takeoff_base.hpp"

namespace takeoff_plugin_platform {

class Plugin : public takeoff_base::TakeoffBase {
public:
  void ownInit() {
    platform_takeoff_cli_ =
        node_ptr_->create_client<std_srvs::srv::SetBool>(as2_names::services::platform::takeoff);
    platform_takeoff_request_       = std::make_shared<std_srvs::srv::SetBool::Request>();
    platform_takeoff_request_->data = true;
    return;
  }

  bool own_activate(as2_msgs::action::Takeoff::Goal &_goal) override {
    using namespace std::chrono_literals;
    if (!platform_takeoff_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Platform takeoff service not available");
      return false;
    }

    platform_takeoff_future_ = platform_takeoff_cli_->async_send_request(platform_takeoff_request_);

    if (!platform_takeoff_future_.valid()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
      return false;
    }
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be cancelled");
    return false;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
    return;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (platform_takeoff_future_.valid() &&
        platform_takeoff_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = platform_takeoff_future_.get();
      if (result->success) {
        result_.takeoff_success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        result_.takeoff_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr platform_takeoff_cli_;

  std_srvs::srv::SetBool::Request::SharedPtr platform_takeoff_request_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture platform_takeoff_future_;

};  // Plugin class
}  // namespace takeoff_plugin_platform

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_platform::Plugin, takeoff_base::TakeoffBase)
