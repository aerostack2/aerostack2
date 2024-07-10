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

/**
* @file set_offboard_mode_behavior.hpp
*
* @brief Class definition for the set offboard mode behavior.
*
* @author Miguel Fernández Cortizas
*         Pedro Arias Pérez
*         David Pérez Saura
*         Rafael Pérez Seguí
*/

#ifndef AS2_BEHAVIORS_PLATFORM__SET_OFFBOARD_MODE_BEHAVIOR_HPP_
#define AS2_BEHAVIORS_PLATFORM__SET_OFFBOARD_MODE_BEHAVIOR_HPP_

#include <chrono>
#include <string>
#include <memory>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/services.hpp"
#include "as2_msgs/action/set_offboard_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"

class SetOffboardModeBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>
{
public:
  SetOffboardModeBehavior()
  : as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>(
      as2_names::services::platform::set_offboard_mode)
  {
    client_ = this->create_client<std_srvs::srv::SetBool>(
      as2_names::services::platform::set_offboard_mode);
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;

public:
  bool on_activate(std::shared_ptr<const as2_msgs::action::SetOffboardMode::Goal> goal) override
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

    req->data = goal->request;
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "service not available");
      return false;
    }
    future_ = client_->async_send_request(req).share();
    if (!future_.valid()) {
      RCLCPP_INFO(get_logger(), "request not sent");
      return false;
    }
    return true;
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::SetOffboardMode::Goal> goal) override
  {
    RCLCPP_WARN(get_logger(), "Cannot modify a service request");
    return false;
  }

  bool on_deactivate(const std::shared_ptr<std::string> & message) override
  {
    *message = "Unable to deactivate InstantBehavior";
    return false;
  }
  bool on_pause(const std::shared_ptr<std::string> & message) override
  {
    *message = "Unable to pause InstantBehavior";
    return false;
  }

  bool on_resume(const std::shared_ptr<std::string> & message) override
  {
    *message = "Unable to resume InstantBehavior";
    return false;
  }

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override {}

  as2_behavior::ExecutionStatus on_run(
    const typename std::shared_ptr<const as2_msgs::action::SetOffboardMode::Goal> & goal,
    typename std::shared_ptr<as2_msgs::action::SetOffboardMode::Feedback> & feedback_msg,
    typename std::shared_ptr<as2_msgs::action::SetOffboardMode::Result> & result_msg) override
  {
    if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = future_.get();
      result_msg->success = result->success;
      if (result->success) {
        result_msg->success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        result_msg->success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }
};

#endif  // AS2_BEHAVIORS_PLATFORM__SET_OFFBOARD_MODE_BEHAVIOR_HPP_
