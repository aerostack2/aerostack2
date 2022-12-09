/*!*******************************************************************************************
 *  \file       as2_basic_behaviour.hpp
 *  \brief      Aerostack2 basic behaviour virtual class header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#ifndef AS2_BASIC_BEHAVIOUR_HPP_
#define AS2_BASIC_BEHAVIOUR_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace as2 {
template <class MessageT>
class BasicBehaviour : public as2::Node {
public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<MessageT>;

  BasicBehaviour(const std::string &name) : Node(name) {
    this->action_server_ = rclcpp_action::create_server<MessageT>(
        this, this->generate_global_name(name),
        std::bind(&BasicBehaviour::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&BasicBehaviour::handleCancel, this, std::placeholders::_1),
        std::bind(&BasicBehaviour::handleAccepted, this, std::placeholders::_1));
  };

public:
  virtual rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const typename MessageT::Goal> goal) = 0;
  virtual rclcpp_action::CancelResponse onCancel(
      const std::shared_ptr<GoalHandleAction> goal_handle) = 0;
  virtual void onExecute(
      const std::shared_ptr<GoalHandleAction> goal_handle) = 0;  // return true when finished

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const typename MessageT::Goal> goal) {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request with UUID: %d", uuid);
    return onAccepted(goal);
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Request to cancel goal received");
    return onCancel(goal_handle);
  };

  // TODO: explore the use of Timers instead std::thread
  void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }

    execution_thread_ = std::thread(
        std::bind(&BasicBehaviour::onExecute, this, std::placeholders::_1), goal_handle);
  };

private:
  std::thread execution_thread_;
  typename rclcpp_action::Server<MessageT>::SharedPtr action_server_;

};  // BasicBehaviour class

}  // end namespace as2

#endif

