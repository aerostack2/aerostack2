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
 * @file goto_emulator.hpp
 *
 * Go to emulator class definition
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

#ifndef NODE_EMULATORS__GOTO_EMULATOR_HPP_
#define NODE_EMULATORS__GOTO_EMULATOR_HPP_

#include <memory>

#include "as2_core/as2_basic_behavior.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"

class GoTobehaviorEmulator
  : public as2::BasicBehavior<as2_msgs::action::GoToWaypoint>
{
public:
  using GoalHandleLand =
    rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

  GoTobehaviorEmulator()
  : as2::BasicBehavior<as2_msgs::action::GoToWaypoint>(
      as2_names::actions::behaviors::gotowaypoint)
  {}

  ~GoTobehaviorEmulator() {}

  rclcpp_action::GoalResponse onAccepted(
    const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(), "Going to %f, %f %f",
      goal->target_pose.point.x, goal->target_pose.point.y,
      goal->target_pose.point.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  onCancel(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onExecute(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    rclcpp::Rate sleep_rate(std::chrono::milliseconds(5000));
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "GO TO IN PROGRESS: 25%%");
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "GO TO IN PROGRESS: 50%%");
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "GO TO IN PROGRESS: 75%%");
    sleep_rate.sleep();

    auto result = std::make_shared<as2_msgs::action::GoToWaypoint::Result>();
    result->go_to_success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "GO TO REACHED!!");
  }
};

#endif  // NODE_EMULATORS__GOTO_EMULATOR_HPP_
