/*!*******************************************************************************************
 *  \file       go_to_emulator.hpp
 *  \brief      Go to emulator class definition
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

#ifndef FOLLOW_PATH_EMULATOR_HPP
#define FOLLOW_PATH_EMULATOR_HPP

#include "as2_core/as2_basic_behavior.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/follow_path.hpp"

class FollowPathBehaviorEmulator
    : public as2::BasicBehavior<as2_msgs::action::FollowPath> {
public:
  using GoalHandleFollowPath =
      rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

  FollowPathBehaviorEmulator()
      : as2::BasicBehavior<as2_msgs::action::FollowPath>(
            as2_names::actions::behaviors::followpath){

        };

  ~FollowPathBehaviorEmulator(){};

  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    // RCLCPP_INFO(this->get_logger(), "Going to %f, %f %f",
    //             goal->target_pose.point.x, goal->target_pose.point.y,
    //             goal->target_pose.point.z);
    RCLCPP_INFO(this->get_logger(), "Follow path executing");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    rclcpp::Rate sleep_rate(std::chrono::milliseconds(5000));
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "Follow Path IN PROGRESS: 25%%");
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "FOLLOW PATH IN PROGRESS: 50%%");
    sleep_rate.sleep();
    RCLCPP_INFO(this->get_logger(), "FOLLOWPATH IN PROGRESS: 75%%");
    sleep_rate.sleep();

    auto result = std::make_shared<as2_msgs::action::FollowPath::Result>();
    result->follow_path_success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "GO TO REACHED!!");
  }
};

#endif // FOLLOW_PATH_EMULATOR_HPP
