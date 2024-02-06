/*!*******************************************************************************************
 *  \file       point_gimbal_behavior.hpp
 *  \brief      Point Gimbal behavior header file.
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

#ifndef POINT_GIMBAL_BEHAVIOR_HPP_
#define POINT_GIMBAL_BEHAVIOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/node.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include "as2_msgs/action/follow_reference.hpp"

class PointGimbalBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::FollowReference> {
public:
  PointGimbalBehavior();

  ~PointGimbalBehavior(){};

private:
  void loadParameters();
  void setup();

private:
  bool on_activate(std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> &message) override;

  bool on_pause(const std::shared_ptr<std::string> &message) override;

  bool on_resume(const std::shared_ptr<std::string> &message) override;

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> &goal,
      std::shared_ptr<as2_msgs::action::FollowReference::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::FollowReference::Result> &result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override;
};

#endif  // POINT_GIMBAL_BEHAVIOR_HPP_
