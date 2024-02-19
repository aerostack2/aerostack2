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

/*!*******************************************************************************************
 *  \file       point_gimbal_behavior.hpp
 *  \brief      Point Gimbal behavior header file.
 *  \authors    Pedro Arias-Perez
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef POINT_GIMBAL_BEHAVIOR__POINT_GIMBAL_BEHAVIOR_HPP_
#define POINT_GIMBAL_BEHAVIOR__POINT_GIMBAL_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/node.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include "as2_msgs/action/follow_reference.hpp"
#include "as2_msgs/msg/gimbal_control.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

struct gimbal_status
{
  geometry_msgs::msg::Vector3 twist;
  geometry_msgs::msg::Vector3 orientation;
};

class PointGimbalBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::FollowReference>
{
public:
  PointGimbalBehavior();

  ~PointGimbalBehavior() {}

private:
  bool on_activate(std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> & goal,
    std::shared_ptr<as2_msgs::action::FollowReference::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::FollowReference::Result> & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  std::string gimbal_name_;
  std::string gimbal_control_mode_;
  gimbal_status gimbal_status_;
  as2_msgs::msg::GimbalControl gimbal_control_msg_;

  rclcpp::Publisher<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_control_pub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr gimbal_orientation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_twist_sub_;

private:
  void gimbal_orientation_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void gimbal_twist_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  // AUX
  bool compareAttitude(
    const geometry_msgs::msg::Point & attitude1,
    const geometry_msgs::msg::Vector3 & attitude2);
};

#endif  // POINT_GIMBAL_BEHAVIOR__POINT_GIMBAL_BEHAVIOR_HPP_
