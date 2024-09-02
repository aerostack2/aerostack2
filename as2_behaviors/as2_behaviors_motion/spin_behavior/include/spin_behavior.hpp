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
 *  \file       spin_behavior.hpp
 *  \brief      Spin behavior header file.
 *  \authors    Tomás Sánchez-Villaluenga, Pedro Arias-Pérez
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef SPIN_BEHAVIOR__SPIN_BEHAVIOR_HPP_
#define SPIN_BEHAVIOR__SPIN_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "as2_core/names/topics.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"

#include "as2_msgs/action/spin.hpp"
#include "as2_msgs/msg/spin_param.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace spin_behavior
{

class SpinBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Spin>
{
public:
  explicit SpinBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SpinBehavior() = default;

protected:
  bool on_activate(std::shared_ptr<const as2_msgs::action::Spin::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::Spin::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Spin::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Spin::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Spin::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  as2::tf::TfHandler tf_handler_;

  // Init parameters
  std::string base_link_frame_id_;
  std::string behavior_name_;
  double angle_threshold;
  rclcpp::Time goal_init_time_;
  rclcpp::Duration behavior_timeout_ = rclcpp::Duration(0, 0);

  // Goal
  as2_msgs::msg::SpinParam desired_goal_position_;

  // Status
  as2_msgs::msg::SpinParam current_goal_position_;
  float current_spin_position;
  float current_spin_speed;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

protected:
  void update_spin_angle(const geometry_msgs::msg::PoseStamped & msg);
  void update_spin_speed(const geometry_msgs::msg::TwistStamped & msg);

  bool update_spin_state();

  bool check_finished();
};
}  // namespace spin_behavior

#endif  // SPIN_BEHAVIOR__SPIN_BEHAVIOR_HPP_
