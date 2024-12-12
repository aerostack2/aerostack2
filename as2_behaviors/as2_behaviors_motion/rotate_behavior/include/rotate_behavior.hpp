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
 *  \file       rotate_behavior.hpp
 *  \brief      Rotate behavior header file.
 *  \authors    Pedro Arias-Pérez, Tomás Sánchez-Villaluenga
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef ROTATE_BEHAVIOR__ROTATE_BEHAVIOR_HPP_
#define ROTATE_BEHAVIOR__ROTATE_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/position_motion.hpp"

#include "as2_msgs/action/rotate.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rotate_behavior
{

class RotateBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Rotate>
{
public:
  explicit RotateBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~RotateBehavior() = default;

protected:
  bool on_activate(std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::Rotate::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Rotate::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Rotate::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Rotate::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  as2::tf::TfHandler tf_handler_;
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

  // Init parameters
  std::string base_link_frame_id_;
  double default_angle_;
  double default_speed_;
  double angle_threshold_;

  // Goal
  as2_msgs::action::Rotate::Goal desired_goal_;

  // Status
  as2_msgs::action::Rotate::Feedback current_status_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

protected:
  void update_rotation_angle(const geometry_msgs::msg::PoseStamped & msg);
  void update_rotation_speed(const geometry_msgs::msg::TwistStamped & msg);

  bool check_finished();
};
}  // namespace rotate_behavior

#endif  // ROTATE_BEHAVIOR__ROTATE_BEHAVIOR_HPP_
