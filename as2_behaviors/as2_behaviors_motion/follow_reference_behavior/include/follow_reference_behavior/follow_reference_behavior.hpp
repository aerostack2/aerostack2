/*!*******************************************************************************************
 *  \file       follow_reference_behavior.hpp
 *  \brief      Follow reference behavior class header file
 *  \authors    Javier Melero Deza
 *              Rafael Pérez Seguí
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

#ifndef FOLLOW_REFERENCE_BEHAVIOR_HPP
#define FOLLOW_REFERENCE_BEHAVIOR_HPP

#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/position_motion.hpp"
#include "as2_msgs/action/follow_reference.hpp"
#include "as2_msgs/msg/platform_info.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/utils/frame_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/msg/platform_status.hpp"

class FollowReferenceBehavior
    : public as2_behavior::BehaviorServer<as2_msgs::action::FollowReference> {
public:
  using GoalHandleFollowReference =
      rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowReference>;

  FollowReferenceBehavior(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~FollowReferenceBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  bool process_goal(std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal,
                    as2_msgs::action::FollowReference::Goal &new_goal);

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

private:
  inline void sendHover();

  inline float getActualYaw();

  bool getState();

  bool computeYaw(const uint8_t yaw_mode,
                  const geometry_msgs::msg::Point &target,
                  const geometry_msgs::msg::Point &actual,
                  float &yaw);

  bool checkGoal(as2_msgs::action::FollowReference::Goal &_goal);

private:
  geometry_msgs::msg::PoseStamped actual_pose_;
  geometry_msgs::msg::TwistStamped actual_twist;

  std::string base_link_frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::chrono::nanoseconds tf_timeout;

  as2_msgs::action::FollowReference::Goal goal_;
  as2_msgs::action::FollowReference::Feedback feedback_;
  as2_msgs::action::FollowReference::Result result_;

  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_       = nullptr;

  int platform_state_;
  bool localization_flag_ = false;
};

#endif