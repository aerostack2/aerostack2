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
 * @file follow_path_behavior.hpp
 *
 * follow_path_behavior header file
 *
 * @authors Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#ifndef FOLLOW_PATH_BEHAVIOR__FOLLOW_PATH_BEHAVIOR_HPP_
#define FOLLOW_PATH_BEHAVIOR__FOLLOW_PATH_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "follow_path_base.hpp"

class FollowPathBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::FollowPath>
{
public:
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

  explicit FollowPathBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~FollowPathBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  bool process_goal(
    std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal,
    as2_msgs::action::FollowPath::Goal & new_goal);

  bool on_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> & goal,
    std::shared_ptr<as2_msgs::action::FollowPath::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::FollowPath::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<follow_path_base::FollowPathBase>> loader_;
  std::shared_ptr<follow_path_base::FollowPathBase> follow_path_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
};

#endif  // FOLLOW_PATH_BEHAVIOR__FOLLOW_PATH_BEHAVIOR_HPP_
