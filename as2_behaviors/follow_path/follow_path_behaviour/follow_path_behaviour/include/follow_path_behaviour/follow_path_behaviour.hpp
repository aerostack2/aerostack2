/*!*******************************************************************************************
 *  \file       follow_path_behaviour.hpp
 *  \brief      follow_path_behaviour header file
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

#ifndef FOLLOW_PATH_BEHAVIOUR_HPP
#define FOLLOW_PATH_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/follow_path.hpp>

#include <pluginlib/class_loader.hpp>
#include "follow_path_plugin_base/follow_path_base.hpp"

class FollowPathBehaviour : public as2::BasicBehaviour<as2_msgs::action::FollowPath> {
public:
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

  FollowPathBehaviour()
      : as2::BasicBehaviour<as2_msgs::action::FollowPath>(
            as2_names::actions::behaviours::followpath) {
    try {
      this->declare_parameter<std::string>("default_follow_path_plugin");
    } catch (const rclcpp::ParameterTypeException& e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_follow_path_plugin> not defined or malformed: %s",
                   e.what());
      this->~FollowPathBehaviour();
    }
    try {
      this->declare_parameter<double>("follow_path_threshold");
    } catch (const rclcpp::ParameterTypeException& e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <follow_path_threshold> not defined or malformed: %s",
                   e.what());
      this->~FollowPathBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<follow_path_base::FollowPathBase>>(
        "follow_path_plugin_base", "follow_path_base::FollowPathBase");

    try {
      std::string plugin_name = this->get_parameter("default_follow_path_plugin").as_string();
      plugin_name += "::Plugin";
      follow_path_ = loader_->createSharedInstance(plugin_name);
      follow_path_->initialize(this, this->get_parameter("follow_path_threshold").as_double());
      RCLCPP_INFO(this->get_logger(), "FOLLOW PATH PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~FollowPathBehaviour();
    }

    RCLCPP_DEBUG(this->get_logger(), "Follow Path Behaviour ready!");
  };

  ~FollowPathBehaviour(){};

  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    if (goal->trajectory_waypoints.poses.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory waypoints are empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return follow_path_->onAccepted(goal);
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    return follow_path_->onCancel(goal_handle);
  }

  void onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
    if (follow_path_->onExecute(goal_handle)) {
      RCLCPP_INFO(this->get_logger(), "Follow Path succeeded");
    } else {
      RCLCPP_WARN(this->get_logger(), "Follow Path canceled");
    }
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<follow_path_base::FollowPathBase>> loader_;
  std::shared_ptr<follow_path_base::FollowPathBase> follow_path_;
};

#endif  // FOLLOW_PATH_BEHAVIOUR_HPP