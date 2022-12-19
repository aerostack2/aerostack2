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

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "follow_path_plugin_base/follow_path_base.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class FollowPathBehaviour : public as2_behavior::BehaviorServer<as2_msgs::action::FollowPath> {
public:
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

  FollowPathBehaviour()
      : as2_behavior::BehaviorServer<as2_msgs::action::FollowPath>(
            as2_names::actions::behaviours::followpath) {
    try {
      this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
                   e.what());
      this->~FollowPathBehaviour();
    }
    try {
      this->declare_parameter<double>("follow_path_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <follow_path_speed> not defined or "
                   "malformed: %s",
                   e.what());
      this->~FollowPathBehaviour();
    }
    try {
      this->declare_parameter<double>("follow_path_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <follow_path_threshold> not defined or malformed: %s",
                   e.what());
      this->~FollowPathBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<follow_path_base::FollowPathBase>>(
        "follow_path_plugin_base", "follow_path_base::FollowPathBase");

    tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

    try {
      std::string plugin_name = this->get_parameter("plugin_name").as_string();
      plugin_name += "::Plugin";
      follow_path_plugin_ = loader_->createSharedInstance(plugin_name);

      follow_path_base::follow_path_plugin_params params;
      params.follow_path_speed     = this->get_parameter("follow_path_speed").as_double();
      params.follow_path_threshold = this->get_parameter("follow_path_threshold").as_double();

      follow_path_plugin_->initialize(this, tf_handler_, params);

      RCLCPP_INFO(this->get_logger(), "FOLLOW PATH PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~FollowPathBehaviour();
    }

    base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

    platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
        as2_names::topics::platform::info, as2_names::topics::platform::qos,
        std::bind(&FollowPathBehaviour::platform_info_callback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
        std::bind(&FollowPathBehaviour::state_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "FollowPath Behaviour ready!");
  };

  ~FollowPathBehaviour(){};

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
      auto [pose_msg, twist_msg] =
          tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
      follow_path_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
    return;
  }

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
    follow_path_plugin_->platform_info_callback(msg);
    return;
  }

  bool process_goal(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal,
                    as2_msgs::action::FollowPath::Goal &new_goal) {
    if (goal->header.frame_id == "") {
      RCLCPP_ERROR(this->get_logger(), "Path frame_id is empty");
      return false;
    }

    if (goal->path.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Path is empty");
      return false;
    }

    if (goal->header.frame_id != "earth") {
      std::vector<as2_msgs::msg::PoseWithID> path_converted;
      path_converted.reserve(goal->path.size());

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = goal->header;
      for (as2_msgs::msg::PoseWithID waypoint : goal->path) {
        pose_msg.pose = waypoint.pose;
        if (!tf_handler_->tryConvert(pose_msg, "earth")) {
          RCLCPP_ERROR(this->get_logger(), "GotoBehaviour: can not get waypoint in earth frame");
          return false;
        }
        waypoint.pose = pose_msg.pose;
        path_converted.push_back(waypoint);
      }
      new_goal.header.frame_id = "earth";
      new_goal.path            = path_converted;
    }

    new_goal.max_speed = (goal->max_speed != 0.0f)
                             ? goal->max_speed
                             : this->get_parameter("follow_path_speed").as_double();

    return true;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    as2_msgs::action::FollowPath::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return follow_path_plugin_->on_activate(
        std::make_shared<const as2_msgs::action::FollowPath::Goal>(new_goal));
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    as2_msgs::action::FollowPath::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return follow_path_plugin_->on_modify(
        std::make_shared<const as2_msgs::action::FollowPath::Goal>(new_goal));
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    return follow_path_plugin_->on_deactivate(message);
  }

  bool on_pause(const std::shared_ptr<std::string> &message) override {
    return follow_path_plugin_->on_pause(message);
  }

  bool on_resume(const std::shared_ptr<std::string> &message) override {
    return follow_path_plugin_->on_resume(message);
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> &goal,
      std::shared_ptr<as2_msgs::action::FollowPath::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::FollowPath::Result> &result_msg) override {
    return follow_path_plugin_->on_run(goal, feedback_msg, result_msg);
  }

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override {
    return follow_path_plugin_->on_execution_end(state);
  }

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<follow_path_base::FollowPathBase>> loader_;
  std::shared_ptr<follow_path_base::FollowPathBase> follow_path_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
};

#endif  // FOLLOW_PATH_BEHAVIOUR_HPP
