/*!*******************************************************************************************
 *  \file       takeoff_behaviour.hpp
 *  \brief      Takeoff behaviour class header file
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

#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/take_off.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "takeoff_plugin_base/takeoff_base.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TakeOffBehaviour : public as2_behavior::BehaviorServer<as2_msgs::action::TakeOff> {
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;
  using PSME              = as2_msgs::msg::PlatformStateMachineEvent;

  TakeOffBehaviour()
      : as2_behavior::BehaviorServer<as2_msgs::action::TakeOff>(
            as2_names::actions::behaviours::takeoff) {
    try {
      this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <plugin_name> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("takeoff_height");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <takeoff_height> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("takeoff_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <takeoff_speed> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("takeoff_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <takeoff_threshold> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>(
        "takeoff_plugin_base", "takeoff_base::TakeOffBase");

    tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

    try {
      std::string plugin_name = this->get_parameter("plugin_name").as_string();
      plugin_name += "::Plugin";
      takeoff_plugin_ = loader_->createSharedInstance(plugin_name);

      takeoff_base::takeoff_plugin_params params;
      params.takeoff_height    = this->get_parameter("takeoff_height").as_double();
      params.takeoff_speed     = this->get_parameter("takeoff_speed").as_double();
      params.takeoff_threshold = this->get_parameter("takeoff_threshold").as_double();

      takeoff_plugin_->initialize(this, tf_handler_, params);

      RCLCPP_INFO(this->get_logger(), "TAKEOFF BEHAVIOUR PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~TakeOffBehaviour();
    }

    base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

    platform_cli_ = std::make_shared<
        as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
        as2_names::services::platform::set_platform_state_machine_event, this);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
        std::bind(&TakeOffBehaviour::state_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "TakeOff Behaviour ready!");
  };

  ~TakeOffBehaviour(){};

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
      auto [pose_msg, twist_msg] =
          tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
      takeoff_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
    return;
  }

  bool sendEventFSME(const int8_t _event) {
    as2_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
    as2_msgs::srv::SetPlatformStateMachineEvent::Response set_platform_fsm_resp;
    set_platform_fsm_req.event.event = _event;
    auto out = platform_cli_->sendRequest(set_platform_fsm_req, set_platform_fsm_resp, 3);
    if (out && set_platform_fsm_resp.success) return true;
    return false;
  }

  bool process_goal(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal,
                    as2_msgs::action::TakeOff::Goal &new_goal) {
    if (goal->takeoff_height < 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff height");
      return false;
    }

    if (goal->takeoff_speed < 0.0f) {
      RCLCPP_WARN(this->get_logger(), "TakeOffBehaviour: Invalid takeoff speed, using default: %f",
                  this->get_parameter("takeoff_speed").as_double());
      return false;
    }
    new_goal.takeoff_speed = (goal->takeoff_speed != 0.0f)
                                 ? goal->takeoff_speed
                                 : this->get_parameter("takeoff_speed").as_double();

    if (!sendEventFSME(PSME::TAKE_OFF)) {
      RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Could not set FSM to takeoff");
      return false;
    }
    return true;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override {
    as2_msgs::action::TakeOff::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return takeoff_plugin_->on_activate(
        std::make_shared<const as2_msgs::action::TakeOff::Goal>(new_goal));
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override {
    as2_msgs::action::TakeOff::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return takeoff_plugin_->on_modify(
        std::make_shared<const as2_msgs::action::TakeOff::Goal>(new_goal));
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    return takeoff_plugin_->on_deactivate(message);
  }

  bool on_pause(const std::shared_ptr<std::string> &message) override {
    return takeoff_plugin_->on_pause(message);
  }

  bool on_resume(const std::shared_ptr<std::string> &message) override {
    return takeoff_plugin_->on_resume(message);
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> &goal,
      std::shared_ptr<as2_msgs::action::TakeOff::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::TakeOff::Result> &result_msg) override {
    return takeoff_plugin_->on_run(goal, feedback_msg, result_msg);
  }

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override {
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      if (!sendEventFSME(PSME::TOOK_OFF)) {
        RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Could not set FSM to Took OFF");
      }
    } else {
      if (!sendEventFSME(PSME::EMERGENCY)) {
        RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Could not set FSM to EMERGENCY");
      }
    }
    return takeoff_plugin_->on_execution_end(state);
  }

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
  std::shared_ptr<takeoff_base::TakeOffBase> takeoff_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
      platform_cli_;
};

#endif  // GOTO_BEHAVIOUR_HPP
