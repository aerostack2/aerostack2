/*!*******************************************************************************************
 *  \file       land_behaviour.hpp
 *  \brief      Land behaviour class header file
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

#ifndef LAND_BEHAVIOUR_HPP
#define LAND_BEHAVIOUR_HPP

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "land_plugin_base/land_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <pluginlib/class_loader.hpp>

class LandBehaviour : public as2_behavior::BehaviorServer<as2_msgs::action::Land> {
public:
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;
  using PSME           = as2_msgs::msg::PlatformStateMachineEvent;

  LandBehaviour()
      : as2_behavior::BehaviorServer<as2_msgs::action::Land>(as2_names::actions::behaviours::land) {
    try {
      this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <plugin_name> not defined or "
                   "malformed: %s",
                   e.what());
      this->~LandBehaviour();
    }
    try {
      this->declare_parameter<double>("land_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <land_speed> not defined or "
                   "malformed: %s",
                   e.what());
      this->~LandBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<land_base::LandBase>>("land_plugin_base",
                                                                            "land_base::LandBase");

    tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

    try {
      std::string plugin_name = this->get_parameter("plugin_name").as_string();
      plugin_name += "::Plugin";
      land_plugin_ = loader_->createSharedInstance(plugin_name);

      land_base::land_plugin_params params;
      params.land_speed = this->get_parameter("land_speed").as_double();
      land_plugin_->initialize(this, tf_handler_, params);
      RCLCPP_INFO(this->get_logger(), "LAND BEHAVIOUR PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~LandBehaviour();
    }

    base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

    platform_disarm_cli_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::SetBool>>(
        as2_names::services::platform::set_arming_state, this);

    platform_land_cli_ = std::make_shared<
        as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
        as2_names::services::platform::set_platform_state_machine_event, this);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
        std::bind(&LandBehaviour::state_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "Land Behaviour ready!");
  };

  ~LandBehaviour(){};

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
      auto [pose_msg, twist_msg] =
          tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
      land_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
    return;
  }

  bool sendEventFSME(const int8_t _event) {
    as2_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
    as2_msgs::srv::SetPlatformStateMachineEvent::Response set_platform_fsm_resp;
    set_platform_fsm_req.event.event = _event;
    auto out = platform_land_cli_->sendRequest(set_platform_fsm_req, set_platform_fsm_resp, 3);
    if (out && set_platform_fsm_resp.success) return true;
    return false;
  }

  bool sendDisarm() {
    RCLCPP_INFO(this->get_logger(), "Disarming platform");
    std_srvs::srv::SetBool::Request set_platform_disarm_req;
    std_srvs::srv::SetBool::Response set_platform_disarm_resp;
    set_platform_disarm_req.data = false;
    auto out =
        platform_disarm_cli_->sendRequest(set_platform_disarm_req, set_platform_disarm_resp, 3);
    if (out && set_platform_disarm_resp.success) return true;
    return false;
  }

  bool process_goal(std::shared_ptr<const as2_msgs::action::Land::Goal> goal,
                    as2_msgs::action::Land::Goal &new_goal) {
    new_goal.land_speed = (goal->land_speed != 0.0f)
                              ? -fabs(goal->land_speed)
                              : -fabs(this->get_parameter("land_speed").as_double());

    if (!sendEventFSME(PSME::LAND)) {
      RCLCPP_ERROR(this->get_logger(), "LandBehaviour: Could not set FSM to land");
      return false;
    }
    return true;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override {
    as2_msgs::action::Land::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return land_plugin_->on_activate(
        std::make_shared<const as2_msgs::action::Land::Goal>(new_goal));
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override {
    as2_msgs::action::Land::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }
    return land_plugin_->on_modify(std::make_shared<const as2_msgs::action::Land::Goal>(new_goal));
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    return land_plugin_->on_deactivate(message);
  }

  bool on_pause(const std::shared_ptr<std::string> &message) override {
    return land_plugin_->on_pause(message);
  }

  bool on_resume(const std::shared_ptr<std::string> &message) override {
    return land_plugin_->on_resume(message);
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::Land::Goal> &goal,
      std::shared_ptr<as2_msgs::action::Land::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::Land::Result> &result_msg) override {
    return land_plugin_->on_run(goal, feedback_msg, result_msg);
  }

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override {
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "LandBehaviour: Land successful");
      if (!sendEventFSME(PSME::LANDED)) {
        RCLCPP_ERROR(this->get_logger(), "LandBehaviour: Could not set FSM to Landed");
      }
      if (!sendDisarm()) {
        RCLCPP_ERROR(this->get_logger(), "LandBehaviour: Could not disarm");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "LandBehaviour: Land failed");
      if (!sendEventFSME(PSME::EMERGENCY)) {
        RCLCPP_ERROR(this->get_logger(), "LandBehaviour: Could not set FSM to EMERGENCY");
      }
    }
    return land_plugin_->on_execution_end(state);
  }

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<land_base::LandBase>> loader_;
  std::shared_ptr<land_base::LandBase> land_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
      platform_land_cli_;
  as2::SynchronousServiceClient<std_srvs::srv::SetBool>::SharedPtr platform_disarm_cli_;
};

#endif  // GOTO_BEHAVIOUR_HPP
