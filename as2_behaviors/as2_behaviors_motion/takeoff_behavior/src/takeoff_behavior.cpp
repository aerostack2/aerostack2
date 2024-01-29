/*!*******************************************************************************************
 *  \file       takeoff_behavior.cpp
 *  \brief      Takeoff behavior class source file
 *  \authors    Rafael Pérez Seguí
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

#include "takeoff_behavior/takeoff_behavior.hpp"

TakeoffBehavior::TakeoffBehavior(const rclcpp::NodeOptions& options)
    : as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>(
          as2_names::actions::behaviors::takeoff,
          options) {
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <plugin_name> not defined or "
                 "malformed: %s",
                 e.what());
    this->~TakeoffBehavior();
  }
  try {
    this->declare_parameter<double>("takeoff_height");
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <takeoff_height> not defined or "
                 "malformed: %s",
                 e.what());
    this->~TakeoffBehavior();
  }
  try {
    this->declare_parameter<double>("takeoff_speed");
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <takeoff_speed> not defined or "
                 "malformed: %s",
                 e.what());
    this->~TakeoffBehavior();
  }
  try {
    this->declare_parameter<double>("takeoff_threshold");
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <takeoff_threshold> not defined or "
                 "malformed: %s",
                 e.what());
    this->~TakeoffBehavior();
  }
  try {
    this->declare_parameter<double>("tf_timeout_threshold");
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <tf_timeout_threshold> not defined or malformed: %s", e.what());
    this->~TakeoffBehavior();
  }

  loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeoffBase>>(
      "as2_behaviors_motion", "takeoff_base::TakeoffBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    takeoff_plugin_ = loader_->createSharedInstance(plugin_name);

    takeoff_base::takeoff_plugin_params params;
    params.takeoff_height       = this->get_parameter("takeoff_height").as_double();
    params.takeoff_speed        = this->get_parameter("takeoff_speed").as_double();
    params.takeoff_threshold    = this->get_parameter("takeoff_threshold").as_double();
    params.tf_timeout_threshold = this->get_parameter("tf_timeout_threshold").as_double();
    tf_timeout                  = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params.tf_timeout_threshold));

    takeoff_plugin_->initialize(this, tf_handler_, params);

    RCLCPP_INFO(this->get_logger(), "TAKEOFF BEHAVIOR PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                 ex.what());
    this->~TakeoffBehavior();
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_cli_ =
      std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
          as2_names::services::platform::set_platform_state_machine_event, this);

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
      std::bind(&TakeoffBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "Takeoff Behavior ready!");
};

TakeoffBehavior::~TakeoffBehavior(){};

void TakeoffBehavior::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    auto [pose_msg, twist_msg] =
        tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_, tf_timeout);
    takeoff_plugin_->state_callback(pose_msg, twist_msg);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

bool TakeoffBehavior::sendEventFSME(const int8_t _event) {
  as2_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
  as2_msgs::srv::SetPlatformStateMachineEvent::Response set_platform_fsm_resp;
  set_platform_fsm_req.event.event = _event;
  auto out = platform_cli_->sendRequest(set_platform_fsm_req, set_platform_fsm_resp, 3);
  if (out && set_platform_fsm_resp.success) return true;
  return false;
}

bool TakeoffBehavior::process_goal(std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal,
                                   as2_msgs::action::Takeoff::Goal& new_goal) {
  if (goal->takeoff_height < 0.0f) {
    RCLCPP_ERROR(this->get_logger(), "TakeoffBehavior: Invalid takeoff height");
    return false;
  }

  if (goal->takeoff_speed < 0.0f) {
    RCLCPP_WARN(this->get_logger(), "TakeoffBehavior: Invalid takeoff speed, using default: %f",
                this->get_parameter("takeoff_speed").as_double());
    return false;
  }
  new_goal.takeoff_speed = (goal->takeoff_speed != 0.0f)
                               ? goal->takeoff_speed
                               : this->get_parameter("takeoff_speed").as_double();

  if (!sendEventFSME(PSME::TAKE_OFF)) {
    RCLCPP_ERROR(this->get_logger(), "TakeoffBehavior: Could not set FSM to takeoff");
    return false;
  }
  return true;
}

bool TakeoffBehavior::on_activate(std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal) {
  as2_msgs::action::Takeoff::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return takeoff_plugin_->on_activate(
      std::make_shared<const as2_msgs::action::Takeoff::Goal>(new_goal));
}

bool TakeoffBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal) {
  as2_msgs::action::Takeoff::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return takeoff_plugin_->on_modify(
      std::make_shared<const as2_msgs::action::Takeoff::Goal>(new_goal));
}

bool TakeoffBehavior::on_deactivate(const std::shared_ptr<std::string>& message) {
  return takeoff_plugin_->on_deactivate(message);
}

bool TakeoffBehavior::on_pause(const std::shared_ptr<std::string>& message) {
  return takeoff_plugin_->on_pause(message);
}

bool TakeoffBehavior::on_resume(const std::shared_ptr<std::string>& message) {
  return takeoff_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus TakeoffBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::Takeoff::Goal>& goal,
    std::shared_ptr<as2_msgs::action::Takeoff::Feedback>& feedback_msg,
    std::shared_ptr<as2_msgs::action::Takeoff::Result>& result_msg) {
  return takeoff_plugin_->on_run(goal, feedback_msg, result_msg);
}

void TakeoffBehavior::on_execution_end(const as2_behavior::ExecutionStatus& state) {
  if (state == as2_behavior::ExecutionStatus::SUCCESS) {
    if (!sendEventFSME(PSME::TOOK_OFF)) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffBehavior: Could not set FSM to Took OFF");
    }
  } else {
    if (!sendEventFSME(PSME::EMERGENCY)) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffBehavior: Could not set FSM to EMERGENCY");
    }
  }
  return takeoff_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(TakeoffBehavior)