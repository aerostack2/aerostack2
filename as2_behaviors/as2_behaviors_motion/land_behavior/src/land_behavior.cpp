/*!*******************************************************************************************
 *  \file       land_behavior.cpp
 *  \brief      land_behavior file
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

#include "land_behavior/land_behavior.hpp"

LandBehavior::LandBehavior(const rclcpp::NodeOptions &options)
    : as2_behavior::BehaviorServer<as2_msgs::action::Land>(as2_names::actions::behaviors::land,
                                                           options) {
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException &e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <plugin_name> not defined or "
                 "malformed: %s",
                 e.what());
    this->~LandBehavior();
  }
  try {
    this->declare_parameter<double>("land_speed");
  } catch (const rclcpp::ParameterTypeException &e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <land_speed> not defined or "
                 "malformed: %s",
                 e.what());
    this->~LandBehavior();
  }
  try {
    this->declare_parameter<double>("tf_timeout_threshold");
  } catch (const rclcpp::ParameterTypeException &e) {
    RCLCPP_FATAL(this->get_logger(),
                 "Launch argument <tf_timeout_threshold> not defined or malformed: %s", e.what());
    this->~LandBehavior();
  }

  loader_ = std::make_shared<pluginlib::ClassLoader<land_base::LandBase>>("as2_behaviors_motion",
                                                                          "land_base::LandBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    land_plugin_ = loader_->createSharedInstance(plugin_name);

    land_base::land_plugin_params params;
    params.land_speed           = this->get_parameter("land_speed").as_double();
    params.tf_timeout_threshold = this->get_parameter("tf_timeout_threshold").as_double();
    tf_timeout                  = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params.tf_timeout_threshold));

    land_plugin_->initialize(this, tf_handler_, params);
    RCLCPP_INFO(this->get_logger(), "LAND BEHAVIOR PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                 ex.what());
    this->~LandBehavior();
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_disarm_cli_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::SetBool>>(
      as2_names::services::platform::set_arming_state, this);

  platform_land_cli_ =
      std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>>(
          as2_names::services::platform::set_platform_state_machine_event, this);

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
      std::bind(&LandBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "Land Behavior ready!");
};

LandBehavior::~LandBehavior(){};

void LandBehavior::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    auto [pose_msg, twist_msg] =
        tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_, tf_timeout);
    land_plugin_->state_callback(pose_msg, twist_msg);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

bool LandBehavior::sendEventFSME(const int8_t _event) {
  as2_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
  as2_msgs::srv::SetPlatformStateMachineEvent::Response set_platform_fsm_resp;
  set_platform_fsm_req.event.event = _event;
  auto out = platform_land_cli_->sendRequest(set_platform_fsm_req, set_platform_fsm_resp, 3);
  if (out && set_platform_fsm_resp.success) return true;
  return false;
}

bool LandBehavior::sendDisarm() {
  RCLCPP_INFO(this->get_logger(), "Disarming platform");
  std_srvs::srv::SetBool::Request set_platform_disarm_req;
  std_srvs::srv::SetBool::Response set_platform_disarm_resp;
  set_platform_disarm_req.data = false;
  auto out =
      platform_disarm_cli_->sendRequest(set_platform_disarm_req, set_platform_disarm_resp, 3);
  if (out && set_platform_disarm_resp.success) return true;
  return false;
}

bool LandBehavior::process_goal(std::shared_ptr<const as2_msgs::action::Land::Goal> goal,
                                as2_msgs::action::Land::Goal &new_goal) {
  new_goal.land_speed = (goal->land_speed != 0.0f)
                            ? -fabs(goal->land_speed)
                            : -fabs(this->get_parameter("land_speed").as_double());

  if (!sendEventFSME(PSME::LAND)) {
    RCLCPP_ERROR(this->get_logger(), "LandBehavior: Could not set FSM to land");
    return false;
  }
  return true;
}

bool LandBehavior::on_activate(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) {
  as2_msgs::action::Land::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return land_plugin_->on_activate(std::make_shared<const as2_msgs::action::Land::Goal>(new_goal));
}

bool LandBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) {
  as2_msgs::action::Land::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return land_plugin_->on_modify(std::make_shared<const as2_msgs::action::Land::Goal>(new_goal));
}

bool LandBehavior::on_deactivate(const std::shared_ptr<std::string> &message) {
  return land_plugin_->on_deactivate(message);
}

bool LandBehavior::on_pause(const std::shared_ptr<std::string> &message) {
  return land_plugin_->on_pause(message);
}

bool LandBehavior::on_resume(const std::shared_ptr<std::string> &message) {
  return land_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus LandBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::Land::Goal> &goal,
    std::shared_ptr<as2_msgs::action::Land::Feedback> &feedback_msg,
    std::shared_ptr<as2_msgs::action::Land::Result> &result_msg) {
  return land_plugin_->on_run(goal, feedback_msg, result_msg);
}

void LandBehavior::on_execution_end(const as2_behavior::ExecutionStatus &state) {
  if (state == as2_behavior::ExecutionStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "LandBehavior: Land successful");
    if (!sendEventFSME(PSME::LANDED)) {
      RCLCPP_ERROR(this->get_logger(), "LandBehavior: Could not set FSM to Landed");
    }
    if (!sendDisarm()) {
      RCLCPP_ERROR(this->get_logger(), "LandBehavior: Could not disarm");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "LandBehavior: Land failed");
    if (!sendEventFSME(PSME::EMERGENCY)) {
      RCLCPP_ERROR(this->get_logger(), "LandBehavior: Could not set FSM to EMERGENCY");
    }
  }
  return land_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(LandBehavior)