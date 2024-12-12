// Copyright 2023 Universidad Politécnica de Madrid
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
 *  \file       aerial_platform.cpp
 *  \brief      Aerostack2 Aerial Platformm class implementation file.
 *  \authors    Miguel Fernandez Cortizas
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 ********************************************************************************/

#include "as2_core/aerial_platform.hpp"

namespace as2
{


void AerialPlatform::initialize()
{
  {
    resetPlatform();
    this->declare_parameter<float>("cmd_freq", 100.0);
    this->declare_parameter<float>("info_freq", 10.0);

    try {
      this->declare_parameter<std::string>("control_modes_file");
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Launch argument <control_modes_file> not defined or malformed: %s",
        e.what());
      this->~AerialPlatform();
    }

    this->get_parameter("cmd_freq", cmd_freq_);
    this->get_parameter("info_freq", info_freq_);

    std::string control_modes_file;
    this->get_parameter("control_modes_file", control_modes_file);

    this->loadControlModes(control_modes_file);

    trajectory_command_sub_ = this->create_subscription<as2_msgs::msg::TrajectorySetpoints>(
      this->generate_global_name(as2_names::topics::actuator_command::trajectory),
      as2_names::topics::actuator_command::qos,
      [this](const as2_msgs::msg::TrajectorySetpoints::ConstSharedPtr msg) {
        this->command_trajectory_msg_ = *msg.get();
        has_new_references_ = true;
      });
    pose_command_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->generate_global_name(as2_names::topics::actuator_command::pose),
      as2_names::topics::actuator_command::qos,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        this->command_pose_msg_ = *msg.get();
        has_new_references_ = true;
      });

    twist_command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name(as2_names::topics::actuator_command::twist),
      as2_names::topics::actuator_command::qos,
      [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        this->command_twist_msg_ = *msg.get();
        has_new_references_ = true;
      });
    thrust_command_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
      this->generate_global_name(as2_names::topics::actuator_command::thrust),
      as2_names::topics::actuator_command::qos,
      [this](const as2_msgs::msg::Thrust::ConstSharedPtr msg) {
        this->command_thrust_msg_ = *msg.get();
        has_new_references_ = true;
      });

    alert_event_sub_ = this->create_subscription<as2_msgs::msg::AlertEvent>(
      this->generate_global_name(as2_names::topics::global::alert_event),
      as2_names::topics::global::qos,
      std::bind(&AerialPlatform::alertEventCallback, this, std::placeholders::_1));

    set_platform_mode_srv_ = this->create_service<as2_msgs::srv::SetControlMode>(
      as2_names::services::platform::set_platform_control_mode,
      std::bind(
        &AerialPlatform::setPlatformControlModeSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    set_arming_state_srv_ = this->create_service<std_srvs::srv::SetBool>(
      as2_names::services::platform::set_arming_state,
      std::bind(
        &AerialPlatform::setArmingStateSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    set_offboard_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
      as2_names::services::platform::set_offboard_mode,
      std::bind(
        &AerialPlatform::setOffboardModeSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    platform_takeoff_srv_ = this->create_service<std_srvs::srv::SetBool>(
      as2_names::services::platform::takeoff,
      std::bind(
        &AerialPlatform::platformTakeoffSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    platform_land_srv_ = this->create_service<std_srvs::srv::SetBool>(
      as2_names::services::platform::land,
      std::bind(
        &AerialPlatform::platformLandSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    list_control_modes_srv_ = this->create_service<as2_msgs::srv::ListControlModes>(
      as2_names::services::platform::list_control_modes,
      std::bind(
        &AerialPlatform::listControlModesSrvCall, this,
        std::placeholders::_1,  // Corresponds to the 'request'  input
        std::placeholders::_2   // Corresponds to the 'response' input
    ));

    platform_info_pub_ = this->create_publisher<as2_msgs::msg::PlatformInfo>(
      this->generate_global_name(as2_names::topics::platform::info),
      as2_names::topics::platform::qos);

    platform_info_timer_ = this->create_timer(
      std::chrono::duration<double>(1.0f / info_freq_),
      std::bind(&AerialPlatform::publishPlatformInfo, this));

    platform_cmd_timer_ = this->create_timer(
      std::chrono::duration<double>(1.0f / cmd_freq_),
      std::bind(&AerialPlatform::sendCommand, this));
  }
}

AerialPlatform::AerialPlatform(const rclcpp::NodeOptions & options)
: as2::Node(std::string("platform"), options), state_machine_(as2::PlatformStateMachine(this))
{
  initialize();
}
AerialPlatform::AerialPlatform(const std::string & ns, const rclcpp::NodeOptions & options)
: as2::Node(std::string("platform"), ns, options), state_machine_(as2::PlatformStateMachine(this))
{
  initialize();
}

void AerialPlatform::resetPlatform()
{
  platform_info_msg_.armed = false;
  platform_info_msg_.offboard = false;
  platform_info_msg_.connected = true;  // TODO(miferco97): Check if connected
  // TODO(miferco97): won't work if current control mode dismatches takeoff control mode
  // platform_info_msg_.current_control_mode.control_mode = as2_msgs::msg::ControlMode::UNSET;
  state_machine_.setState(as2_msgs::msg::PlatformStatus::DISARMED);
}

bool AerialPlatform::setArmingState(bool state)
{
  if (state == platform_info_msg_.armed && state == true) {
    RCLCPP_WARN(this->get_logger(), "UAV is already armed");
  } else if (state == platform_info_msg_.armed && state == false) {
    RCLCPP_WARN(this->get_logger(), "UAV is already disarmed");
  } else {
    if (ownSetArmingState(state)) {
      platform_info_msg_.armed = state;
      if (state) {
        handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
      } else {
        handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::DISARM);
      }
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "Unable to set arming state %s", state ? "ON" : "OFF");
  }
  return false;
}

bool AerialPlatform::setOffboardControl(bool offboard)
{
  if (offboard == platform_info_msg_.offboard && offboard == true) {
    RCLCPP_WARN(this->get_logger(), "UAV is already in OFFBOARD mode");
  } else if (offboard == platform_info_msg_.offboard && offboard == false) {
    RCLCPP_WARN(this->get_logger(), "UAV is already in MANUAL mode");
  } else {
    if (ownSetOffboardControl(offboard)) {
      platform_info_msg_.offboard = offboard;
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "Unable to set offboard mode %s", offboard ? "ON" : "OFF");
  }
  return false;
}

bool AerialPlatform::setPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  if (ownSetPlatformControlMode(msg)) {
    if (msg.control_mode == as2_msgs::msg::ControlMode::HOVER ||
      msg.control_mode == as2_msgs::msg::ControlMode::UNSET)
    {
      has_new_references_ = true;
    } else {
      has_new_references_ = false;
    }
    platform_info_msg_.current_control_mode = msg;
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Unable to set control mode %d", msg.control_mode);
  return false;
}

bool AerialPlatform::takeoff()
{
  // TODO(miferco97): Implement STATE MACHINE check
  if (ownTakeoff()) {
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF);
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Unable to takeoff");
  return false;
}

bool AerialPlatform::land()
{
  // TODO(miferco97): Implement STATE MACHINE check
  if (ownLand()) {
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::LANDED);
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Unable to land");
  return false;
}

void AerialPlatform::alertEvent(const as2_msgs::msg::AlertEvent & msg)
{
  if (msg.alert > 0) {
    return;
  }
  if (!msg.description.empty()) {
    RCLCPP_WARN(this->get_logger(), "Alert event received: %s", msg.description.c_str());
  }
  switch (msg.alert) {
    case as2_msgs::msg::AlertEvent::KILL_SWITCH: {
        state_machine_.processEvent(as2_msgs::msg::PlatformStateMachineEvent::EMERGENCY);
        RCLCPP_WARN(this->get_logger(), "KILL SWITCH ACTIVATED");
        ownKillSwitch();
      } break;
    case as2_msgs::msg::AlertEvent::EMERGENCY_HOVER: {
        state_machine_.processEvent(as2_msgs::msg::PlatformStateMachineEvent::EMERGENCY);
        RCLCPP_WARN(this->get_logger(), "EMERGENCY HOVER ACTIVATED");
        ownStopPlatform();
      } break;
    default:
      break;
  }
}

void AerialPlatform::sendCommand()
{
  auto & clk = *this->get_clock();
  if (!isControlModeSettled()) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), clk, 5000, "Platform control mode is not settled yet");
    return;
  }
  if (!getConnectedStatus()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 5000, "Platform is not connected");
    return;
  } else if (!getArmingState()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 5000, "Platform is not armed yet");
    return;
  } else if (!getOffboardMode()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 5000, "Platform is not in offboard mode");
    return;
  }

  if (state_machine_.getState().state == as2_msgs::msg::PlatformStatus::EMERGENCY) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "SEND PLATFORM STOP COMMAND");
    ownStopPlatform();
  } else if (has_new_references_) {
    if (!ownSendCommand()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 5000, "Platform command failed");
    }
  }
}  // namespace as2

void AerialPlatform::loadControlModes(const std::string & filename)
{
  std::vector<std::string> modes = as2::yaml::find_tag_in_yaml_file(filename, "available_modes");

  for (std::vector<std::string>::iterator it = modes.begin(); it != modes.end(); ++it) {
    uint8_t m = as2::yaml::parse_uint_from_string(it->c_str());
    as2::control_mode::printControlMode(m);
    available_control_modes_.emplace_back(m);
  }
}

// Services Callbacks
void AerialPlatform::setPlatformControlModeSrvCall(
  const std::shared_ptr<as2_msgs::srv::SetControlMode::Request> request,
  std::shared_ptr<as2_msgs::srv::SetControlMode::Response> response)
{
  response->success = setPlatformControlMode(request->control_mode);
}

void AerialPlatform::setOffboardModeSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setOffboardControl(request->data);
}

void AerialPlatform::setArmingStateSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setArmingState(request->data);
}

void AerialPlatform::platformTakeoffSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request;
  response->success = takeoff();
}

void AerialPlatform::platformLandSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request;
  response->success = land();
}

void AerialPlatform::listControlModesSrvCall(
  const std::shared_ptr<as2_msgs::srv::ListControlModes::Request> request,
  std::shared_ptr<as2_msgs::srv::ListControlModes::Response> response)
{
  (void)request;
  response->control_modes = this->available_control_modes_;
  response->source = "Platform";
}

void AerialPlatform::alertEventCallback(const as2_msgs::msg::AlertEvent::SharedPtr msg)
{
  alertEvent(*msg.get());
}
}  // namespace as2
