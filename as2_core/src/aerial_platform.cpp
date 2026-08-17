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
    cmd_freq_ = this->getParameter<float>("cmd_freq", 100.0);
    info_freq_ = this->getParameter<float>("info_freq", 10.0);

    // Created up front, so that its buffer is filled before the first conversion,
    // and so that platform implementations can use it for their own transforms
    tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

    this->loadControlModes(this->getParameter<std::string>("control_modes_file"));

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
      [this](const as2_msgs::msg::AlertEvent::ConstSharedPtr msg) {
        this->alertEventCallback(msg);
      });

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

  resetActuatorCommandMsgs();
}

void AerialPlatform::resetActuatorCommandMsgs()
{
  command_trajectory_msg_ = as2_msgs::msg::TrajectorySetpoints();
  command_pose_msg_ = geometry_msgs::msg::PoseStamped();
  command_twist_msg_ = geometry_msgs::msg::TwistStamped();
  command_thrust_msg_ = as2_msgs::msg::Thrust();

  command_trajectory_msg_.header.frame_id = command_pose_frame_id_;
  command_pose_msg_.header.frame_id = command_pose_frame_id_;
  command_twist_msg_.header.frame_id = command_twist_frame_id_;

  has_new_references_ = true;
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
  as2_msgs::msg::ControlMode resolved_mode;
  if (!as2::control_mode::resolveControlMode(msg, available_control_modes_, resolved_mode)) {
    RCLCPP_ERROR(
      this->get_logger(), "Control mode [%s] is not available in this platform",
      as2::control_mode::controlModeToString(msg).c_str());
    return false;
  }

  // The mode that is active keeps its frames if this change does not go through
  const std::string previous_pose_frame_id = command_pose_frame_id_;
  const std::string previous_twist_frame_id = command_twist_frame_id_;

  // Cleared so that ownSetPlatformControlMode() is the only place that declares
  // the frames of the mode it is accepting
  command_pose_frame_id_.clear();
  command_twist_frame_id_.clear();

  const as2::control_mode::CommandFrameUsage usage =
    as2::control_mode::getCommandFrameUsage(resolved_mode);
  bool accepted = ownSetPlatformControlMode(resolved_mode);

  // Read after the call, since that is where the platform overrides the frames
  const bool frames_missing = (usage.pose && command_pose_frame_id_.empty()) ||
    (usage.twist && command_twist_frame_id_.empty());
  // TrajectorySetpoints carries a single header, so it cannot hold two frames
  const bool trajectory_frames_differ =
    resolved_mode.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY &&
    command_pose_frame_id_ != command_twist_frame_id_;

  if (!accepted) {
    RCLCPP_ERROR(this->get_logger(), "Unable to set control mode %d", resolved_mode.control_mode);
  } else if (frames_missing) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Control mode [%s] does not declare its command frames: call "
      "setCommandPoseFrameId() and setCommandTwistFrameId() from ownSetPlatformControlMode()",
      as2::control_mode::controlModeToString(resolved_mode).c_str());
    accepted = false;
  } else if (trajectory_frames_differ) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Control mode [%s] declares different pose ('%s') and twist ('%s') command frames. "
      "TrajectorySetpoints carries a single header, so both must be the same frame",
      as2::control_mode::controlModeToString(resolved_mode).c_str(),
      command_pose_frame_id_.c_str(), command_twist_frame_id_.c_str());
    accepted = false;
  }

  if (!accepted) {
    command_pose_frame_id_ = previous_pose_frame_id;
    command_twist_frame_id_ = previous_twist_frame_id;
    return false;
  }

  has_new_references_ = resolved_mode.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    resolved_mode.control_mode == as2_msgs::msg::ControlMode::UNSET;
  platform_info_msg_.current_control_mode = resolved_mode;
  return true;
}

void AerialPlatform::setCommandPoseFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Pose command frame id cannot be empty");
    return;
  }
  if (frame_id == command_pose_frame_id_) {
    return;
  }
  command_pose_frame_id_ = frame_id;
  RCLCPP_INFO(this->get_logger(), "Pose command frame set to '%s'", frame_id.c_str());
}

void AerialPlatform::setCommandTwistFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Twist command frame id cannot be empty");
    return;
  }
  if (frame_id == command_twist_frame_id_) {
    return;
  }
  command_twist_frame_id_ = frame_id;
  RCLCPP_INFO(this->get_logger(), "Twist command frame set to '%s'", frame_id.c_str());
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
    // Zero timeout: use the latest cached transform, never block the command timer
    if (!as2::control_mode::convertCommandsToFrame(
        *tf_handler_, command_pose_frame_id_, command_twist_frame_id_,
        platform_info_msg_.current_control_mode,
        command_pose_msg_, command_twist_msg_, command_trajectory_msg_))
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), clk, 1000,
        "Unable to express the commands in the frames of the control mode: pose from '%s' to "
        "'%s', twist from '%s' to '%s'",
        command_pose_msg_.header.frame_id.c_str(), command_pose_frame_id_.c_str(),
        command_twist_msg_.header.frame_id.c_str(), command_twist_frame_id_.c_str());
      return;
    }
    if (!ownSendCommand()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 5000, "Platform command failed");
    }
  }
}  // namespace as2

void AerialPlatform::loadControlModes(const std::string & filename)
{
  std::vector<std::string> modes = as2::yaml::find_tag_in_yaml_file(filename, "available_modes");

  for (std::vector<std::string>::iterator it = modes.begin(); it != modes.end(); ++it) {
    const uint8_t raw = as2::yaml::parse_uint_from_string(it->c_str());

    // Bits [1:0] used to encode the reference frame of the mode, which no longer
    // exists, so they are dropped and the file is asked to drop them too
    const uint8_t m = raw & ~MATCH_RESERVED_BITS;
    if (raw != m) {
      RCLCPP_WARN(
        this->get_logger(),
        "Control mode 0x%02X of %s sets the reserved bits [1:0], which are ignored. "
        "The reference frame is no longer part of the control mode, set them to zero",
        raw, filename.c_str());
    }
    as2::control_mode::printControlMode(m);

    // Modes are matched by control mode and yaw mode alone, so an entry equal to
    // an earlier one in both can never be resolved
    uint8_t duplicate = UNSET_MODE_MASK;
    if (as2::control_mode::findBestMatchWithMask(
        m, available_control_modes_, MATCH_CONTROL_MODE | MATCH_YAW_MODE, duplicate))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Control mode [%s] is already available in this platform. This entry is "
        "unreachable, remove it from %s",
        as2::control_mode::controlModeToString(m).c_str(), filename.c_str());
      continue;
    }

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

void AerialPlatform::alertEventCallback(const as2_msgs::msg::AlertEvent::ConstSharedPtr msg)
{
  alertEvent(*msg.get());
}
}  // namespace as2
