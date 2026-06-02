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
 *  @file       controller_handler.cpp
 *  @brief      Controller handler class implementation
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#include "as2_motion_controller/controller_handler.hpp"

#include <as2_core/utils/tf_utils.hpp>

namespace controller_handler
{

static inline bool checkMatchWithMask(
  const uint8_t mode1,
  const uint8_t mode2,
  const uint8_t mask)
{
  return (mode1 & mask) == (mode2 & mask);
}

static uint8_t findBestMatchWithMask(
  const uint8_t mode,
  const std::vector<uint8_t> & mode_list,
  const uint8_t mask)
{
  uint8_t best_match = 0;
  for (const auto & candidate : mode_list) {
    if (checkMatchWithMask(mode, candidate, mask)) {
      best_match = candidate;
      if (candidate == mode) {
        return candidate;
      }
    }
  }
  return best_match;
}

ControllerHandler::ControllerHandler(
  std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller,
  as2::Node * node,
  as2::tf::TfHandler * tf_handler)
: controller_ptr_(controller), node_ptr_(node), tf_handler_(tf_handler)
{
  node_ptr_->get_parameter("use_bypass", use_bypass_);
  node_ptr_->get_parameter("odom_frame_id", enu_frame_id_);
  node_ptr_->get_parameter("base_frame_id", flu_frame_id_);

  // Frame ids
  enu_frame_id_ = as2::tf::generateTfName(node_ptr_, enu_frame_id_);
  flu_frame_id_ = as2::tf::generateTfName(node_ptr_, flu_frame_id_);
  input_pose_frame_id_ = as2::tf::generateTfName(node_ptr_, input_pose_frame_id_);
  input_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, input_twist_frame_id_);
  output_pose_frame_id_ = as2::tf::generateTfName(node_ptr_, output_pose_frame_id_);
  output_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, output_twist_frame_id_);

  // Subscribers
  ref_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::motion_reference::pose, as2_names::topics::motion_reference::qos,
    std::bind(&ControllerHandler::refPoseCallback, this, std::placeholders::_1));
  ref_twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::motion_reference::twist, as2_names::topics::motion_reference::qos,
    std::bind(&ControllerHandler::refTwistCallback, this, std::placeholders::_1));
  ref_traj_sub_ = node_ptr_->create_subscription<as2_msgs::msg::TrajectorySetpoints>(
    as2_names::topics::motion_reference::trajectory, as2_names::topics::motion_reference::qos,
    std::bind(&ControllerHandler::refTrajCallback, this, std::placeholders::_1));
  ref_thrust_sub_ = node_ptr_->create_subscription<as2_msgs::msg::Thrust>(
    as2_names::topics::motion_reference::thrust, as2_names::topics::motion_reference::qos,
    std::bind(&ControllerHandler::refThrustCallback, this, std::placeholders::_1));
  platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&ControllerHandler::platformInfoCallback, this, std::placeholders::_1));
  twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&ControllerHandler::stateCallback, this, std::placeholders::_1));

  // Publishers
  trajectory_pub_ = node_ptr_->create_publisher<as2_msgs::msg::TrajectorySetpoints>(
    as2_names::topics::actuator_command::trajectory, as2_names::topics::actuator_command::qos);
  pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::actuator_command::pose, as2_names::topics::actuator_command::qos);
  twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
  thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
    as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);

  // Services servers
  set_control_mode_srv_ = node_ptr_->create_service<as2_msgs::srv::SetControlMode>(
    as2_names::services::controller::set_control_mode,
    std::bind(
      &ControllerHandler::setControlModeSrvCall, this,
      std::placeholders::_1,            // Corresponds to the 'request'  input
      std::placeholders::_2             // Corresponds to the 'response' input
  ));

  // Services clients
  set_control_mode_client_ =
    std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>(
    as2_names::services::platform::set_platform_control_mode, node_ptr_);
  list_control_modes_client_ =
    std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>>(
    as2_names::services::platform::list_control_modes, node_ptr_);

  // Timers
  double cmd_freq = 0.0;
  node_ptr_->get_parameter("cmd_freq", cmd_freq);
  control_timer_ =
    node_ptr_->create_timer(
    std::chrono::duration<double>(1.0 / cmd_freq),
    std::bind(&ControllerHandler::controlTimerCallback, this));

  parameters_callback_handle_ = node_ptr_->add_on_set_parameters_callback(
    std::bind(&ControllerHandler::parametersCallback, this, std::placeholders::_1));

  control_mode_in_.control_mode = as2_msgs::msg::ControlMode::UNSET;
  control_mode_out_.control_mode = as2_msgs::msg::ControlMode::UNSET;

  initializeDebugPublishers();
}

rcl_interfaces::msg::SetParametersResult ControllerHandler::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Frame parameters are immutable at runtime: changing them mid-flight
  // would invalidate cached references and integrators in the plugin.
  for (const auto & param : parameters) {
    if (param.get_name() == "desired_pose_frame" ||
      param.get_name() == "desired_twist_frame")
    {
      result.successful = false;
      result.reason = "Frame parameters cannot be modified at runtime";
      return result;
    }
  }

  controller_ptr_->dispatchParameters(parameters);
  return result;
}

void ControllerHandler::getMode(
  as2_msgs::msg::ControlMode & mode_in,
  as2_msgs::msg::ControlMode & mode_out)
{
  mode_in = control_mode_in_;
  mode_out = control_mode_out_;
  return;
}

void ControllerHandler::setInputControlModesAvailables(const std::vector<uint8_t> & available_modes)
{
  controller_available_modes_in_ = available_modes;
  // sort modes in ascending order
  std::sort(controller_available_modes_in_.begin(), controller_available_modes_in_.end());
}

void ControllerHandler::setOutputControlModesAvailables(
  const std::vector<uint8_t> & available_modes)
{
  controller_available_modes_out_ = available_modes;
  // sort modes in ascending order
  std::sort(controller_available_modes_out_.begin(), controller_available_modes_out_.end());
}

void ControllerHandler::reset()
{
  controller_ptr_->reset();
  last_time_ = node_ptr_->now();
  state_acquired_ = false;
  ref_pose_acquired_ = false;
  ref_twist_acquired_ = false;
  ref_traj_acquired_ = false;
  ref_thrust_acquired_ = false;
}

void ControllerHandler::stateCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  if (!control_mode_established_ || bypass_controller_) {
    return;
  }

  try {
    // Use the latest cached transform (`tf2::TimePointZero` via timeout==0)
    // Either this change have more error, is more efficient to ensure the
    // controller frequency
    auto [pose_msg, twist_msg] = tf_handler_->getState(
      *_twist_msg, input_twist_frame_id_, input_pose_frame_id_, flu_frame_id_,
      std::chrono::nanoseconds::zero());

    state_acquired_ = true;
    state_pose_ = pose_msg;
    state_twist_ = twist_msg;
    if (!bypass_controller_) {controller_ptr_->updateState(state_pose_, state_twist_);}
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void ControllerHandler::refPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if ((!control_mode_established_ && !bypass_controller_) ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::UNSET)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose_msg = *msg;
  if (!tf_handler_->tryConvert(pose_msg, input_pose_frame_id_)) {
    auto & clk = *node_ptr_->get_clock();
    RCLCPP_ERROR_THROTTLE(
      node_ptr_->get_logger(), clk, 1000,
      "Failed to convert reference pose to input frame, from %s to %s",
      pose_msg.header.frame_id.c_str(), input_pose_frame_id_.c_str());
    return;
  }
  ref_pose_ = pose_msg;
  ref_pose_acquired_ = true;

  if (!bypass_controller_) {controller_ptr_->updateReference(ref_pose_);}
}

void ControllerHandler::refTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if ((!control_mode_established_ && !bypass_controller_) ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::UNSET)
  {
    return;
  }

  geometry_msgs::msg::TwistStamped twist_msg = *msg;
  if (!tf_handler_->tryConvert(twist_msg, input_twist_frame_id_)) {
    auto & clk = *node_ptr_->get_clock();
    RCLCPP_ERROR_THROTTLE(
      node_ptr_->get_logger(), clk, 1000,
      "Failed to convert reference twist to input frame, from %s to %s",
      twist_msg.header.frame_id.c_str(), input_twist_frame_id_.c_str());
    return;
  }
  ref_twist_ = twist_msg;
  ref_twist_acquired_ = true;

  if (!bypass_controller_) {controller_ptr_->updateReference(ref_twist_);}
}

void ControllerHandler::refTrajCallback(const as2_msgs::msg::TrajectorySetpoints::SharedPtr msg)
{
  if ((!control_mode_established_ && !bypass_controller_) ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::UNSET)
  {
    return;
  }

  auto & clk = *node_ptr_->get_clock();

  if (msg->setpoints.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), clk, 1000,
      "Received TrajectorySetpoints with empty setpoints array. Dropping.");
    return;
  }

  // TrajectorySetpoints encodes pose and twist in the same frame: if the
  // active mode in the plugin reports different frames for pose and twist,
  // the plugin does not support trajectory references in this mode.
  if (input_pose_frame_id_ != input_twist_frame_id_) {
    RCLCPP_ERROR_THROTTLE(
      node_ptr_->get_logger(), clk, 1000,
      "Plugin expects different frames for pose ('%s') and twist ('%s') in "
      "the active mode; trajectory references require both to be equal. "
      "Dropping.",
      input_pose_frame_id_.c_str(), input_twist_frame_id_.c_str());
    return;
  }

  as2_msgs::msg::TrajectorySetpoints traj = *msg;
  if (traj.header.frame_id != input_pose_frame_id_) {
    if (!tf_handler_->tryConvert(traj, input_pose_frame_id_)) {
      RCLCPP_ERROR_THROTTLE(
        node_ptr_->get_logger(), clk, 1000,
        "Cannot transform trajectory from '%s' to '%s' at time %f. Dropping.",
        msg->header.frame_id.c_str(), input_pose_frame_id_.c_str(), rclcpp::Time(
          traj.header.stamp).seconds());
      return;
    }
  }

  ref_traj_ = traj;
  ref_traj_acquired_ = true;
  if (!bypass_controller_) {controller_ptr_->updateReference(ref_traj_);}
}

void ControllerHandler::refThrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg)
{
  if ((!control_mode_established_ && !bypass_controller_) ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::UNSET)
  {
    return;
  }

  ref_thrust_ = *msg;
  ref_thrust_acquired_ = true;
  if (!bypass_controller_) {controller_ptr_->updateReference(ref_thrust_);}
}

void ControllerHandler::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  platform_info_ = *msg;
}

void ControllerHandler::setControlModeSrvCall(
  const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
  as2_msgs::srv::SetControlMode::Response::SharedPtr response)
{
  uint8_t _control_mode_plugin_in = 0;
  uint8_t _control_mode_plugin_out = 0;

  as2_msgs::msg::ControlMode _control_mode_msg_plugin_in;
  as2_msgs::msg::ControlMode _control_mode_msg_plugin_out;

  control_mode_established_ = false;

  // check if platform_available_modes is set
  if (!listPlatformAvailableControlModes()) {
    response->success = false;
    return;
  }

  // If the input mode is Hover, set desired control mode in to Hover,
  // else, set desired control mode in to the request one
  if (request->control_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    _control_mode_plugin_in = HOVER_MODE_MASK;
  } else {
    _control_mode_plugin_in =
      as2::control_mode::convertAS2ControlModeToUint8t(request->control_mode);
  }

  // Check if a bypass is possible for the input_control_mode_desired ( DISCARDING REFERENCE
  // COMPONENT)
  bypass_controller_ = false;
  if (use_bypass_) {
    if (_control_mode_plugin_in == HOVER_MODE_MASK) {
      if (trySetPlatformHover()) {
        bypass_controller_ = true;
        _control_mode_plugin_out = HOVER_MODE_MASK;
      }
    } else {
      bypass_controller_ = tryToBypassController(_control_mode_plugin_in, _control_mode_plugin_out);
    }
  }

  if (bypass_controller_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Bypassing controller");
    _control_mode_plugin_in = UNSET_MODE_MASK;
  } else {
    bool success = findSuitableControlModes(_control_mode_plugin_in, _control_mode_plugin_out);

    if (!success) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "No suitable control mode found");
      response->success = false;
      return;
    }
  }

  // request the out mode to the platform
  _control_mode_msg_plugin_out =
    as2::control_mode::convertUint8tToAS2ControlMode(_control_mode_plugin_out);
  if (!setPlatformControlMode(_control_mode_msg_plugin_out)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to set platform control mode");
    response->success = false;
    return;
  }

  // request the input and output modes to the platform
  _control_mode_msg_plugin_in =
    as2::control_mode::convertUint8tToAS2ControlMode(_control_mode_plugin_in);
  if (!controller_ptr_->setMode(_control_mode_msg_plugin_in, _control_mode_msg_plugin_out)) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Failed to set plugin control mode to [%s]",
      as2::control_mode::controlModeToString(_control_mode_msg_plugin_in).c_str());

    as2_msgs::msg::ControlMode hover_mode =
      as2::control_mode::convertUint8tToAS2ControlMode(HOVER_MODE_MASK);

    if (_control_mode_msg_plugin_in.control_mode != hover_mode.control_mode) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Try to set hover mode instead");

      auto request_hover = std::make_shared<as2_msgs::srv::SetControlMode::Request>();
      request_hover->control_mode = hover_mode;

      setControlModeSrvCall(request_hover, response);
      if (response->success) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Hover mode set successfully");
      }
    }
    response->success = false;
    return;
  }
  control_mode_established_ = true;

  control_mode_out_ = _control_mode_msg_plugin_out;
  if (bypass_controller_) {
    control_mode_in_ = _control_mode_msg_plugin_out;
  } else {
    control_mode_in_ = _control_mode_msg_plugin_in;
  }

  // set frames id
  output_pose_frame_id_ = getFrameIdByReferenceFrame(control_mode_out_.reference_frame);
  output_twist_frame_id_ = getFrameIdByReferenceFrame(control_mode_out_.reference_frame);
  if (bypass_controller_) {
    input_pose_frame_id_ = output_pose_frame_id_;
    input_twist_frame_id_ = output_twist_frame_id_;
  } else {
    // The plugin returns frames already namespaced frame ids
    input_pose_frame_id_ = controller_ptr_->getDesiredPoseFrameId();
    input_twist_frame_id_ = controller_ptr_->getDesiredTwistFrameId();
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "input_mode:[%s]",
    as2::control_mode::controlModeToString(control_mode_in_).c_str());
  RCLCPP_INFO(
    node_ptr_->get_logger(), "output_mode:[%s]",
    as2::control_mode::controlModeToString(control_mode_out_).c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "input_pose_frame_id:[%s]", input_pose_frame_id_.c_str());
  RCLCPP_INFO(node_ptr_->get_logger(), "input_twist_frame_id:[%s]", input_twist_frame_id_.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "output_pose_frame_id:[%s]", output_pose_frame_id_.c_str());
  RCLCPP_INFO(
    node_ptr_->get_logger(), "output_twist_frame_id:[%s]",
    output_twist_frame_id_.c_str());

  reset();

  // After a successful HOVER setup the plugin must produce a hover reference
  if (!bypass_controller_ &&
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER)
  {
    controller_ptr_->requestHoverLatch();
  }

  response->success = true;
  return;
}


bool ControllerHandler::listPlatformAvailableControlModes()
{
  if (platform_available_modes_in_.empty()) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "LISTING AVAILABLE MODES");
    // if the list is empty, send a request to the platform to get the list of
    // available modes
    as2_msgs::srv::ListControlModes::Request list_control_modes_req;
    as2_msgs::srv::ListControlModes::Response list_control_modes_resp;

    bool out =
      list_control_modes_client_->sendRequest(list_control_modes_req, list_control_modes_resp);
    if (!out) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Error listing control_modes");
      return false;
    }
    if (list_control_modes_resp.control_modes.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "No available control modes");
      return false;
    }

    // log the available modes
    for (auto & mode : list_control_modes_resp.control_modes) {
      RCLCPP_DEBUG(
        node_ptr_->get_logger(), "Available mode: %s",
        as2::control_mode::controlModeToString(mode).c_str());
    }

    platform_available_modes_in_ = list_control_modes_resp.control_modes;
  }
  return true;
}

void ControllerHandler::controlTimerCallback()
{
  if (!platform_info_.offboard || !platform_info_.armed ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::HOVER)
  {
    return;
  }

  if (!control_mode_established_) {
    auto & clock = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock, 10000, "Control mode not established");
    return;
  }

  if (!state_acquired_ && !bypass_controller_) {
    auto & clock = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock, 1000, "Waiting for odometry ");
    return;
  }

  if (!controller_ptr_->isReferenceReceived() && !bypass_controller_) {
    auto & clock = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(
      node_ptr_->get_logger(), clock, 1000, "Waiting for motion reference");
    return;
  }

  sendCommand();
  // Publish debug snapshot using a single tick stamp so state, reference and
  // output stay aligned in time across topics.
  publishDebug(node_ptr_->now());
}

std::string ControllerHandler::getFrameIdByReferenceFrame(uint8_t reference_frame)
{
  switch (reference_frame) {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      return enu_frame_id_;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      return flu_frame_id_;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      return "not_implemented";
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
    default:
      return "undefined";
  }
}

bool ControllerHandler::setPlatformControlMode(const as2_msgs::msg::ControlMode & mode)
{
  as2_msgs::srv::SetControlMode::Request set_control_mode_req;
  as2_msgs::srv::SetControlMode::Response set_control_mode_resp;
  set_control_mode_req.control_mode = mode;
  auto out = set_control_mode_client_->sendRequest(set_control_mode_req, set_control_mode_resp);
  if (out && set_control_mode_resp.success) {return true;}
  return false;
}

bool ControllerHandler::findSuitableOutputControlModeForPlatformInputMode(
  uint8_t & output_mode,
  const uint8_t input_mode)
{
  //  check if the preferred mode is available
  if (preferred_output_mode_) {
    auto match = findBestMatchWithMask(
      preferred_output_mode_, platform_available_modes_in_,
      MATCH_MODE_AND_YAW);
    if (match) {
      output_mode = match;
      return true;
    }
  }

  // if the preferred mode is not available, search for the first common mode

  uint8_t common_mode = 0;
  bool same_yaw = false;

  for (auto & mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_MODE_AND_YAW);
    if (common_mode) {
      break;
    }
  }

  // check if the common mode exist
  if (common_mode == 0) {
    return false;
  }
  output_mode = common_mode;
  return true;
}

bool ControllerHandler::checkSuitabilityInputMode(uint8_t & input_mode, const uint8_t output_mode)
{
  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  for (auto & mode : controller_available_modes_in_) {
    if ((input_mode & MATCH_MODE) == HOVER_MODE_MASK && (input_mode & MATCH_MODE) == mode) {
      mode_found = true;
      return true;
    } else if (mode == input_mode) {
      input_mode = mode;
      mode_found = true;
      break;
    }
  }

  // if not match, try to match only control mode and yaw mode
  if (!mode_found) {
    for (auto & mode : controller_available_modes_in_) {
      if (checkMatchWithMask(mode, input_mode, MATCH_MODE_AND_YAW)) {
        input_mode = mode;
        mode_found = true;
        break;
      }
    }
  }

  // check if the input mode is compatible with the output mode
  if ((input_mode & MATCH_MODE) < (output_mode & 0b1111000)) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Input control mode has lower level than output control mode");
    return false;
  }

  return mode_found;
}

bool ControllerHandler::findSuitableControlModes(uint8_t & input_mode, uint8_t & output_mode)
{
  // check if the input mode is available. Get the best output mode
  bool success = findSuitableOutputControlModeForPlatformInputMode(output_mode, input_mode);
  if (!success) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No suitable output control mode found");
    return false;
  }

  // Get the best input mode for the output mode
  success = checkSuitabilityInputMode(input_mode, output_mode);
  if (!success) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Input control mode is not suitable for this controller");
    return false;
  }
  return success;
}

bool ControllerHandler::trySetPlatformHover()
{
  // Check if HOVER_MODE_MASK is available in platform_available_modes_in_
  for (auto & mode : platform_available_modes_in_) {
    if ((mode & MATCH_MODE) == HOVER_MODE_MASK) {
      as2_msgs::msg::ControlMode _control_mode_msg_plugin_out =
        as2::control_mode::convertUint8tToAS2ControlMode(HOVER_MODE_MASK);

      // set the platform in hover mode
      if (setPlatformControlMode(_control_mode_msg_plugin_out)) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Platform set in HOVER mode");
        return true;
      } else {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to set platform control mode to HOVER");
        return false;
      }
    }
  }
  return false;
}

bool ControllerHandler::tryToBypassController(const uint8_t input_mode, uint8_t & output_mode)
{
  // check if platform available modes are set
  if ((input_mode & MATCH_MODE) == UNSET_MODE_MASK ||
    (input_mode & MATCH_MODE) == HOVER_MODE_MASK)
  {
    return false;
  }

  uint8_t candidate_mode =
    findBestMatchWithMask(input_mode, platform_available_modes_in_, MATCH_MODE_AND_YAW);
  if (candidate_mode) {
    output_mode = candidate_mode;
    return true;
  }
  return false;
}

void ControllerHandler::sendCommand()
{
  if (bypass_controller_) {
    bool motion_reference_acquired = ref_pose_acquired_ || ref_twist_acquired_ ||
      ref_traj_acquired_ || ref_thrust_acquired_;
    if (!motion_reference_acquired) {
      auto & clock = *node_ptr_->get_clock();
      RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clock, 2000, "Waiting for motion reference");
      return;
    }
    command_pose_ = ref_pose_;
    command_twist_ = ref_twist_;
  } else {
    rclcpp::Time current_time = node_ptr_->now();
    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    if (dt <= 0) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Loop delta time is zero or below. Check your clock");
      return;
    }

    last_time_ = current_time;
    const rclcpp::Time t0 = node_ptr_->now();
    const bool ok =
      controller_ptr_->computeOutput(dt, command_pose_, command_twist_, command_thrust_);
    const rclcpp::Time t1 = node_ptr_->now();
    if (debug_compute_output_time_pub_) {
      std_msgs::msg::Float64 msg;
      msg.data = (t1 - t0).seconds();
      debug_compute_output_time_pub_->publish(msg);
    }
    if (!ok) {return;}
  }
  publishCommand();
  return;
}

void ControllerHandler::publishCommand()
{
  command_pose_.header.stamp = node_ptr_->now();
  command_twist_.header.stamp = command_pose_.header.stamp;

  if (control_mode_out_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::ATTITUDE)
  {
    if (command_pose_.header.frame_id != output_pose_frame_id_ &&
      !tf_handler_->tryConvert(
        command_pose_, output_pose_frame_id_,
        std::chrono::nanoseconds::zero()))
    {
      auto & clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(
        node_ptr_->get_logger(), clk, 1000,
        "Failed to convert command pose to output frame, from %s to %s",
        command_pose_.header.frame_id.c_str(), output_pose_frame_id_.c_str());
      return;
    }
  }

  if (control_mode_out_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::ACRO)
  {
    if (command_twist_.header.frame_id != output_twist_frame_id_ &&
      !tf_handler_->tryConvert(
        command_twist_, output_twist_frame_id_,
        std::chrono::nanoseconds::zero()))
    {
      auto & clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(
        node_ptr_->get_logger(), clk, 1000,
        "Failed to convert command twist to output frame, from %s to %s",
        command_twist_.header.frame_id.c_str(), output_twist_frame_id_.c_str());
      return;
    }
  }

  switch (control_mode_out_.control_mode) {
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      trajectory_pub_->publish(ref_traj_);
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      pose_pub_->publish(command_pose_);
      twist_pub_->publish(command_twist_);  // For twist limits
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      twist_pub_->publish(command_twist_);
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      pose_pub_->publish(command_pose_);
      twist_pub_->publish(command_twist_);
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      command_thrust_.header = command_pose_.header;
      pose_pub_->publish(command_pose_);
      thrust_pub_->publish(command_thrust_);
      break;
    case as2_msgs::msg::ControlMode::ACRO:
      command_thrust_.header = command_pose_.header;
      twist_pub_->publish(command_twist_);
      thrust_pub_->publish(command_thrust_);
      break;
  }
}

void ControllerHandler::initializeDebugPublishers()
{
  // Helper that declares an optional string parameter holding a topic name and
  // returns it. An empty value (default) keeps the publisher disabled.
  auto declare_topic = [this](const std::string & name) -> std::string {
      if (!node_ptr_->has_parameter(name)) {
        node_ptr_->declare_parameter<std::string>(name, "");
      }
      return node_ptr_->get_parameter(name).as_string();
    };

  const std::string state_pose_topic = declare_topic("debug.state_pose_topic");
  const std::string state_twist_topic = declare_topic("debug.state_twist_topic");
  const std::string ref_pose_topic = declare_topic("debug.reference_pose_topic");
  const std::string ref_twist_topic = declare_topic("debug.reference_twist_topic");
  const std::string ref_traj_topic = declare_topic("debug.reference_trajectory_topic");
  const std::string ref_thrust_topic = declare_topic("debug.reference_thrust_topic");
  const std::string compute_output_time_topic = declare_topic("debug.compute_output_time_topic");

  const auto qos = rclcpp::SensorDataQoS();
  if (!state_pose_topic.empty()) {
    debug_state_pose_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(state_pose_topic, qos);
  }
  if (!state_twist_topic.empty()) {
    debug_state_twist_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(state_twist_topic, qos);
  }
  if (!ref_pose_topic.empty()) {
    debug_reference_pose_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(ref_pose_topic, qos);
  }
  if (!ref_twist_topic.empty()) {
    debug_reference_twist_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(ref_twist_topic, qos);
  }
  if (!ref_traj_topic.empty()) {
    debug_reference_trajectory_pub_ =
      node_ptr_->create_publisher<as2_msgs::msg::TrajectorySetpoints>(ref_traj_topic, qos);
  }
  if (!ref_thrust_topic.empty()) {
    debug_reference_thrust_pub_ =
      node_ptr_->create_publisher<as2_msgs::msg::Thrust>(ref_thrust_topic, qos);
  }
  if (!compute_output_time_topic.empty()) {
    debug_compute_output_time_pub_ =
      node_ptr_->create_publisher<std_msgs::msg::Float64>(compute_output_time_topic, qos);
  }
}

void ControllerHandler::publishDebug(const rclcpp::Time & tick)
{
  // State and reference are published in the frame the plugin requested
  // (input_*_frame_id_, which is the plugin's getDesiredXFrameId() outside
  // bypass mode). The plugin output is exposed via `actuator_command/*`,
  // which carries the same tick stamp.
  if (debug_state_pose_pub_ && state_acquired_) {
    auto msg = state_pose_;
    msg.header.stamp = tick;
    debug_state_pose_pub_->publish(msg);
  }
  if (debug_state_twist_pub_ && state_acquired_) {
    auto msg = state_twist_;
    msg.header.stamp = tick;
    debug_state_twist_pub_->publish(msg);
  }
  if (debug_reference_pose_pub_ && ref_pose_acquired_) {
    auto msg = ref_pose_;
    msg.header.stamp = tick;
    debug_reference_pose_pub_->publish(msg);
  }
  if (debug_reference_twist_pub_ && ref_twist_acquired_) {
    auto msg = ref_twist_;
    msg.header.stamp = tick;
    debug_reference_twist_pub_->publish(msg);
  }
  if (debug_reference_trajectory_pub_ && ref_traj_acquired_) {
    auto msg = ref_traj_;
    msg.header.stamp = tick;
    debug_reference_trajectory_pub_->publish(msg);
  }
  if (debug_reference_thrust_pub_ && ref_thrust_acquired_) {
    auto msg = ref_thrust_;
    msg.header.stamp = tick;
    debug_reference_thrust_pub_->publish(msg);
  }
}

}  // namespace controller_handler
