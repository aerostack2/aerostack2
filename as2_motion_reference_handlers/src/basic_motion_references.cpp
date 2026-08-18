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
 *  \file       basic_motion_references.cpp
 *  \brief      Virtual class for basic motion references implementations
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_motion_reference_handlers/basic_motion_references.hpp"
#include "motion_reference_bus.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

namespace as2
{
namespace motionReferenceHandlers
{

BasicMotionReferenceHandler::BasicMotionReferenceHandler(
  as2::Node * as2_ptr,
  const std::string & ns)
: node_ptr_(as2_ptr), namespace_(ns)
{
  namespace_ = ns == "" ? ns : "/" + ns + "/";
  shared_ = MotionReferenceBus::get(as2_ptr, namespace_);

  // Set initial control mode
  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::NONE;
  desired_control_mode_.control_mode = as2_msgs::msg::ControlMode::UNSET;
}

BasicMotionReferenceHandler::~BasicMotionReferenceHandler() {}

bool BasicMotionReferenceHandler::checkMode()
{
  // Hovering does not need to be settled again, whatever its yaw mode is
  if ((shared_->current_mode_.control_mode == desired_control_mode_.control_mode) &&
    (desired_control_mode_.control_mode == as2_msgs::msg::ControlMode::HOVER))
  {
    return true;
  }

  if (shared_->current_mode_.yaw_mode != desired_control_mode_.yaw_mode ||
    shared_->current_mode_.control_mode != desired_control_mode_.control_mode)
  {
    if (!setMode(desired_control_mode_)) {
      return false;
    }
  }
  return true;
}

bool BasicMotionReferenceHandler::checkFrameId(
  const std::string & frame_id, const std::string & reference)
{
  if (!frame_id.empty()) {
    return true;
  }
  // Without a frame id the reference cannot be converted, and whoever receives
  // it would act on it as if it were already in its own frame
  RCLCPP_ERROR(
    node_ptr_->get_logger(), "Not sending %s reference without frame_id", reference.c_str());
  return false;
}

bool BasicMotionReferenceHandler::sendPoseCommand()
{
  if (!checkFrameId(command_pose_msg_.header.frame_id, "pose") || !checkMode()) {
    return false;
  }
  shared_->command_pose_pub_->publish(command_pose_msg_);
  return true;
}

bool BasicMotionReferenceHandler::sendTwistCommand()
{
  if (!checkFrameId(command_twist_msg_.header.frame_id, "twist") || !checkMode()) {
    return false;
  }
  shared_->command_twist_pub_->publish(command_twist_msg_);
  return true;
}

bool BasicMotionReferenceHandler::sendTrajectoryCommand()
{
  if (!checkFrameId(command_trajectory_msg_.header.frame_id, "trajectory") || !checkMode()) {
    return false;
  }
  shared_->command_traj_pub_->publish(command_trajectory_msg_);
  return true;
}

bool BasicMotionReferenceHandler::sendThrustCommand()
{
  if (!checkMode()) {
    return false;
  }
  shared_->command_thrust_pub_->publish(command_thrust_msg_);
  return true;
}

bool BasicMotionReferenceHandler::setMode(const as2_msgs::msg::ControlMode & mode)
{
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Setting control mode to [%s]",
    as2::control_mode::controlModeToString(mode).c_str());

  // Set request
  auto request = as2_msgs::srv::SetControlMode::Request();
  auto response = as2_msgs::srv::SetControlMode::Response();
  request.control_mode = mode;

  const std::string set_mode_service = shared_->use_actuator_commands_ ?
    as2_names::services::platform::set_platform_control_mode :
    as2_names::services::controller::set_control_mode;

  auto set_mode_cli = as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>(
    namespace_ + set_mode_service, node_ptr_);

  bool out = set_mode_cli.sendRequest(request, response);

  if (out && response.success) {
    shared_->current_mode_ = mode;
    // Sleep for the control mode callback to update
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
  }
  RCLCPP_ERROR(
    node_ptr_->get_logger(),
    "Control Mode was not able to be settled successfully");
  return false;
}

}    // namespace motionReferenceHandlers
}  // namespace as2
