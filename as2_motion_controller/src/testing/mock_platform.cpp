// Copyright 2026 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

#include "as2_motion_controller/testing/mock_platform.hpp"

#include "as2_core/names/services.hpp"

namespace as2_motion_controller_test
{

MockPlatform::MockPlatform(
  const std::vector<uint8_t> & available_modes,
  const ControlModeRequest & request,
  std::chrono::milliseconds init_delay,
  const rclcpp::NodeOptions & options)
: as2::Node("mock_platform", options),
  available_modes_(available_modes),
  request_(request)
{
  list_modes_srv_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  set_mode_srv_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  set_control_mode_client_ =
    std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>(
    as2_names::services::controller::set_control_mode, this);

  list_control_modes_server_ = this->create_service<as2_msgs::srv::ListControlModes>(
    as2_names::services::platform::list_control_modes,
    std::bind(
      &MockPlatform::handleListControlModes, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, list_modes_srv_callback_group_);

  set_platform_control_mode_server_ = this->create_service<as2_msgs::srv::SetControlMode>(
    as2_names::services::platform::set_platform_control_mode,
    std::bind(
      &MockPlatform::handleSetPlatformControlMode, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, set_mode_srv_callback_group_);

  init_timer_ = this->create_wall_timer(
    init_delay,
    std::bind(&MockPlatform::callSetControlMode, this));

  RCLCPP_INFO(this->get_logger(), "Mock platform ready");
}

void MockPlatform::callSetControlMode()
{
  init_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Sending set_control_mode request to controller...");

  auto request = std::make_shared<as2_msgs::srv::SetControlMode::Request>();
  request->control_mode.control_mode = request_.control_mode;
  request->control_mode.yaw_mode = request_.yaw_mode;
  request->control_mode.reference_frame = request_.reference_frame;
  auto response = std::make_shared<as2_msgs::srv::SetControlMode::Response>();

  if (set_control_mode_client_->sendRequest(request, response)) {
    RCLCPP_INFO(this->get_logger(), "set_control_mode succeeded");
  } else {
    RCLCPP_ERROR(this->get_logger(), "set_control_mode failed");
  }
}

void MockPlatform::handleListControlModes(
  const std::shared_ptr<as2_msgs::srv::ListControlModes::Request>/*request*/,
  std::shared_ptr<as2_msgs::srv::ListControlModes::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "list_control_modes service called");
  response->source = "Platform";
  response->control_modes = available_modes_;
}

void MockPlatform::handleSetPlatformControlMode(
  const std::shared_ptr<as2_msgs::srv::SetControlMode::Request> request,
  std::shared_ptr<as2_msgs::srv::SetControlMode::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "set_platform_control_mode called: control_mode=%u yaw_mode=%u frame=%u",
    request->control_mode.control_mode,
    request->control_mode.yaw_mode,
    request->control_mode.reference_frame);
  response->success = true;
}

}  // namespace as2_motion_controller_test
