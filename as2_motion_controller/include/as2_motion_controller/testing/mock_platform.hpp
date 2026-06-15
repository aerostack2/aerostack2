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

/*!*******************************************************************************************
 *  @file       mock_platform.hpp
 *  @brief      Reusable fake-platform fixture for end-to-end smoke tests of
 *              `as2_motion_controller` plugins.
 *  @authors    Rafael Perez-Segui
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__TESTING__MOCK_PLATFORM_HPP_
#define AS2_MOTION_CONTROLLER__TESTING__MOCK_PLATFORM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

namespace as2_motion_controller_test
{

/**
 * @brief Fake platform node used by `*_mock.cpp` smoke executables.
 *
 * Serves the two services the controller manager needs at startup
 * (`list_control_modes`, `set_platform_control_mode`) and, after a configurable
 * delay, drives the controller into the requested control mode through the
 * public `controller/set_control_mode` service. Replaces the ~150 lines of
 * boilerplate that used to be duplicated verbatim in every plugin's
 * `<plugin>_mock.cpp`.
 */
class MockPlatform : public as2::Node
{
public:
  /**
   * @brief Configuration of the synthetic `set_control_mode` request fired by the mock.
   */
  struct ControlModeRequest
  {
    uint8_t control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
    uint8_t yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
    uint8_t reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  };

  /**
   * @brief Construct the fake platform node and arm the synthetic mode request.
   *
   * @param available_modes Modes returned by `list_control_modes` (typically a HOVER input + an ACRO output bitmap).
   * @param request Mode requested through `controller/set_control_mode` after `init_delay`.
   * @param init_delay Delay before issuing the request, to give the manager time to advertise its services.
   * @param options Standard NodeOptions (namespace, params, etc.).
   */
  explicit MockPlatform(
    const std::vector<uint8_t> & available_modes,
    const ControlModeRequest & request,
    std::chrono::milliseconds init_delay = std::chrono::seconds(1),
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief One-shot timer callback that fires the synthetic `controller/set_control_mode` request.
   */
  void callSetControlMode();

  /**
   * @brief Service handler for `list_control_modes`.
   *
   * Returns the modes configured at construction time as the platform-supported list.
   *
   * @param request Service request (ignored).
   * @param response Service response filled with the configured modes.
   */
  void handleListControlModes(
    const std::shared_ptr<as2_msgs::srv::ListControlModes::Request> request,
    std::shared_ptr<as2_msgs::srv::ListControlModes::Response> response);

  /**
   * @brief Service handler for `platform/set_platform_control_mode`.
   *
   * Always succeeds; emulates the platform accepting the requested mode.
   *
   * @param request Mode the controller asks the platform to enter.
   * @param response Service response with success set to true.
   */
  void handleSetPlatformControlMode(
    const std::shared_ptr<as2_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<as2_msgs::srv::SetControlMode::Response> response);

  std::vector<uint8_t> available_modes_;
  ControlModeRequest request_;

  std::shared_ptr<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>
  set_control_mode_client_;
  rclcpp::Service<as2_msgs::srv::ListControlModes>::SharedPtr list_control_modes_server_;
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_platform_control_mode_server_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp::CallbackGroup::SharedPtr list_modes_srv_callback_group_;
  rclcpp::CallbackGroup::SharedPtr set_mode_srv_callback_group_;
};

}  // namespace as2_motion_controller_test

#endif  // AS2_MOTION_CONTROLLER__TESTING__MOCK_PLATFORM_HPP_
