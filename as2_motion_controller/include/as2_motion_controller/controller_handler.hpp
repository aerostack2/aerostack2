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
 *  @file       controller_handler.hpp
 *  @brief      Controller handler class definition
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_
#define AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_

#include <memory>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>
#include <string>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

#include "controller_base.hpp"

namespace controller_handler
{

#define MATCH_ALL 0b11111111
#define MATCH_MODE_AND_FRAME 0b11110011
#define MATCH_MODE 0b11110000
#define MATCH_MODE_AND_YAW 0b11111100

#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

using namespace std::chrono_literals; // NOLINT

/**
 * @brief Orchestrates the controller plugin life cycle inside the ControllerManager.
 *
 * Owns the subscriptions for state and motion references, the publishers for
 * actuator commands and debug topics, the `controller/set_control_mode`
 * service server and the periodic control timer. The ControllerHandler does
 * not own the plugin nor the TfHandler: both are injected by the
 * ControllerManager.
 */
class ControllerHandler
{
public:
  /**
   * @brief Construct a handler bound to a controller plugin and a node.
   *
   * @param controller Loaded controller plugin shared with ControllerManager.
   * @param node Controller node providing logger, parameters and IO.
   * @param tf_handler Non-owning pointer to the TfHandler owned by ControllerManager.
   */
  ControllerHandler(
    std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller,
    as2::Node * node,
    as2::tf::TfHandler * tf_handler);

  virtual ~ControllerHandler() {}

  /**
   * @brief rclcpp on_set_parameters callback applied to all parameter changes.
   *
   * Forwards plugin-namespaced entries to ControllerBase::dispatchParameters
   * and updates handler-owned state (frame ids, debug publishers).
   *
   * @param parameters Batch of parameters being set.
   * @return SetParametersResult with the merged success flag.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Read the currently negotiated input/output control modes.
   *
   * @param mode_in Output: latest negotiated input control mode.
   * @param mode_out Output: latest negotiated output control mode.
   */
  void getMode(as2_msgs::msg::ControlMode & mode_in, as2_msgs::msg::ControlMode & mode_out);

  /**
   * @brief Declare the input control modes the plugin can accept.
   *
   * @param available_modes Bitmasks of supported input modes.
   */
  void setInputControlModesAvailables(const std::vector<uint8_t> & available_modes);

  /**
   * @brief Declare the output control modes the plugin can produce.
   *
   * @param available_modes Bitmasks of supported output modes.
   */
  void setOutputControlModesAvailables(const std::vector<uint8_t> & available_modes);

  /**
   * @brief Clear the handler-side mode and reference state.
   */
  void reset();

private:
  as2::Node * node_ptr_;

  // Control modes availables
  std::vector<uint8_t> controller_available_modes_in_;
  std::vector<uint8_t> controller_available_modes_out_;
  std::vector<uint8_t> platform_available_modes_in_;

  // Frame ids
  std::string enu_frame_id_ = "odom";
  std::string flu_frame_id_ = "base_link";
  std::string input_pose_frame_id_ = "odom";
  std::string input_twist_frame_id_ = "odom";
  std::string output_pose_frame_id_ = "odom";
  std::string output_twist_frame_id_ = "odom";

  // TF handler. Owned by ControllerManager and shared with the plugin.
  as2::tf::TfHandler * tf_handler_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectorySetpoints>::SharedPtr ref_traj_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr ref_thrust_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;

  // Publishers
  rclcpp::Publisher<as2_msgs::msg::TrajectorySetpoints>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  // Debug publishers. Created from `debug.*` parameters on construction
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_state_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_state_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_reference_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_reference_twist_pub_;
  rclcpp::Publisher<as2_msgs::msg::TrajectorySetpoints>::SharedPtr debug_reference_trajectory_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr debug_reference_thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_compute_output_time_pub_;

  // Services servers
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;

  // Services clients
  as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;
  as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>::SharedPtr
    list_control_modes_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Internal variables
  bool control_mode_established_ = false;
  // Aggregated reference gate for control flow (set whenever any of the four
  // specific reference types arrives). Per-type flags below drive debug
  // publishing so we don't emit default-constructed messages on topics for
  // reference types the active mode never produced.
  bool ref_pose_acquired_ = false;
  bool ref_twist_acquired_ = false;
  bool ref_traj_acquired_ = false;
  bool ref_thrust_acquired_ = false;
  bool state_acquired_ = false;
  bool use_bypass_ = false;
  bool bypass_controller_ = false;

  uint8_t preferred_output_mode_ = 0b00000000;  // by default, no output mode is preferred

  rclcpp::Time last_time_;

  as2_msgs::msg::PlatformInfo platform_info_;
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  geometry_msgs::msg::PoseStamped state_pose_;
  geometry_msgs::msg::TwistStamped state_twist_;
  geometry_msgs::msg::PoseStamped ref_pose_;
  geometry_msgs::msg::TwistStamped ref_twist_;
  as2_msgs::msg::TrajectorySetpoints ref_traj_;
  as2_msgs::msg::Thrust ref_thrust_;
  geometry_msgs::msg::PoseStamped command_pose_;
  geometry_msgs::msg::TwistStamped command_twist_;
  as2_msgs::msg::Thrust command_thrust_;

  // Controller plugin
  std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_ptr_;

  // Lifetime-bound handle of the on_set_parameters callback registered with
  // the node. Releasing the shared_ptr unregisters the callback.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameters_callback_handle_;

  // Subscribers callbacks

  /**
   * @brief Subscription callback for `self_localization/twist`.
   *
   * Looks up the matching pose via TF, transforms both messages into the
   * desired frames and forwards the pair to ControllerBase::updateState.
   *
   * @param msg Latest twist message published by the localization stack.
   */
  void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief Subscription callback for `motion_reference/pose`.
   *
   * @param msg Latest pose reference message.
   */
  void refPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Subscription callback for `motion_reference/twist`.
   *
   * @param msg Latest twist reference message.
   */
  void refTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief Subscription callback for `motion_reference/trajectory`.
   *
   * @param msg Latest trajectory reference message.
   */
  void refTrajCallback(const as2_msgs::msg::TrajectorySetpoints::SharedPtr msg);

  /**
   * @brief Subscription callback for `motion_reference/thrust`.
   *
   * @param msg Latest thrust reference message.
   */
  void refThrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg);

  /**
   * @brief Subscription callback for `platform/info`.
   *
   * @param msg Latest platform info message used to track platform readiness.
   */
  void platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  // Services servers callbacks

  /**
   * @brief Service handler for `controller/set_control_mode`.
   *
   * Negotiates the input/output mode pair with the platform, applies the
   * negotiated mode to the plugin, and arms the hover latch when the
   * requested mode is HOVER.
   *
   * @param request Mode requested by the upstream client.
   * @param response Service response with the success flag.
   */
  void setControlModeSrvCall(
    const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
    as2_msgs::srv::SetControlMode::Response::SharedPtr response);

  /**
   * @brief Query the platform for the supported input control modes.
   *
   * @return true if the platform service responded successfully.
   */
  bool listPlatformAvailableControlModes();

  // Timer callbacks

  /**
   * @brief Periodic callback that runs one tick of the control loop.
   */
  void controlTimerCallback();

  // Internal methods

  /**
   * @brief Map a control-mode reference-frame enum to the corresponding TF frame id.
   *
   * @param reference_frame Reference-frame enum from as2_msgs::msg::ControlMode.
   * @return Fully-qualified TF frame id.
   */
  std::string getFrameIdByReferenceFrame(uint8_t reference_frame);

  /**
   * @brief Find a controller output mode that matches the platform's input mode.
   *
   * @param output_mode Output: matching output mode bitmask, if any.
   * @param input_mode Platform input mode the output must feed.
   * @return true if a match was found.
   */
  bool findSuitableOutputControlModeForPlatformInputMode(
    uint8_t & output_mode,
    const uint8_t input_mode);

  /**
   * @brief Check whether a controller input mode is compatible with a given output mode.
   *
   * @param input_mode In/out: input mode under evaluation; refined on success.
   * @param output_mode Output mode the input must feed.
   * @return true if the combination is supported.
   */
  bool checkSuitabilityInputMode(uint8_t & input_mode, const uint8_t output_mode);

  /**
   * @brief Send a `set_platform_control_mode` request to the platform.
   *
   * @param mode Control mode the platform should enter.
   * @return true if the platform accepted the new mode.
   */
  bool setPlatformControlMode(const as2_msgs::msg::ControlMode & mode);

  /**
   * @brief Find a self-consistent input/output mode pair given the active modes.
   *
   * @param input_mode In/out: candidate input mode; refined on success.
   * @param output_mode In/out: candidate output mode; refined on success.
   * @return true if a compatible pair was found.
   */
  bool findSuitableControlModes(uint8_t & input_mode, uint8_t & output_mode);

  /**
   * @brief Drive the platform into HOVER directly, bypassing the plugin.
   *
   * @return true if the platform accepted the hover request.
   */
  bool trySetPlatformHover();

  /**
   * @brief Negotiate a bypass path that skips the plugin for the requested input mode.
   *
   * @param input_mode Input mode the upstream client asked for.
   * @param output_mode Output: bypass output mode, if any.
   * @return true if the platform can ingest `input_mode` without the plugin.
   */
  bool tryToBypassController(const uint8_t input_mode, uint8_t & output_mode);

  /**
   * @brief Compute the controller output for the current tick.
   *
   * Builds the input messages from the cached references, calls
   * ControllerBase::computeOutput and stores the result for publishCommand.
   */
  void sendCommand();

  /**
   * @brief Publish the latest controller command on `actuator_command/*`.
   */
  void publishCommand();

  /**
   * @brief Read the optional `debug.<name>_topic` parameters and create the debug publishers.
   *
   * An empty topic name disables the corresponding publisher.
   */
  void initializeDebugPublishers();

  /**
   * @brief Publish the standard state and reference debug topics for the current tick.
   *
   * @param tick Timestamp shared by all standard debug messages.
   */
  void publishDebug(const rclcpp::Time & tick);
};  //  class ControllerHandler

}  //  namespace controller_handler

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_
