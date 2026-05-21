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
 *  @file       pid_speed_controller_plugin.hpp
 *  @brief      Speed PID controller plugin for the Aerostack framework.
 *  @authors    Rafael Perez-Segui
 *              Miguel Fernández Cortizas
 ********************************************************************************************/

#ifndef PID_SPEED_CONTROLLER__PID_SPEED_CONTROLLER_HPP_
#define PID_SPEED_CONTROLLER__PID_SPEED_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_motion_controller/controller_base.hpp"
#include "pid_controller/pid.hpp"
#include "pid_controller/pid_1d.hpp"

namespace pid_speed_controller
{

/**
 * @brief Cached UAV state used by the PID handlers.
 */
struct UAV_state
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw = Eigen::Vector3d::Zero();
};

/**
 * @brief Output command produced by the PID handlers.
 */
struct UAV_command
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw_speed = 0.0;
};

/**
 * @brief Per-mode readiness for the optional gain groups.
 *
 * The base already gates the essential groups (plugin / position_control /
 * yaw_control) via essentialParamsReady(); these flags add the per-mode
 * gating that the base does not know about (TRAJECTORY needs
 * trajectory_control gains; SPEED / SPEED_IN_A_PLANE need speed_control
 * gains when !use_bypass_).
 */
struct ModeParametersRead
{
  bool velocity = false;
  bool speed_in_a_plane = false;
  bool trajectory = false;
};

/**
 * @brief PID-based speed controller plugin.
 */
class Plugin : public as2_motion_controller_plugin_base::ControllerBase
{
  using PID = pid_controller::PID<double>;
  using PID_1D = pid_1d_controller::PID<double>;

public:
  Plugin() {}
  ~Plugin() {}

  /**
   * @brief Plugin-specific initialization, called from initialize().
   */
  void ownInitialize() override;

  /**
   * @brief Names of the parameters required before the plugin can accept setMode.
   *
   * Returns the fully-namespaced names of the essential PID gain groups
   * (plugin / position_control / yaw_control). Optional groups are tracked
   * separately via params_read_.
   *
   * @return Vector of fully-qualified essential parameter names.
   */
  std::vector<std::string> getEssentialParameters() const override;

  /**
   * @brief Apply a single parameter to the plugin.
   *
   * Routes the value to the corresponding PID handler and toggles the
   * plugin-side flags (use_bypass_, proportional_limitation_). Tracks the
   * optional gain groups in params_read_ so setMode can refuse modes whose
   * gains have not been delivered yet.
   *
   * @param parameter Parameter to apply.
   */
  void updateParameter(const rclcpp::Parameter & parameter) override;

  /**
   * @brief Reset the cached state, references and commands.
   *
   * Calls ControllerBase::reset() to clear the base flags. The
   * essentialParamsReady() latch is intentionally preserved.
   */
  void reset() override;

  /**
   * @brief Update the control mode to be used by the controller plugin.
   *
   * Validates that the requested mode is supported by the active gain
   * groups, configures the output twist frame id, and resets the integrators
   * of the affected PID handlers.
   *
   * @param mode_in Input control mode requested.
   * @param mode_out Output control mode requested.
   * @return true if the in-out control mode configuration is valid.
   */
  bool setMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) override;

  /**
   * @brief Plugin hook called by the base after frame validation and hover latch.
   *
   * Caches the position, velocity and yaw used by the PID handlers.
   *
   * @param pose_msg Latest validated pose message.
   * @param twist_msg Latest validated twist message.
   */
  void onUpdateState(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg) override;

  /**
   * @brief Plugin hook for pose reference.
   *
   * @param ref Latest pose reference message.
   */
  void onUpdateReference(const geometry_msgs::msg::PoseStamped & ref) override;

  /**
   * @brief Plugin hook for twist reference.
   *
   * @param ref Latest twist reference message.
   */
  void onUpdateReference(const geometry_msgs::msg::TwistStamped & ref) override;

  /**
   * @brief Plugin hook for trajectory reference.
   *
   * @param ref Latest trajectory reference message.
   */
  void onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & ref) override;

  /**
   * @brief Hover latch override.
   *
   * Synthesizes the reference directly into control_ref_ (zero velocity at
   * the cached pose) because onUpdateReference(TrajectorySetpoints) is gated
   * to TRAJECTORY mode and would reject the default base-class latch.
   *
   * @param pose Cached state pose used as the hover anchor.
   * @param twist Cached state twist (unused).
   */
  void latchHoverReference(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & twist) override;

  /**
   * @brief Compute the output signal of the controller plugin.
   *
   * Runs the active PID handler on the cached state/reference and packs the
   * twist command in the output_twist_frame_id_.
   *
   * @param dt Time elapsed since the last call to computeOutput().
   * @param pose Output pose (unused by this plugin).
   * @param twist Output twist in the output reference frame.
   * @param thrust Output thrust (unused by this plugin).
   * @return true if the output is valid.
   */
  bool computeOutput(
    double dt,
    geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::TwistStamped & twist,
    as2_msgs::msg::Thrust & thrust) override;

private:
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  ModeParametersRead params_read_;

  PID_1D pid_yaw_handler_;
  PID pid_3D_position_handler_;
  PID pid_3D_velocity_handler_;
  PID_1D pid_1D_speed_in_a_plane_handler_;
  PID pid_3D_speed_in_a_plane_handler_;
  PID pid_3D_trajectory_handler_;

  // Tail names of the parameters tracked by the plugin. The full names are
  // prefixed with the plugin namespace at runtime via ControllerBase::param().
  const std::vector<std::string> plugin_parameters_tail_ =
  {"proportional_limitation", "use_bypass"};

  const std::vector<std::string> position_control_parameters_tail_ = {
    "position_control.reset_integral", "position_control.antiwindup_cte",
    "position_control.alpha", "position_control.kp.x",
    "position_control.kp.y", "position_control.kp.z",
    "position_control.ki.x", "position_control.ki.y",
    "position_control.ki.z", "position_control.kd.x",
    "position_control.kd.y", "position_control.kd.z"};

  const std::vector<std::string> velocity_control_parameters_tail_ = {
    "speed_control.reset_integral", "speed_control.antiwindup_cte", "speed_control.alpha",
    "speed_control.kp.x", "speed_control.kp.y", "speed_control.kp.z",
    "speed_control.ki.x", "speed_control.ki.y", "speed_control.ki.z",
    "speed_control.kd.x", "speed_control.kd.y", "speed_control.kd.z"};

  const std::vector<std::string> speed_in_a_plane_control_parameters_tail_ = {
    "speed_in_a_plane_control.reset_integral", "speed_in_a_plane_control.antiwindup_cte",
    "speed_in_a_plane_control.alpha", "speed_in_a_plane_control.height.kp",
    "speed_in_a_plane_control.height.ki", "speed_in_a_plane_control.height.kd",
    "speed_in_a_plane_control.speed.kp.x", "speed_in_a_plane_control.speed.kp.y",
    "speed_in_a_plane_control.speed.ki.x", "speed_in_a_plane_control.speed.ki.y",
    "speed_in_a_plane_control.speed.kd.x", "speed_in_a_plane_control.speed.kd.y"};

  const std::vector<std::string> trajectory_control_parameters_tail_ = {
    "trajectory_control.reset_integral", "trajectory_control.antiwindup_cte",
    "trajectory_control.alpha", "trajectory_control.kp.x",
    "trajectory_control.kp.y", "trajectory_control.kp.z",
    "trajectory_control.ki.x", "trajectory_control.ki.y",
    "trajectory_control.ki.z", "trajectory_control.kd.x",
    "trajectory_control.kd.y", "trajectory_control.kd.z"};

  const std::vector<std::string> yaw_control_parameters_tail_ = {"yaw_control.reset_integral",
    "yaw_control.antiwindup_cte",
    "yaw_control.alpha",
    "yaw_control.kp",
    "yaw_control.ki",
    "yaw_control.kd"};

  // Mutable copies of the optional-group tails. Decremented as parameters
  // arrive in updateParameter(); when a list empties, the corresponding
  // params_read_ flag flips and gates setMode for that mode. The essential
  // groups (plugin / position / yaw) are tracked by the base.
  std::vector<std::string> velocity_control_parameters_to_read_;
  std::vector<std::string> speed_in_a_plane_control_parameters_to_read_;
  std::vector<std::string> trajectory_control_parameters_to_read_;

  UAV_state uav_state_;
  UAV_state control_ref_;
  UAV_command control_command_;

  Eigen::Vector3d speed_limits_;
  double yaw_speed_limit_;

  bool use_bypass_ = true;
  bool proportional_limitation_ = false;

  // Frame id used for the published twist command. Updated in setMode() to
  // match the output reference frame negotiated by the controller. Pose and
  // twist input frames are owned by ControllerBase and accessed via
  // getDesiredPoseFrameId/getDesiredTwistFrameId.
  std::string output_twist_frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    debug_desired_velocity_pub_;

private:
  /**
   * @brief Mark a parameter as read inside an optional-group tail list.
   *
   * @param param Parameter name (already namespaced) to remove.
   * @param _params_list In/out tail list still pending; entries are erased on hit.
   * @param _all_params_read Out: flipped to true once `_params_list` is empty.
   */
  void checkParamList(
    const std::string & param,
    std::vector<std::string> & _params_list,
    bool & _all_params_read);

  /**
   * @brief Apply a parameter change to a 1-D PID handler.
   *
   * @param _pid_handler PID handler to configure.
   * @param _parameter_name Tail name of the parameter (without plugin namespace).
   * @param _param New parameter value.
   */
  void updateControllerParameter(
    PID_1D & _pid_handler,
    const std::string & _parameter_name,
    const rclcpp::Parameter & _param);

  /**
   * @brief Apply a parameter change to a 3-D PID handler.
   *
   * @param _pid_handler PID handler to configure.
   * @param _parameter_name Tail name of the parameter (without plugin namespace).
   * @param _param New parameter value.
   */
  void updateController3DParameter(
    PID & _pid_handler,
    const std::string & _parameter_name,
    const rclcpp::Parameter & _param);

  /**
   * @brief Apply a parameter change to the speed-in-a-plane PID pair.
   *
   * @param _pid_1d_handler 1-D handler used for the height PID.
   * @param _pid_3d_handler 3-D handler used for the planar PIDs.
   * @param _parameter_name Tail name of the parameter (without plugin namespace).
   * @param _param New parameter value.
   */
  void updateSpeedInAPlaneParameter(
    PID_1D & _pid_1d_handler,
    PID & _pid_3d_handler,
    const std::string & _parameter_name,
    const rclcpp::Parameter & _param);

  /**
   * @brief Reset the cached UAV state.
   */
  void resetState();

  /**
   * @brief Reset the cached references.
   */
  void resetReferences();

  /**
   * @brief Reset the cached commands.
   */
  void resetCommands();

  /**
   * @brief Compute the twist command from the current state and reference.
   *
   * @param twist_msg Output twist message filled by the controller.
   * @return true if the command was successfully computed.
   */
  bool getOutput(geometry_msgs::msg::TwistStamped & twist_msg);
};  // class Plugin
}   // namespace pid_speed_controller

#endif  // PID_SPEED_CONTROLLER__PID_SPEED_CONTROLLER_HPP_
