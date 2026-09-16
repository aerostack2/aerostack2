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
 *  @file       geometric_controller.hpp
 *  @brief      Geometric controller plugin.
 *  @authors    Rafael Pérez Seguí
 ********************************************************************************************/

#ifndef GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_motion_controller/controller_base.hpp"
#include "as2_motion_controller/param_utils.hpp"

#include "pid_controller/pid_1d.hpp"

#include "geometric_controllers/geometric_controller.hpp"
#include "pid_controllers/position_controller.hpp"
#include "pid_controllers/trajectory_controller.hpp"
#include "pid_controllers/velocity_controller.hpp"

namespace geometric_controller
{

/**
 * @brief Geometric controller plugin.
 */
class Plugin : public as2_motion_controller_plugin_base::ControllerBase
{
public:
  Plugin() {}
  ~Plugin() {}

  /**
   * @brief Plugin-specific initialization, called from initialize().
   *
   * Creates the plugin-specific debug publishers from the optional
   * `debug.<name>_topic` parameters.
   */
  void ownInitialize() override;

  /**
   * @brief Apply one parameter of the plugin to the controller.
   *
   * @param name Parameter name, without the plugin namespace.
   * @param param Parameter as delivered.
   */
  void updateParameter(
    const std::string & name,
    const rclcpp::Parameter & param) override;

  /**
   * @brief Names of the parameters the plugin needs before it can control.
   *
   * @return Gains, limits and mass of every loop of the cascade.
   */
  std::vector<std::string> requiredParameters() const override;

  /**
   * @brief Reset the plugin state.
   *
   * Calls ControllerBase::reset() to clear the base flags and invalidates
   * the local reference cache.
   */
  void reset() override;

  /**
   * @brief Control mode the plugin runs to perform a hover request.
   *
   * @return POSITION when the position loop is usable, SPEED when only the
   *         velocity loop is, UNSET when neither can hold.
   */
  as2_msgs::msg::ControlMode hoverMode() const override;

  /**
   * @brief Accept a control mode pair and clear the stale references.
   *
   * @param mode_in Input control mode, already resolved.
   * @param mode_out Output control mode requested.
   * @return true if the loops the pair relies on have usable gains.
   */
  bool onSetMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) override;

  /**
   * @brief Plugin hook called by the base after frame validation.
   *
   * Caches the position, velocity and orientation used by the controller.
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
   * @param ref Latest trajectory setpoints message.
   */
  void onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & ref) override;

  /**
   * @brief Compute the output signal of the controller plugin.
   *
   * Cascades the position controller into the velocity controller and the
   * geometric attitude controller, packing the resulting command into the
   * output messages.
   *
   * @param dt Time elapsed since the last call to computeOutput().
   * @param pose Output pose (unused by this plugin).
   * @param twist Output twist (body rates).
   * @param thrust Output collective thrust.
   * @return true if the output is valid.
   */
  bool computeOutput(
    double dt,
    geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::TwistStamped & twist,
    as2_msgs::msg::Thrust & thrust) override;

private:
  /**
   * @brief Apply one parameter of a PID loop.
   *
   * @param pid_handler Loop the parameter applies to.
   * @param name Parameter name inside the loop block.
   * @param param Parameter as delivered.
   * @param with_alpha Whether the loop filters its derivative term.
   * @param with_saturation Whether the loop takes output saturation limits.
   * @return true if the loop owns a parameter with that name.
   */
  bool updatePidParameter(
    pid_controller::PID<double, 3> & pid_handler,
    const std::string & name,
    const rclcpp::Parameter & param,
    bool with_alpha,
    bool with_saturation);

  /**
   * @brief Update one availability flag, warning when the loop is unusable.
   *
   * @param _flag Flag to update.
   * @param _enabled Whether the loop can actuate with its current gains.
   * @param _reason Message logged when the loop is unusable.
   */
  void setLoopEnabled(bool & _flag, const bool _enabled, const char * _reason);

  /**
   * @brief Refresh the per-loop availability flags from the live controllers.
   */
  void refreshAvailability();

  /**
   * @brief Whether the loops a control mode relies on have usable gains.
   *
   * A loop with zero proportional gains, a non-positive mass or a non-positive
   * rotation gain on any axis produces no actuation, so the modes that need it
   * are not available.
   *
   * @param mode_in Input control mode to check.
   * @param mode_out Output control mode, which decides where the cascade stops.
   * @return true if the mode can be controlled with the current parameters.
   */
  bool isControlModeAvailable(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) const;

  /**
   * @brief Yaw angle reference that makes the attitude loop generate a yaw rate.
   *
   * Inverting the proportional rotation gain places the angle reference where
   * the attitude loop generates the requested rate, so no integrator is needed.
   * A non-positive gain cannot be inverted and holds the current heading.
   *
   * @param yaw_speed Requested yaw rate, in rad/s.
   * @param current_orientation Current attitude of the vehicle.
   * @param kp_yaw Proportional rotation gain of the yaw axis, in 1/s.
   * @return Yaw angle reference, in rad.
   */
  /**
   * @brief Yaw rate the SPEED output is commanded with.
   *
   * The SPEED output carries the yaw as a rate, so a yaw angle reference is
   * closed by the yaw loop and a yaw rate reference is forwarded.
   *
   * @param dt Time elapsed since the last call, in seconds.
   * @return Yaw rate, in rad/s.
   */
  double getYawSpeedCommand(const double dt);

  /**
   * @brief Requested yaw rate after applying the configured limit.
   *
   * @return Yaw rate, in rad/s.
   */
  double getLimitedYawSpeed() const;

  static double yawSpeedToYawAngle(
    const double yaw_speed,
    const Eigen::Quaterniond & current_orientation,
    const double kp_yaw);

  /**
   * @brief Publish the plugin-specific debug topics for the current tick.
   *
   * @param stamp Timestamp used for the published messages.
   */
  void publishPluginDebug(const rclcpp::Time & stamp);


  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d ref_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_acceleration_ = Eigen::Vector3d::Zero();
  double ref_yaw_ = 0.0;
  double ref_yaw_speed_ = 0.0;
  Eigen::Vector3d desired_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_acceleration_ = Eigen::Vector3d::Zero();

  double yaw_speed_limit_ = 0.0;

  // Disabled until the gains prove otherwise.
  bool position_enabled_ = false;
  bool yaw_enabled_ = false;
  bool velocity_enabled_ = false;
  bool trajectory_enabled_ = false;
  bool geometric_enabled_ = false;

  pid_1d_controller::PID<double> yaw_controller_;
  pid_controllers::PositionController<double> position_controller_;
  pid_controllers::VelocityController<double> velocity_controller_;
  pid_controllers::TrajectoryController<double> trajectory_controller_;
  geometric_controllers::GeometricController<double> geometric_controller_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    debug_desired_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr
    debug_acceleration_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    debug_velocity_norm_pub_;
};   // class Plugin

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__GEOMETRIC_CONTROLLER_HPP_
