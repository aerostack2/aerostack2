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
 *  @file       differential_flatness_controller.hpp
 *  @brief      Declares the controller plugin differential flatness
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#ifndef DIFFERENTIAL_FLATNESS_CONTROLLER__DIFFERENTIAL_FLATNESS_CONTROLLER_HPP_
#define DIFFERENTIAL_FLATNESS_CONTROLLER__DIFFERENTIAL_FLATNESS_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

#include "as2_motion_controller/controller_base.hpp"

namespace differential_flatness_controller
{

/**
 * @brief Cached UAV state used by the differential-flatness controller.
 */
struct UAV_state
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  tf2::Quaternion attitude_state = tf2::Quaternion::getIdentity();
};

/**
 * @brief Trajectory reference fed to the differential-flatness controller.
 */
struct UAV_reference
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

/**
 * @brief Output command of the BODY_RATES mode: body rates plus thrust.
 */
struct BodyRates_command
{
  Eigen::Vector3d PQR = Eigen::Vector3d::Zero();
  double thrust = 0.0;
};

/**
 * @brief Differential-flatness controller plugin.
 */
class Plugin : public as2_motion_controller_plugin_base::ControllerBase
{
public:
  Plugin() {}
  ~Plugin() {}

  /**
   * @brief Plugin-specific initialization, called from initialize().
   */
  void ownInitialize() override;

  /**
   * @brief Control mode the plugin runs to hold its position.
   *
   * @return TRAJECTORY with yaw angle, the only law this plugin implements.
   */
  as2_msgs::msg::ControlMode hoverMode() const override;

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
   * @return Parameter names, without the plugin namespace.
   */
  std::vector<std::string> requiredParameters() const override;

  /**
   * @brief Accept a control mode pair.
   *
   * @param mode_in Input control mode, already resolved.
   * @param mode_out Output control mode requested.
   * @return true if the plugin can serve the pair.
   */
  bool onSetMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) override;

  /**
   * @brief Reset the cached state, references and commands.
   *
   * Calls ControllerBase::reset() to clear the base flags.
   */
  void reset() override;

  /**
   * @brief Plugin hook called by the base after frame validation.
   *
   * Caches the position, velocity and attitude used by the controller.
   *
   * @param pose_msg Latest validated pose message.
   * @param twist_msg Latest validated twist message.
   */
  void onUpdateState(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg) override;

  /**
   * @brief Plugin hook for trajectory reference.
   *
   * @param ref Latest trajectory reference message.
   */
  void onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & ref) override;

  /**
   * @brief Compute the output signal of the controller plugin.
   *
   * Solves the differential-flatness law and packs the resulting BODY_RATES
   * command into the twist (body rates) and thrust output messages.
   *
   * @param dt Time elapsed since the last call to computeOutput().
   * @param pose Output pose (unused by this plugin).
   * @param twist Output twist with the body rates in base_link.
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
   * @brief Compute the desired force from position/velocity tracking errors.
   *
   * @param _dt Time elapsed since the previous call, in seconds.
   * @param _pos_state Current position.
   * @param _vel_state Current linear velocity.
   * @param _pos_reference Reference position.
   * @param _vel_reference Reference linear velocity.
   * @param _acc_reference Reference linear acceleration (feed-forward).
   * @return Desired force expressed in the inertial frame.
   */
  Eigen::Vector3d getForce(
    const double & _dt,
    const Eigen::Vector3d & _pos_state,
    const Eigen::Vector3d & _vel_state,
    const Eigen::Vector3d & _pos_reference,
    const Eigen::Vector3d & _vel_reference,
    const Eigen::Vector3d & _acc_reference);

  /**
   * @brief Compute the BODY_RATES command for trajectory tracking.
   *
   * @param _dt Time elapsed since the previous call, in seconds.
   * @param _pos_state Current position.
   * @param _vel_state Current linear velocity.
   * @param _attitude_state Current attitude quaternion.
   * @param _pos_reference Reference position.
   * @param _vel_reference Reference linear velocity.
   * @param _acc_reference Reference linear acceleration (feed-forward).
   * @param _yaw_angle_reference Reference yaw angle.
   * @return BODY_RATES command (body rates + thrust).
   */
  BodyRates_command computeTrajectoryControl(
    const double & _dt,
    const Eigen::Vector3d & _pos_state,
    const Eigen::Vector3d & _vel_state,
    const tf2::Quaternion & _attitude_state,
    const Eigen::Vector3d & _pos_reference,
    const Eigen::Vector3d & _vel_reference,
    const Eigen::Vector3d & _acc_reference,
    const double & _yaw_angle_reference);

  /**
   * @brief Pack the latest BODY_RATES command into the output messages.
   *
   * @param twist_msg Output twist message (body rates).
   * @param thrust_msg Output thrust message.
   * @return true if the command was successfully packed.
   */
  bool getOutput(geometry_msgs::msg::TwistStamped & twist_msg, as2_msgs::msg::Thrust & thrust_msg);

  // Plugin state
  UAV_state uav_state_;
  UAV_reference control_ref_;
  BodyRates_command control_command_;


  // Controller gains and parameters
  Eigen::Matrix3d Kp_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Kd_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Ki_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d Kp_ang_mat_{Eigen::Matrix3d::Zero()};

  Eigen::Vector3d accum_pos_error_{Eigen::Vector3d::Zero()};

  double mass_;
  double antiwindup_cte_ = 0.0;

  const Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, -9.81);
};  // class Plugin

}  // namespace differential_flatness_controller

#endif  // DIFFERENTIAL_FLATNESS_CONTROLLER__DIFFERENTIAL_FLATNESS_CONTROLLER_HPP_
