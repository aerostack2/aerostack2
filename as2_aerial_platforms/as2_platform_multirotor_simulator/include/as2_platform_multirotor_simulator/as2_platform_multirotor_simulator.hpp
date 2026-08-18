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

/**
 * @file as2_platform_multirotor_simulator.hpp
 *
 * MultirotorSimulatorPlatform class definition
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_PLATFORM_MULTIROTOR_SIMULATOR_HPP_
#define AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_PLATFORM_MULTIROTOR_SIMULATOR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/gps_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/gimbal_control.hpp"
#include "as2_interface.hpp"
#include "multirotor_simulator.hpp"

namespace as2_platform_multirotor_simulator
{

struct PlatformParams
{
  double update_freq = 1000.0;
  double control_freq = 100.0;
  double inertial_odometry_freq = 1000.0;
  double state_freq = 100.0;
  double imu_pub_freq = 100.0;
  double odometry_pub_freq = 100.0;
  double ground_truth_pub_freq = 100.0;
  double gps_pub_freq = 100.0;
  double gimbal_pub_freq = 100.0;
  double latitude;
  double longitude;
  double altitude;
};

class MultirotorSimulatorPlatform : public as2::AerialPlatform
{
  using Simulator = multirotor::Simulator<double, 4>;
  using SimulatorParams = multirotor::SimulatorParams<double, 4>;
  using Kinematics = multirotor::state::internal::Kinematics<double>;

public:
  /**
   * @brief Construct the platform, set the simulator up and start its timers.
   *
   * @param options Node options.
   */
  explicit MultirotorSimulatorPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Multirotor Simulator Platform object.
   */
  ~MultirotorSimulatorPlatform();

public:
  /**
   * @brief Create the sensor interfaces the platform publishes.
   */
  void configureSensors() override;

  /**
   * @brief Arm or disarm the simulated vehicle.
   *
   * @param state True to arm, false to disarm.
   * @return true if the request was applied.
   */
  bool ownSetArmingState(bool state) override;

  /**
   * @brief Enter or leave offboard control.
   *
   * @param offboard True to take control, false to release it.
   * @return true if the request was applied.
   */
  bool ownSetOffboardControl(bool offboard) override;

  /**
   * @brief Set the control mode of the simulator controller.
   *
   * @param msg Requested control mode.
   * @return true if the simulator supports the mode.
   */
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;

  /**
   * @brief Feed the current actuator commands to the simulator, as its reference.
   *
   * @return true if the reference was applied.
   */
  bool ownSendCommand() override;

  /**
   * @brief Hold the vehicle in place with a zero reference.
   */
  void ownStopPlatform() override;

  /**
   * @brief Stop the motors immediately, without landing.
   */
  void ownKillSwitch() override;

  /**
   * @brief Take off with the platform own takeoff routine.
   *
   * @return true if the takeoff finished successfully.
   */
  bool ownTakeoff() override;

  /**
   * @brief Land with the platform own landing routine.
   *
   * @return true if the landing finished successfully.
   */
  bool ownLand() override;

  /**
   * @brief Store a gimbal reference, applied by the simulator on its next step.
   *
   * @param msg Gimbal control reference.
   */
  void gimbalControlCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg);

private:
  As2MultirotorSimulatorInterface as2_interface_;

  as2::gps::GpsHandler gps_handler_;
  PlatformParams platform_params_;
  Simulator simulator_;
  SimulatorParams simulator_params_;
  geometry_msgs::msg::Point initial_position_;
  Kinematics control_state_;
  bool using_odom_for_control_ = false;

  rclcpp::TimerBase::SharedPtr simulator_timer_;
  rclcpp::TimerBase::SharedPtr simulator_control_timer_;
  rclcpp::TimerBase::SharedPtr simulator_inertial_odometry_timer_;
  rclcpp::TimerBase::SharedPtr simulator_state_pub_timer_;

  std::string frame_id_baselink_;
  std::string frame_id_earth_;

  // Gimbal
  geometry_msgs::msg::QuaternionStamped gimbal_desired_orientation_;
  rclcpp::Subscription<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_control_sub_;

  // Publisher state

  // Ground truth
  std::unique_ptr<as2::sensors::GroundTruth> sensor_ground_truth_ptr_;
  // Odometry
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> sensor_odom_estimate_ptr_;
  // IMU
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::Imu>> sensor_imu_ptr_;
  // GPS
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::NavSatFix>> sensor_gps_ptr_;
  // Gimabl
  std::unique_ptr<as2::sensors::Gimbal> sensor_gimbal_ptr_;

private:
  /**
   * @brief Read given parameter of vector type
   *
   * @param param_name Name of the parameter
   *
   * @return Eigen::Vector3d Vector parameter
  */
  Eigen::Vector3d readVectorParams(const std::string & param_name);

  /**
   * @brief Read platform parameters
   *
   * @param param_name platform parameters
  */
  inline void readParams(PlatformParams & platform_params);

  /**
   * @brief Simulator timer callback
  */
  void simulatorTimerCallback();

  /**
   * @brief Simulator control timer callback
  */
  void simulatorControlTimerCallback();

  /**
   * @brief Simulator inertial odometry timer callback
  */
  void simulatorInertialOdometryTimerCallback();

  /**
   * @brief Simulator state timer callback
  */
  void simulatorStateTimerCallback();
};  // class MultirotorSimulatorPlatform

}  // namespace as2_platform_multirotor_simulator

#endif  // AS2_PLATFORM_MULTIROTOR_SIMULATOR__AS2_PLATFORM_MULTIROTOR_SIMULATOR_HPP_
