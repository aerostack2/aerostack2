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
  explicit MultirotorSimulatorPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MultirotorSimulatorPlatform();

public:
  void configureSensors() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  void ownStopPlatform() override;
  void ownKillSwitch() override;
  bool ownTakeoff() override;
  bool ownLand() override;

  void gimbalControlCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg);

private:
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

  std::string frame_id_baselink_ = "base_link";
  std::string frame_id_odom_ = "odom";
  std::string frame_id_earth_ = "earth";

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
   * @brief Get parameter from the parameter server
   *
   * @param param_name Name of the parameter
   * @param param_value Value of the parameter
   * @param use_default Use default value if parameter is not found
  */
  template<typename T>
  inline void getParam(const std::string & param_name, T & param_value, bool use_default = false)
  {
    try {
      // Declare parameter if not declared
      if (!this->has_parameter(param_name)) {
        if (use_default) {
          this->declare_parameter<T>(param_name, param_value);
        } else {
          this->declare_parameter<T>(param_name);
        }
      }

      if constexpr (std::is_same<T, std::vector<double>>::value) {
        param_value = this->get_parameter(param_name).as_double_array();
      } else if constexpr (std::is_same<T, double>::value) {
        param_value = this->get_parameter(param_name).as_double();
      } else if constexpr (std::is_same<T, std::string>::value) {
        param_value = this->get_parameter(param_name).as_string();
      } else if constexpr (std::is_same<T, bool>::value) {
        param_value = this->get_parameter(param_name).as_bool();
      } else {
        RCLCPP_WARN(this->get_logger(), "Parameter type %s not expected", typeid(T).name());
        param_value = this->get_parameter<T>(param_name, param_value);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Error getting parameter %s: %s", param_name.c_str(), e.what());
    }
  }

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
