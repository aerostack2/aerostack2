/*!*******************************************************************************************
 *  \file       pixhawk_platform.hpp
 *  \brief      Implementation of PX4 Autopilot UAV platform
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef PIXHAWK_PLATFORM_HPP_
#define PIXHAWK_PLATFORM_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <string>

#include <as2_core/utils/frame_utils.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PixhawkPlatform : public as2::AerialPlatform {
public:
  PixhawkPlatform();
  ~PixhawkPlatform(){};

public:
  void configureSensors();
  void publishSensorData();

  // TODO: set ATTITUDE as default mode with yaw_speed = 0  and Thrust = 0 N
  void setDefaultControlMode(){};

  bool ownSetArmingState(bool state);
  bool ownSetOffboardControl(bool offboard);
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg);
  void sendCommand() override;
  bool ownSendCommand();
  void ownKillSwitch() override;
  void ownStopPlatform() override;

  void resetTrajectorySetpoint();
  void resetAttitudeSetpoint();
  void resetRatesSetpoint();

  bool getFlagSimulationMode();

private:
  bool has_mode_settled_ = false;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::unique_ptr<as2::sensors::GPS> gps_sensor_ptr_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;
  void externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // PX4 subscribers
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr px4_vehicle_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr px4_timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr px4_battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_sub_;

  // PX4 publishers
  rclcpp::Publisher<px4_msgs::msg::ManualControlSwitches>::SharedPtr
      px4_manual_control_switches_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
      px4_vehicle_attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr px4_vehicle_rates_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr px4_visual_odometry_pub_;

  // PX4 Functions
  void PX4arm() const;
  void PX4disarm() const;
  void PX4publishOffboardControlMode();
  void PX4publishTrajectorySetpoint();
  void PX4publishAttitudeSetpoint();
  void PX4publishRatesSetpoint();
  void PX4publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
  void PX4publishVisualOdometry();

private:
  bool manual_from_operator_ = false;
  bool set_disarm_           = false;
  nav_msgs::msg::Odometry odometry_msg_;

  std::atomic<uint64_t> timestamp_;

  px4_msgs::msg::OffboardControlMode px4_offboard_control_mode_;
  px4_msgs::msg::TrajectorySetpoint px4_trajectory_setpoint_;
  px4_msgs::msg::VehicleAttitudeSetpoint px4_attitude_setpoint_;
  px4_msgs::msg::VehicleRatesSetpoint px4_rates_setpoint_;
  px4_msgs::msg::VehicleVisualOdometry px4_visual_odometry_msg_;

  float mass_;
  float max_thrust_;
  float min_thrust_;
  bool simulation_mode_ = false;
  bool external_odom_   = true;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  Eigen::Quaterniond q_ned_to_enu_;
  Eigen::Quaterniond q_enu_to_ned_;
  Eigen::Quaterniond q_aircraft_to_baselink_;
  Eigen::Quaterniond q_baselink_to_aircraft_;

private:
  // PX4 Callbacks
  void px4imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void px4VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  void px4BatteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void px4GpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg);
};

#endif  // PIXHAWK_PLATFORM_HPP_
