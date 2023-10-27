/*!*******************************************************************************************
 *  \file       tello_platform.cpp
 *  \brief      Implements the functionality and communication with the tello drone.
 *  \authors    Daniel Fernández Sánchez
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *
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

#ifndef TELLO_PLATFORM_H
#define TELLO_PLATFORM_H

#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "opencv2/core.hpp"

#include "tello/tello.hpp"

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

class TelloPlatform : public as2::AerialPlatform {
private:
  std::unique_ptr<Tello> tello;
  double camera_freq_;
  double sensor_freq_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::shared_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_ptr_;
  std::shared_ptr<as2::sensors::Battery> battery_ptr_;
  std::shared_ptr<as2::sensors::Barometer> barometer_ptr_;
  rclcpp::TimerBase::SharedPtr cam_timer_;
  std::shared_ptr<as2::sensors::Camera> camera_ptr_;
  nav_msgs::msg::Odometry odom_msg_;
  bool connected_ = false;
  as2_msgs::msg::ControlMode control_mode_in_;
  double min_speed_;
  double max_speed_;
  const double max_linear_pose_ = 5;    // m
  const double min_linear_pose_ = 0.2;  // m

  std::vector<double> reference_point_ = {0.0, 0.0, 0.0, 0.0};  // x, y, z, yaw
  std::vector<double> reference_speed_ = {0.0, 0.0, 0.0, 0.0};  // vx, vy, vz, vyaw

  std::string odom_frame_id_      = "odom";
  std::string base_link_frame_id_ = "base_link";

private:
  void recvIMU();
  void recvBattery();
  void recvBarometer();
  void recvOdometry();
  void recvVideo();
  void resetOdometry();

  double normalize(double value, double min_value, double max_value);
  double normalizeDegrees(double value);

public:
  TelloPlatform();
  ~TelloPlatform();
  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) override;
  void ownKillSwitch() override { tello->sendCommand("emergency", false); };
  void ownStopPlatform() override {
    RCLCPP_INFO(this->get_logger(), "Stopping platform");
    tello->sendCommand("rc 0 0 0 0", false);
  };

  bool ownTakeoff() override;
  bool ownLand() override;
};

#endif  // TELLO_PLATFORM_H
