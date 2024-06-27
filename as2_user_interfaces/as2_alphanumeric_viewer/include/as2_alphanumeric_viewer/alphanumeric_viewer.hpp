// Copyright 2024 Universidad Politécnica de Madrid
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
 *  \file       alphanumeric_viewer.hpp
 *  \brief      Alphanumeric viewer header file.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_ALPHANUMERIC_VIEWER__ALPHANUMERIC_VIEWER_HPP_
#define AS2_ALPHANUMERIC_VIEWER__ALPHANUMERIC_VIEWER_HPP_

#include <curses.h>
#include <math.h>
#include <stdio.h>
#include <tf2/utils.h>
#include <iostream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/msg/controller_info.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

class AlphanumericViewer : public as2::Node
{
public:
  AlphanumericViewer();

  void run();
  void setupNode();
  void printSummaryMenu();
  void printSensorMenu();
  void printNavigationMenu();
  void printPlatformMenu();
  void printStream(double var, bool aux);
  void printStream3(float var, bool aux);
  void printStream(float var, bool aux);
  void printSummaryValues();
  void printNavigationValues();
  void printSensorValues();
  void printPlatformValues();
  void printBattery();
  void printQuadrotorState();
  void printControlModeInYaw();
  void printControlModeInControl();
  void printControlModeInFrame();
  void printControlModeOutYaw();
  void printControlModeOutControl();
  void printControlModeOutFrame();
  void printPlatformStatus(int line);
  void clearValues();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr self_localization_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr self_localization_speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr actuator_command_pose_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr actuator_command_thrust_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr actuator_command_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_reference_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_reference_sub_;
  // rclcpp::Subscription <as2_msgs::msg::TrajectoryWaypoints>::SharedPtr trajectory_reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<as2_msgs::msg::ControlMode>::SharedPtr control_mode_sub_;

  geometry_msgs::msg::PoseStamped self_localization_pose_;
  geometry_msgs::msg::TwistStamped self_localization_twist_;
  sensor_msgs::msg::BatteryState battery_status_;
  sensor_msgs::msg::Imu imu_;
  as2_msgs::msg::PlatformInfo platform_info_;
  geometry_msgs::msg::PoseStamped actuator_pose_;
  as2_msgs::msg::Thrust actuator_thrust_;
  geometry_msgs::msg::TwistStamped actuator_twist_;
  as2_msgs::msg::ControllerInfo controller_info_;
  geometry_msgs::msg::PoseStamped reference_pose_;
  geometry_msgs::msg::TwistStamped reference_twist_;
  sensor_msgs::msg::NavSatFix gps_;
  int battery_mode_ = 0;  // 0: [0, 1], 1: [0, 100]
  // as2_msgs::msg::TrajectoryWaypoints reference_traj_;

  std::stringstream interface_printout_stream;
  std::stringstream pinterface_printout_stream;

  bool battery_aux = false;
  bool altitude_aux = false;
  bool altitude_sea_level_aux = false;
  bool ground_speed_aux = false;
  bool imu_aux = false;
  bool temperature_aux = false;
  bool platform_info_aux = false;

  // Aux navigation
  bool current_speed_reference_aux = false;
  bool current_pose_reference_aux = false;
  bool current_trajectory_reference_aux = false;

  bool actuator_command_pose_aux = false;
  bool actuator_command_twist_aux = false;
  bool actuator_command_thrust_aux = false;
  bool current_pose_aux = false;
  bool current_speed_aux = false;
  bool controller_info_aux = false;
  bool gps_aux = false;
  bool thrust_aux = false;

  int last_received_yaw_mode;
  int last_received_control_mode;
  int last_received_reference_frame;

  char command = 0;

  int window = 0;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr _msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr _msg);
  void platformCallback(const as2_msgs::msg::PlatformInfo::SharedPtr _msg);
  void actuatorPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void actuatorThrustCallback(const as2_msgs::msg::Thrust::SharedPtr _msg);
  void actuatorSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg);
  void controllerCallback(const as2_msgs::msg::ControllerInfo::SharedPtr _msg);
  void poseReferenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void speedReferenceCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg);
  // void trajectoryReferenceCallback (const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg);
};

#endif  // AS2_ALPHANUMERIC_VIEWER__ALPHANUMERIC_VIEWER_HPP_
