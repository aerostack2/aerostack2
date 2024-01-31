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

/*******************************************************************************************
 *  \file       topics.hpp
 *  \brief      This file contains the definition of the topics used by the AS2 framework.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *******************************************************************************************/

#ifndef AS2_CORE__NAMES__TOPICS_HPP_
#define AS2_CORE__NAMES__TOPICS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace as2_names
{
namespace topics
{
namespace global
{
const rclcpp::QoS qos = rclcpp::QoS(10);
const char alert_event[] = "alert_event";
}  // namespace global
namespace sensor_measurements
{
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char base[] = "sensor_measurements/";
const char imu[] = "sensor_measurements/imu";
const char lidar[] = "sensor_measurements/lidar";
const char gps[] = "sensor_measurements/gps";
const char camera[] = "sensor_measurements/camera";
const char battery[] = "sensor_measurements/battery";
const char odom[] = "sensor_measurements/odom";
}  // namespace sensor_measurements
namespace ground_truth
{
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char pose[] = "ground_truth/pose";
const char twist[] = "ground_truth/twist";
}  // namespace ground_truth
namespace self_localization
{
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char odom[] = "self_localization/odom";
const char pose[] = "self_localization/pose";
const char twist[] = "self_localization/twist";
}  // namespace self_localization
namespace motion_reference
{
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const rclcpp::QoS qos_waypoint = rclcpp::QoS(10);
const rclcpp::QoS qos_trajectory = rclcpp::QoS(10);
const char pose[] = "motion_reference/pose";
const char twist[] = "motion_reference/twist";
const char trajectory[] = "motion_reference/trajectory";
const char waypoints[] = "motion_reference/waypoints";
const char modify_waypoint[] = "motion_reference/modify_waypoint";
const char traj_gen_info[] = "motion_reference/traj_gen_info";
}  // namespace motion_reference
namespace actuator_command
{
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char pose[] = "actuator_command/pose";
const char twist[] = "actuator_command/twist";
const char thrust[] = "actuator_command/thrust";
const char trajectory[] = "actuator_command/trajectory";
}  // namespace actuator_command
namespace platform
{
const rclcpp::QoS qos = rclcpp::QoS(10);
const char info[] = "platform/info";
}  // namespace platform
namespace controller
{
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const char info[] = "controller/info";
}  // namespace controller
namespace follow_target
{
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const char info[] = "follow_target/info";
}  // namespace follow_target
}  // namespace topics
}  // namespace as2_names
#endif  // AS2_CORE__NAMES__TOPICS_HPP_
