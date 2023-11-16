/*!*******************************************************************************************
 *  \file       topics.hpp
 *  \brief      This file contains the definition of the topics used by the AS2 framework.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#ifndef __AS2_NAMES_TOPICS_HPP__
#define __AS2_NAMES_TOPICS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names {
namespace topics {
namespace global {
const rclcpp::QoS qos         = rclcpp::QoS(10);
const std::string alert_event = "alert_event";
}  // namespace global
namespace sensor_measurements {
const rclcpp::QoS qos     = rclcpp::SensorDataQoS();
const std::string base    = "sensor_measurements/";
const std::string imu     = base + "imu";
const std::string lidar   = base + "lidar";
const std::string gps     = base + "gps";
const std::string camera  = base + "camera";
const std::string battery = base + "battery";
const std::string odom    = base + "odom";
}  // namespace sensor_measurements
namespace ground_truth {
const rclcpp::QoS qos   = rclcpp::SensorDataQoS();
const std::string pose  = "ground_truth/pose";
const std::string twist = "ground_truth/twist";
}  // namespace ground_truth
namespace self_localization {
const rclcpp::QoS qos   = rclcpp::SensorDataQoS();
const std::string odom  = "self_localization/odom";
const std::string pose  = "self_localization/pose";
const std::string twist = "self_localization/twist";
}  // namespace self_localization
namespace motion_reference {
const rclcpp::QoS qos        = rclcpp::SensorDataQoS();
const std::string pose       = "motion_reference/pose";
const std::string twist      = "motion_reference/twist";
const std::string trajectory = "motion_reference/trajectory";
// const rclcpp::QoS qos_wp = rclcpp::ServicesQoS();
// const std::string wayp = "motion_reference/waypoints";
const rclcpp::QoS qos_waypoint    = rclcpp::QoS(10);
const std::string modify_waypoint = "motion_reference/modify_waypoint";

const std::string traj_gen_info = "traj_gen/info";
const rclcpp::QoS traj_gen_qos  = rclcpp::QoS(10);
}  // namespace motion_reference
namespace actuator_command {
const rclcpp::QoS qos        = rclcpp::SensorDataQoS();
const std::string pose       = "actuator_command/pose";
const std::string twist      = "actuator_command/twist";
const std::string thrust     = "actuator_command/thrust";
const std::string trajectory = "actuator_command/trajectory";
}  // namespace actuator_command
namespace platform {
const rclcpp::QoS qos  = rclcpp::QoS(10);
const std::string info = "platform/info";
}  // namespace platform
namespace controller {
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const std::string info     = "controller/info";
}  // namespace controller
namespace follow_target {
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const std::string info     = "follow_target/info";
}  // namespace follow_target
}  // namespace topics
}  // namespace as2_names
#endif  // __AS2_NAMES_TOPICS_HPP__
