/*!*******************************************************************************************
 *  \file       crazyflie_platform.hpp
 *  \brief      Header for the crazyflie_platform interface.
 *  \authors    Miguel Granero Ramos
 *              Miguel Fernández Cortizas
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
#ifndef CRAZYFLIE_PLATFORM_HPP_
#define CRAZYFLIE_PLATFORM_HPP_

#include <Crazyflie.h>
#include <Eigen/Dense>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/bool.hpp"

struct logBattery {
  float pm_vbat;
  uint8_t charge_percent;
} __attribute__((packed));

class CrazyfliePlatform : public as2::AerialPlatform {
  as2::tf::TfHandler tf_handler_;
  std::string base_frame_;
  std::string odom_frame_;

public:
  void init();
  CrazyfliePlatform();
  CrazyfliePlatform(const std::string &ns, const std::string &radio_uri);
  void configureParams(const std::string &radio_uri = "");

  /*  --  AS2 FUNCTIONS --  */

  void configureSensors() override;

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) override;
  bool ownSendCommand() override;
  void ownKillSwitch() override { cf_->emergencyStop(); }
  void ownStopPlatform() override { cf_->sendStop(); };

  /*  --  CRAZYFLIE FUNCTIONS --  */

  void listVariables();
  void pingCB();
  void onLogIMU(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
  void onLogOdomOri(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
  void onLogOdomPos(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
  void onLogBattery();
  void onLogRange(uint32_t time_in_ms, std::vector<double> *values, void * /*userData*/);
  void updateOdom();
  void externalOdomCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /*  --  AUX FUNCTIONS --  */

  Eigen::Vector3d quaternion2Euler(geometry_msgs::msg::Quaternion quat);

private:
  std::shared_ptr<Crazyflie> cf_;
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::TimerBase::SharedPtr bat_timer_;
  bool is_connected_;
  bool is_armed_;
  std::string uri_;
  uint8_t controller_type_;
  uint8_t estimator_type_;
  bool enable_multiranger_;

  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

  /*  --  SENSORS --  */

  float initial_yaw_    = 0.0;
  bool has_initial_yaw_ = false;

  // Odometry
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_estimate_ptr_;
  double odom_buff_[10];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_ori_;
  std::shared_ptr<LogBlockGeneric> odom_logBlock_ori_;
  bool ori_rec_;

  std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_pos_;
  std::shared_ptr<LogBlockGeneric> odom_logBlock_pos_;
  bool pos_rec_;

  // IMU
  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  double imu_buff_[6];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_imu_;
  std::shared_ptr<LogBlockGeneric> imu_logBlock_;

  // Battery
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  unsigned char battery_buff_;

  /* std::unique_ptr<LogBlock<struct logBattery>> bat_logBlock_;
  std::function<void(uint32_t, struct logBattery *)> cb_bat_; */

  // Optitrack
  bool external_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_odom_sub_;
  std::string external_odom_topic_;

  // Multi-ranger deck
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> multi_ranger_sensor_ptr_;
  double range_buff_[6];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_range_;
  std::shared_ptr<LogBlockGeneric> range_logBlock_;
};

#endif
