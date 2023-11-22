/*!*******************************************************************************************
 *  \file       crazyflie_platform.cpp
 *  \brief      Implements the functionality and communication with the crazyflie drone.
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
 *    and/or speedother materials provided with the distribution.
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
#include "crazyflie_platform.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include "as2_core/core_functions.hpp"

void CrazyfliePlatform::configureParams(const std::string &radio_uri) {
  if (radio_uri.empty()) {
    this->declare_parameter<std::string>("drone_URI", "radio://0/80/250K/E7E7E7E7E7");
    this->get_parameter("drone_URI", uri_);
  } else {
    uri_ = radio_uri;
  }
}

void CrazyfliePlatform::init() {
  base_frame_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_ = as2::tf::generateTfName(this, "odom");

  /*    PARAMETERS    */
  this->declare_parameter<bool>("multi_ranger_deck", false);  // Availability of multi-ranger deck
  this->get_parameter("multi_ranger_deck", enable_multiranger_);

  configureSensors();
  /*    SET-UP    */
  do {
    try {
      RCLCPP_DEBUG(this->get_logger(), "Connecting to: %s", uri_.c_str());
      cf_           = std::make_shared<Crazyflie>(uri_);
      is_connected_ = true;
    } catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Connection error: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
  } while (!is_connected_ && rclcpp::ok());
  RCLCPP_DEBUG(this->get_logger(), "Connected to: %s", uri_.c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  cf_->logReset();
  listVariables();

  /*    CONFIGURATION    */
  this->declare_parameter<uint8_t>("controller_type",
                                   1);  // Any(0), PID(1), Mellinger(2), INDI(3)
  this->get_parameter("controller_type", controller_type_);
  if (controller_type_ < 0 || controller_type_ > 3) controller_type_ = 1;
  cf_->setParamByName<uint8_t>("stabilizer", "controller", (uint8_t)(controller_type_));

  this->declare_parameter<uint8_t>("estimator_type", 2);  // Any(0), Complementary(1), EKF(2)
  this->get_parameter("estimator_type", estimator_type_);
  if (estimator_type_ < 0 || estimator_type_ > 2) estimator_type_ = 2;
  cf_->setParamByName<uint8_t>("stabilizer", "estimator", (uint8_t)(estimator_type_));  // EKF

  // TODO: SET_THIS_AS_A_PARAM
  cf_->setParamByName<float>("locSrv", "extQuatStdDev",
                             (float)(0.045));  // external parameter value
  // Odom
  ori_rec_ = pos_rec_ = false;
  // std::vector<std::string> vars_odom1 = {"kalman.q0","kalman.q1","kalman.q2","kalman.q3"};
  std::vector<std::string> vars_odom1 = {"stateEstimate.qx", "stateEstimate.qy", "stateEstimate.qz",
                                         "stateEstimate.qw"};
  cb_odom_ori_ = std::bind(&CrazyfliePlatform::onLogOdomOri, this, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3);
  odom_logBlock_ori_ =
      std::make_shared<LogBlockGeneric>(cf_.get(), vars_odom1, nullptr, cb_odom_ori_);
  odom_logBlock_ori_->start(2);

  // std::vector<std::string> vars_odom2 = {"kalman.stateX", "kalman.stateY", "kalman.stateZ",
  // "kalman.statePX", "kalman.statePY", "kalman.statePZ"};
  std::vector<std::string> vars_odom2 = {"stateEstimate.x",  "stateEstimate.y",
                                         "stateEstimate.z",  "stateEstimate.vx",
                                         "stateEstimate.vy", "stateEstimate.vz"};
  cb_odom_pos_ = std::bind(&CrazyfliePlatform::onLogOdomPos, this, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3);
  odom_logBlock_pos_ =
      std::make_shared<LogBlockGeneric>(cf_.get(), vars_odom2, nullptr, cb_odom_pos_);
  odom_logBlock_pos_->start(2);

  // IMU
  std::vector<std::string> vars_imu = {"gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z"};
  cb_imu_ = std::bind(&CrazyfliePlatform::onLogIMU, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3);

  imu_logBlock_ = std::make_shared<LogBlockGeneric>(cf_.get(), vars_imu, nullptr, cb_imu_);
  imu_logBlock_->start(10);

  // Multi-ranger deck
  if (enable_multiranger_) {
    std::vector<std::string> vars_range = {"range.back", "range.right", "range.front", "range.left",
                                           "range.up"};
    cb_range_ = std::bind(&CrazyfliePlatform::onLogRange, this, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3);

    range_logBlock_ = std::make_shared<LogBlockGeneric>(cf_.get(), vars_range, nullptr, cb_range_);
    range_logBlock_->start(10);
  }

  // External estimation
  this->declare_parameter<bool>("external_odom", false);
  this->get_parameter("external_odom", external_odom_);
  this->declare_parameter<std::string>("external_odom_topic", "external_odom");
  this->get_parameter("external_odom_topic", external_odom_topic_);

  // If using external localization, create the subscriber to it
  if (external_odom_) {
    RCLCPP_INFO(this->get_logger(), "External Localization: %s", external_odom_topic_.c_str());
    external_odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        external_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CrazyfliePlatform::externalOdomCB, this, std::placeholders::_1));
    RCLCPP_DEBUG(this->get_logger(), "Subscribed to external odom topic!");
  }

  /*  TIMERS */
  ping_timer_ = this->create_timer(std::chrono::milliseconds(10), [this]() { pingCB(); });
  bat_timer_  = this->create_timer(std::chrono::milliseconds(100), [this]() { onLogBattery(); });

  RCLCPP_INFO(this->get_logger(), "Finished Init");
}

CrazyfliePlatform::CrazyfliePlatform() : as2::AerialPlatform(), tf_handler_(this) {
  RCLCPP_INFO(this->get_logger(), "CrazyfliePlatform::CrazyfliePlatform");
  configureParams();
  init();
}

CrazyfliePlatform::CrazyfliePlatform(const std::string &ns, const std::string &radio_uri)
    : as2::AerialPlatform(ns), uri_(radio_uri), tf_handler_(this) {
  RCLCPP_INFO(get_logger(), "CrazyfliePlatform constructor with ns[%s] and uri[%s]", ns.c_str(),
              radio_uri.c_str());
  init();
};

void CrazyfliePlatform::onLogIMU(uint32_t time_in_ms,
                                 std::vector<double> *values,
                                 void * /*userData*/) {
  // Data is received as follows: {"gyro.x","gyro.y","gyro.z","acc.x","acc.y","acc.z"}
  // Acc. data is in g
  // Gyro. data is in º/s
  // The transformation array holds the unit transformations.
  int i                     = 0;
  double transformations[6] = {0.01745329252, 0.01745329252, 0.01745329252, 9.81, 9.81, 9.81};
  for (double v : *values) {
    imu_buff_[i] = double(v * transformations[i]);
    i++;
  }

  // Update IMU state
  sensor_msgs::msg::Imu imu_msg;
  auto timestamp                = this->get_clock()->now();
  imu_msg.header.stamp          = timestamp;
  imu_msg.header.frame_id       = "imu";
  imu_msg.linear_acceleration.x = imu_buff_[3];
  imu_msg.linear_acceleration.y = imu_buff_[4];
  imu_msg.linear_acceleration.z = imu_buff_[5];
  imu_msg.angular_velocity.x    = imu_buff_[0];
  imu_msg.angular_velocity.y    = imu_buff_[1];
  imu_msg.angular_velocity.z    = imu_buff_[2];

  imu_sensor_ptr_->updateData(imu_msg);
}

void CrazyfliePlatform::onLogRange(uint32_t time_in_ms,
                                   std::vector<double> *values,
                                   void * /*userData*/) {
  // Data: {"range.back", "range.right", "range.front", "range.left",  "range.up"}
  const double range_min = 0.002;
  const double range_max = 4.0;
  int i                  = 0;
  for (double v : *values) {
    range_buff_[i] = (double)v / 1000.0 > range_max ? std::numeric_limits<double>::infinity()
                                                    : (double)v / 1000;  // mm to m
    i++;
  }

  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.frame_id = base_frame_;
  scan_msg.header.stamp    = this->get_clock()->now();
  scan_msg.angle_min       = -M_PI;
  scan_msg.angle_max       = M_PI;
  scan_msg.angle_increment = M_PI / 2;
  scan_msg.range_min       = range_min;
  scan_msg.range_max       = range_max;
  scan_msg.ranges.resize(5);
  scan_msg.ranges[0] = range_buff_[0];
  scan_msg.ranges[1] = range_buff_[1];
  scan_msg.ranges[2] = range_buff_[2];
  scan_msg.ranges[3] = range_buff_[3];
  scan_msg.ranges[4] = range_buff_[0];  // closing scan
  // range_buff_[4] // UP, not used
  scan_msg.intensities.resize(5);
  scan_msg.intensities = {0.0, 0.0, 0.0, 0.0, 0.0};
  multi_ranger_sensor_ptr_->updateData(scan_msg);
}

void CrazyfliePlatform::onLogOdomOri(uint32_t time_in_ms,
                                     std::vector<double> *values,
                                     void * /*userData*/) {
  // Data is received as follows: {"kalman.q0","kalman.q1","kalman.q2","kalman.q3"};

  int i = 0;
  for (double v : *values) {
    odom_buff_[i] = (double)v;
    i++;
  }
  ori_rec_ = true;
  if (pos_rec_ && ori_rec_) updateOdom();
}

void CrazyfliePlatform::onLogOdomPos(uint32_t time_in_ms,
                                     std::vector<double> *values,
                                     void * /*userData*/) {
  // Data is received as follows:
  // {"kalman.stateX","kalman.stateY","kalman.stateZ","kalman.statePX","kalman.statePY","kalman.statePZ"}
  // Pos in m
  // Vel in m/s
  int i = 4;
  for (double v : *values) {
    odom_buff_[i] = (double)v;
    i++;
  }
  pos_rec_ = true;
  if (pos_rec_ && ori_rec_) updateOdom();
}

void CrazyfliePlatform::updateOdom() {
  pos_rec_ = ori_rec_ = false;

  // Send the odom message from the data received from the drone
  auto timestamp = this->get_clock()->now();

  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp    = timestamp;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id  = base_frame_;

  odom_msg.pose.pose.orientation.w = odom_buff_[3];
  odom_msg.pose.pose.orientation.x = odom_buff_[0];
  odom_msg.pose.pose.orientation.y = odom_buff_[1];
  odom_msg.pose.pose.orientation.z = odom_buff_[2];

  odom_msg.pose.pose.position.x = odom_buff_[4];
  odom_msg.pose.pose.position.y = odom_buff_[5];
  odom_msg.pose.pose.position.z = odom_buff_[6];

  odom_msg.twist.twist.linear.x = odom_buff_[7];
  odom_msg.twist.twist.linear.y = odom_buff_[8];
  odom_msg.twist.twist.linear.z = odom_buff_[9];

  odom_estimate_ptr_->updateData(odom_msg);
}

void CrazyfliePlatform::onLogBattery() {
  auto bat = cf_->vbat();
  // RCLCPP_INFO(this->get_logger(), "Battery: %f", bat);
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = bat / 4.2 * 100.0;  // 4.2V is the max voltage of the battery
  msg.voltage    = bat;
  battery_sensor_ptr_->updateData(msg);
}

void CrazyfliePlatform::configureSensors() {
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  odom_estimate_ptr_ =
      std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
  battery_sensor_ptr_ =
      std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>>("battery", this);
  if (enable_multiranger_) {
    // TODO: Create new laser scan sensor that gathers lidar/scan and lidar/points
    multi_ranger_sensor_ptr_ =
        std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::LaserScan>>("lidar/scan", this);
  }
}

bool CrazyfliePlatform::ownSendCommand() {
  // If the drones doesn't receive constantly commands it stops. Therefore setting the UNSET
  // mode or any that does not send any commands will make the drone to stop the propellers.

  as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();

  /* LIST OF ALL VARIABLES THAT CAN BE USED, REDUCE UNNECESSARY VARIABLES depending on the
   * control mode

  const double rollRate  = (this->command_twist_msg_.twist.angular.x / 3.1416 * 180.0);
  const double pitchRate = (this->command_twist_msg_.twist.angular.y / 3.1416 * 180.0);
  const double yawRate   = (this->command_twist_msg_.twist.angular.z / 3.1416 * 180.0);

  const double thrust = this->command_thrust_msg_.thrust;

  const double y = this->command_pose_msg_.pose.position.y;
  const double x = this->command_pose_msg_.pose.position.x;
  const double z = this->command_pose_msg_.pose.position.z;

  const double qx = this->command_pose_msg_.pose.orientation.x;
  const double qy = this->command_pose_msg_.pose.orientation.y;
  const double qz = this->command_pose_msg_.pose.orientation.z;
  const double qw = this->command_pose_msg_.pose.orientation.w;

  const auto eulerAngles = this->quaternion2Euler(this->command_pose_msg_.pose.orientation);
  const double roll      = (eulerAngles[0] / 3.1416 * 180.0);
  const double pitch     = (eulerAngles[1] / 3.1416 * 180.0);
  const double yaw       = (eulerAngles[2] / 3.1416 * 180.0); */
  // this->command_twist_msg_.header.frame_id = "earth";

  // bool out = tf_handler_.tryConvert(this->command_twist_msg_, odom_frame_);

  // if (!out) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not convert command to odom frame");
  //   return false;
  // }

  if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED &&
      platform_control_mode.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME &&
      this->getArmingState() && is_connected_) {
    switch (platform_control_mode.control_mode) {
      case as2_msgs::msg::ControlMode::SPEED: {
        const double vx      = this->command_twist_msg_.twist.linear.x;
        const double vy      = this->command_twist_msg_.twist.linear.y;
        const double vz      = this->command_twist_msg_.twist.linear.z;
        const double yawRate = (this->command_twist_msg_.twist.angular.z / 3.1416 * 180.0);
        cf_->sendVelocityWorldSetpoint(vx, vy, vz, -yawRate);
      } break;

      case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
        const double vx      = this->command_twist_msg_.twist.linear.x;
        const double vy      = this->command_twist_msg_.twist.linear.y;
        const double z       = this->command_pose_msg_.pose.position.z;
        const double yawRate = (this->command_twist_msg_.twist.angular.z / 3.1416 * 180.0);
        cf_->sendHoverSetpoint(vx, vy, yawRate, z);
        RCLCPP_DEBUG(this->get_logger(), "Hover set to z: %f", z);
      } break;

      default:
        static rclcpp::Clock clock;
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2, "Command/Control Mode not supported");
        return false;
    }
  } else if (platform_control_mode.control_mode == as2_msgs::msg::ControlMode::POSITION &&
             platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    const double y         = this->command_pose_msg_.pose.position.y;
    const double x         = this->command_pose_msg_.pose.position.x;
    const double z         = this->command_pose_msg_.pose.position.z;
    const auto eulerAngles = this->quaternion2Euler(this->command_pose_msg_.pose.orientation);
    const double yaw       = (eulerAngles[2] / 3.1416 * 180.0);
    cf_->sendPositionSetpoint(x, y, z, yaw);
  } else if (platform_control_mode.control_mode == as2_msgs::msg::ControlMode::UNSET) {
    cf_->sendStop();  // Not really needed, will stop anyway if no command is set.

    // TODO: ATTITUDE Mode
    /* } else if (platform_control_mode.control_mode == as2_msgs::msg::ControlMode::ATTITUDE)
     {
     // Compute the thrust from the thrust message from N to PWM between 0 and 65535
      cf_->sendSetpoint(roll, pitch, yawRate, thrust); }*/
  } else {
    static rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2, "Command/Control Mode not supported");
    return false;
  }
  return true;
}

bool CrazyfliePlatform::ownSetArmingState(bool state) {
  // Crazyflie does not have arming. Unarming will be used to stop the motors.
  if (!state) {
    RCLCPP_WARN(this->get_logger(), "STOP");
    cf_->sendStop();
  }
  is_armed_ = state;
  return is_connected_;
}

bool CrazyfliePlatform::ownSetOffboardControl(bool offboard) { return is_connected_; }

bool CrazyfliePlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) {
  // Only the yaw speed modes implemented with ENU reference.
  if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED &&
      msg.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME) {
    switch (msg.control_mode) {
      case as2_msgs::msg::ControlMode::SPEED:

        RCLCPP_DEBUG(this->get_logger(), "SPEED ENABLED");
        break;

      case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:

        RCLCPP_DEBUG(this->get_logger(), "SPEED_IN_A_PLANE ENABLED");
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
        return false;
    }
    return true;
  } else if (msg.control_mode == as2_msgs::msg::ControlMode::POSITION &&
             msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE)
    return true;
  /* TODO: Add attitude mode. Thrust is uint16_t and we need to know how it goes.
  else if (msg.control_mode == as2_msgs::msg::ControlMode::ATTITUDE)
    return false;
  */
  else if (msg.control_mode == as2_msgs::msg::ControlMode::UNSET)
    return true;
  else
    return false;
}

void CrazyfliePlatform::listVariables() {
  cf_->requestLogToc(false);
  std::for_each(cf_->logVariablesBegin(), cf_->logVariablesEnd(),
                [this](const Crazyflie::LogTocEntry &entry) {
                  std::ostringstream output_stream;
                  output_stream << entry.group << "." << entry.name << " (";
                  switch (entry.type) {
                    case Crazyflie::LogTypeUint8:
                      output_stream << "uint8";
                      break;
                    case Crazyflie::LogTypeInt8:
                      output_stream << "int8";
                      break;
                    case Crazyflie::LogTypeUint16:
                      output_stream << "uint16";
                      break;
                    case Crazyflie::LogTypeInt16:
                      output_stream << "int16";
                      break;
                    case Crazyflie::LogTypeUint32:
                      output_stream << "uint32";
                      break;
                    case Crazyflie::LogTypeInt32:
                      output_stream << "int32";
                      break;
                    case Crazyflie::LogTypeFloat:
                      output_stream << "double";
                      break;
                    case Crazyflie::LogTypeFP16:
                      output_stream << "fp16";
                      break;
                  }
                  output_stream << ")";
                  output_stream << std::endl;

                  RCLCPP_DEBUG(this->get_logger(), "%s", std::string{output_stream.str()}.c_str());
                });
}

void CrazyfliePlatform::pingCB() {
  try {
    cf_->sendPing();
    if (!is_connected_) {
      is_connected_ = true;
      RCLCPP_INFO(this->get_logger(), "Connection restored!");
    }
  } catch (std::exception &e) {
    static rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2, "Connection error: %s", e.what());
    is_connected_ = false;
  }
}

void CrazyfliePlatform::externalOdomCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  try {
    // pose obtained in odom frame to avoid yaw issues
    // auto pose = tf_handler.getPoseStamped(odom_frame, base_frame);
    auto pose = tf_handler_.getPoseStamped(odom_frame_, base_frame_);

    // RCLCPP_INFO(get_logger(), " %f, %f, %f ", pose.pose.position.x, pose.pose.position.y,
    //             pose.pose.position.z);
    cf_->sendExternalPoseUpdate(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                                pose.pose.orientation.x, pose.pose.orientation.y,
                                pose.pose.orientation.z, pose.pose.orientation.w);
    // cf_->sendExternalPositionUpdate(msg->pose.position.x, msg->pose.position.y,
    //                                 msg->pose.position.z);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform external odom: %s", ex.what());
  }
}

Eigen::Vector3d CrazyfliePlatform::quaternion2Euler(geometry_msgs::msg::Quaternion quat) {
  Eigen::Quaterniond quaternion;
  quaternion.x()        = quat.x;
  quaternion.y()        = quat.y;
  quaternion.z()        = quat.z;
  quaternion.w()        = quat.w;
  Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

  return euler;
}
