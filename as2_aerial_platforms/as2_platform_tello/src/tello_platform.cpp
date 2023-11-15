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

#include "tello_platform.hpp"
#include <as2_core/utils/tf_utils.hpp>

TelloPlatform::TelloPlatform() : as2::AerialPlatform() {
  this->tello      = std::make_unique<Tello>();
  this->connected_ = this->tello->connect();

  this->declare_parameter<double>("minSpeed", 0.02);
  this->declare_parameter<double>("maxSpeed", 15.0);
  this->get_parameter("minSpeed", min_speed_);
  this->get_parameter("maxSpeed", max_speed_);
  configureSensors();

  this->declare_parameter<double>("camera_freq", 30.0);
  this->get_parameter("camera_freq", camera_freq_);
  this->declare_parameter<double>("sensor_freq");
  this->get_parameter("sensor_freq", sensor_freq_);

  odom_frame_id_      = as2::tf::generateTfName(this, "odom");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  resetOdometry();

  this->timer_ = this->create_timer(std::chrono::duration<double>(1.0f / sensor_freq_), [this]() {
    recvIMU();
    recvBattery();
    recvBarometer();
    // recvOdometry();
  });

  static auto odom_timer = this->create_timer(std::chrono::duration<double>(1.0f / 200.0f),
                                              [this]() { recvOdometry(); });

  static auto time_ping_timer = this->create_timer(std::chrono::duration<double>(5.0f), [this]() {
    std::string response;
    tello->sendCommand("time?", true, &response);
    const auto& clock = this->get_clock();
    // cast response to int
    const auto& time = std::stoi(response);
    if (time > 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 15000, "Flight Time: %s", response.c_str());
    }
  });

  this->cam_timer_ = this->create_timer(std::chrono::duration<double>(1.0f / camera_freq_),
                                        [this]() { recvVideo(); });
}

TelloPlatform::~TelloPlatform() {}

// *********************************************************
// ***************** Aerial Platform Methods ***************
// *********************************************************

void TelloPlatform::configureSensors() {
  imu_sensor_ptr_ = std::make_shared<as2::sensors::Imu>("imu", this);
  battery_ptr_    = std::make_shared<as2::sensors::Battery>("battery", this);
  barometer_ptr_  = std::make_shared<as2::sensors::Barometer>("barometer", this);
  odometry_ptr_   = std::make_shared<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
  camera_ptr_     = std::make_shared<as2::sensors::Camera>("camera", this);

  sensor_msgs::msg::CameraInfo cam_info;  // TODO: fill camera info
  camera_ptr_->setParameters(cam_info, "bgr8", "pinhole");
}

bool TelloPlatform::ownSendCommand() {
  if (as2::control_mode::isHoverMode(this->getControlMode())) {
    bool speed_send = tello->speedMotion(0, 0, 0, 0);

    if (!speed_send) {
      RCLCPP_ERROR(this->get_logger(), "Error sending Hover control command");
      return false;
    }
    return true;
  }

  const uint8_t pose_control_mode = as2::control_mode::convertToUint8t(
      as2_msgs::msg::ControlMode::POSITION, as2_msgs::msg::ControlMode::YAW_ANGLE,
      as2_msgs::msg::ControlMode::BODY_FLU_FRAME);

  const uint8_t speed_control_mode = as2::control_mode::convertToUint8t(
      as2_msgs::msg::ControlMode::SPEED, as2_msgs::msg::ControlMode::YAW_SPEED,
      as2_msgs::msg::ControlMode::BODY_FLU_FRAME);

  const uint8_t speed_plane_control_mode = as2::control_mode::convertToUint8t(
      as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE, as2_msgs::msg::ControlMode::YAW_SPEED,
      as2_msgs::msg::ControlMode::BODY_FLU_FRAME);

  switch (as2::control_mode::convertToUint8t(this->getControlMode())) {
    case pose_control_mode: {
      RCLCPP_ERROR(this->get_logger(), "Position control not implemented");
      return false;  // TODO THIS is temporary
      double x_m, y_m, z_m, yaw_rad;
      x_m     = this->command_pose_msg_.pose.position.x;     // m
      y_m     = this->command_pose_msg_.pose.position.y;     // m
      z_m     = this->command_pose_msg_.pose.position.z;     // m
      yaw_rad = this->command_pose_msg_.pose.orientation.z;  // rad

      // std::vector<double> new_ref = {x_m, y_m, z_m, yaw_rad}; //FIXME: find a better way to do
      // this if (reference_point_ != new_ref) {
      double x   = std::clamp(x_m, min_linear_pose_, max_linear_pose_);  // cm
      double y   = std::clamp(y_m, min_linear_pose_, max_linear_pose_);  // cm
      double z   = std::clamp(z_m, min_linear_pose_, max_linear_pose_);  // cm
      double yaw = normalizeDegrees(yaw_rad * 180 / M_PI);               // degrees

      if (!tello->x_motion(x)) {
        RCLCPP_ERROR(this->get_logger(), "Sending X position command failed.");
        return false;
      }
      if (!tello->y_motion(y)) {
        RCLCPP_ERROR(this->get_logger(), "Sending Y position command failed.");
        return false;
      }
      if (!tello->z_motion(z)) {
        RCLCPP_ERROR(this->get_logger(), "Sending Z position failed.");
        return false;
      }
      if (!tello->yaw_twist(yaw)) {
        RCLCPP_ERROR(this->get_logger(), "Sending Yaw orientation failed.");
        return false;
        // }
        // reference_point_ = new_ref;
      }
    } break;
    case speed_control_mode: {
      // RCLCPP_INFO(this->get_logger(), "Speed control");
      double vx   = this->command_twist_msg_.twist.linear.x;   // cm/s
      double vy   = this->command_twist_msg_.twist.linear.y;   // cm/s
      double vz   = this->command_twist_msg_.twist.linear.z;   // cm/s
      double vyaw = this->command_twist_msg_.twist.angular.z;  // degrees/s

      // RCLCPP_INFO(this->get_logger(), "Speed control %f %f %f %f", vx, vy, vz, vyaw);
      bool speed_send = tello->speedMotion(vx, vy, vz, vyaw);

      if (!speed_send) {
        RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending control speed command");
        return false;
      }
    } break;
    case speed_plane_control_mode: {
      double z_m = this->command_pose_msg_.pose.position.z;  // m
      double z   = z_m - tello->getHeight();                 // m

      double vx   = this->command_twist_msg_.twist.linear.x;   // m/s
      double vy   = this->command_twist_msg_.twist.linear.y;   // m/s
      double vyaw = this->command_twist_msg_.twist.angular.z;  // degrees/s

      if (!tello->z_motion(z)) {
        RCLCPP_ERROR(this->get_logger(), "Sending Z position failed.");
      }
      if (!tello->speedMotion(vx, vy, 0, vyaw)) {
        RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending control speed command");
        return false;
      }
      break;
    };
  }
  return true;
}

bool TelloPlatform::ownSetArmingState(bool state) {
  bool resp = state;
  if (state && !this->connected_) {
    resp = tello->sendCommand("command");
    return resp;
  }
  return resp;
}

bool TelloPlatform::ownSetOffboardControl(bool offboard) {
  RCLCPP_DEBUG(this->get_logger(), "Offboard status changed to %d", offboard);
  return this->connected_;
}

bool TelloPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) {
  RCLCPP_DEBUG(this->get_logger(), "New control mode: %s",
               as2::control_mode::controlModeToString(msg).c_str());
  this->control_mode_in_ = msg;
  return true;
}

bool TelloPlatform::ownTakeoff() {
  if (!this->connected_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Tello Platform: Error sending takeoff command, not connected");
    return false;
  }

  for (int i = 0; i < 3; i++) {
    if (tello->sendCommand("takeoff")) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500ms before retry
  }

  return false;
}

bool TelloPlatform::ownLand() {
  if (!this->connected_) {
    RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending land command, not connected");
    return false;
  }

  for (int i = 0; i < 3; i++) {
    if (tello->sendCommand("land")) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500ms before retry
  }
  RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending land command");
  return false;
}

// **********************************************************
// ******************** CALLBACK METHODS ********************
// **********************************************************
void TelloPlatform::recvIMU() {
  std::array<coordinates, 3> imu_info = tello->getIMU();

  tf2::Quaternion q;
  float roll_rad  = imu_info[0].x;
  float pitch_rad = imu_info[0].y;
  float yaw_rad   = imu_info[0].z;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp              = this->get_clock()->now();
  static const std::string frame_id = as2::tf::generateTfName(this, "imu");
  imu_msg.header.frame_id           = frame_id;
  imu_msg.orientation.x             = q.x();
  imu_msg.orientation.y             = q.y();
  imu_msg.orientation.z             = q.z();
  imu_msg.orientation.w             = q.w();
  imu_msg.angular_velocity.x        = 0;
  imu_msg.angular_velocity.y        = 0;
  imu_msg.angular_velocity.z        = 0;
  imu_msg.linear_acceleration.x     = imu_info[2].x;
  imu_msg.linear_acceleration.y     = imu_info[2].y;
  imu_msg.linear_acceleration.z     = imu_info[2].z;

  imu_sensor_ptr_->updateData(imu_msg);
}

void TelloPlatform::recvBattery() {
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.percentage   = tello->getBattery();

  battery_ptr_->updateData(battery_msg);
}

void TelloPlatform::recvBarometer() {
  sensor_msgs::msg::FluidPressure barometer_msg;
  barometer_msg.header.stamp   = this->get_clock()->now();
  barometer_msg.fluid_pressure = tello->getBarometer();
  barometer_ptr_->updateData(barometer_msg);
}

void integrate_speed(double vx, double vy, double& x, double& y, double dt) {
  x += vx * dt;
  y += vy * dt;

  /* RCLCPP_WARN(rclcpp::get_logger("speed_debugger"), "REMOVE THIS : vx: %f, vy: %f,dt %f", vx, vy,
              dt); */
}

void TelloPlatform::resetOdometry() {
  odom_msg_                      = nav_msgs::msg::Odometry();
  odom_msg_.header.stamp         = this->get_clock()->now();
  odom_msg_.pose.pose.position.x = 0;
  odom_msg_.pose.pose.position.y = 0;
}

void TelloPlatform::recvOdometry() {
  const auto now         = this->now();
  double dt              = (now - odom_msg_.header.stamp).seconds();
  odom_msg_.header.stamp = now;

  odom_msg_.header.frame_id = odom_frame_id_;
  odom_msg_.child_frame_id  = base_link_frame_id_;

  odom_msg_.pose.pose.position.z = tello->getHeight();
  auto rpy                       = tello->getOrientation();
  float roll_rad                 = rpy.x;
  float pitch_rad                = rpy.y;
  float yaw_rad                  = rpy.z;

  // std::cout << "roll: " << roll_rad << " pitch: " << pitch_rad << " yaw: " << yaw_rad <<
  // std::endl;
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  odom_msg_.pose.pose.orientation.w = q.w();
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  auto twist                        = tello->getVelocity();
  odom_msg_.twist.twist.linear.x    = twist.x;
  odom_msg_.twist.twist.linear.y    = twist.y;
  odom_msg_.twist.twist.linear.z    = twist.z;

  integrate_speed(odom_msg_.twist.twist.linear.x, odom_msg_.twist.twist.linear.y,
                  odom_msg_.pose.pose.position.x, odom_msg_.pose.pose.position.y, dt);

  odometry_ptr_->updateData(odom_msg_);
}

void TelloPlatform::recvVideo() {
  cv::Mat frame = tello->getFrame();
  camera_ptr_->updateData(frame);
}

// **********************************************************
// ******************** AUXILIAR METHODS ********************
// **********************************************************
/**
 * @brief -100 .. 100
 *
 * @param value
 * @param min_value
 * @param max_value
 * @return double
 */
double TelloPlatform::normalize(double value, double min_value, double max_value) {
  value = 2 * (value - min_value) / (max_value - min_value) - 1;
  return 100 * value;
}

/**
 * @brief 0 .. 359
 *
 * @param value
 * @return double
 */
double TelloPlatform::normalizeDegrees(double value) {
  // FIXME: what happens if value is negative?
  while (abs(value) > 360) {
    value = abs(value) - 360;
  }
  return value;
}
