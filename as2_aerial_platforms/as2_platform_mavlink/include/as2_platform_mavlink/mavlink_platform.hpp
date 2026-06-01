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
 * @file mavlink_platform.hpp
 *
 * PixhawkPlatform class definition
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 */

#ifndef AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
#define AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_

#include <Eigen/Dense>

#include <string>
#include <memory>
#include <cmath>


#include <mavros_msgs/msg/detail/attitude_target__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/thrust.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>


#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/synchronous_service_client.hpp"

namespace as2_platform_mavlink
{

class MavlinkPlatform : public as2::AerialPlatform
{
public:
  explicit MavlinkPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MavlinkPlatform() {}

public:
  void configureSensors() override;
  void publishSensorData();

  // TODO(miferco97): set ATTITUDE as default mode with yaw_speed = 0  and Thrust = 0 N
  void setDefaultControlMode() {}

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  void sendCommand() override;
  bool ownSendCommand() override;
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

  // mavlink_ subscribers

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavlink_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavlink_odom_sub_;

  void mavlinkStateCb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    this->platform_info_msg_.set__connected(msg->connected);
    this->platform_info_msg_.set__armed(msg->armed);
    if (msg->mode == "OFFBOARD") {
      this->platform_info_msg_.set__offboard(true);
    } else {
      this->platform_info_msg_.set__offboard(false);
    }
  }

  // mavlink clients
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandBool>::SharedPtr mavlink_arm_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::SetMode>::SharedPtr mavlink_set_mode_client_;
  as2::SynchronousServiceClient<mavros_msgs::srv::CommandLong>::SharedPtr
    mavlink_command_long_client_;


  // mavlink publishers

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavlink_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavlink_twist_setpoint_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr mavlink_acro_setpoint_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavlink_vision_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavlink_vision_speed_pub_;

  // mavlink_ Functions
  bool mavlink_callOffboardControlMode();

  // void mavlink_publishTrajectorySetpoint();
  void mavlink_publishAttitudeSetpoint(const geometry_msgs::msg::Quaternion & q, double thrust);
  void mavlink_publishRatesSetpoint(double droll, double dpitch, double dyaw, double thrust);
  void mavlink_publishTwistSetpoint(const geometry_msgs::msg::TwistStamped & msg);
  void mavlink_publishPoseSetpoint(const geometry_msgs::msg::PoseStamped & msg);
  bool mavlink_callVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    auto response = std::make_shared<mavros_msgs::srv::CommandLong::Response>();
    request->command = command;
    request->broadcast = false;
    request->confirmation = 0;
    request->param1 = param1;
    request->param2 = param2;
    auto out = mavlink_command_long_client_->sendRequest(request, response);
    if (!out || !response->success) {
      RCLCPP_ERROR(this->get_logger(), "Error calling vehicle command");
      return false;
    }
    return true;
  }
  void mavlink_publishVisualOdometry()
  {
    mavlink_vision_pose_pub_->publish(mavlink_vision_pose_msg_);
    mavlink_vision_speed_pub_->publish(mavlink_vision_speed_msg_);
  }

private:
  bool manual_from_operator_ = false;
  bool set_disarm_ = false;
  geometry_msgs::msg::PoseStamped mavlink_vision_pose_msg_;
  geometry_msgs::msg::TwistStamped mavlink_vision_speed_msg_;


  std::atomic<uint64_t> timestamp_;


  float max_thrust_;
  float min_thrust_;
  bool simulation_mode_ = false;
  bool external_odom_ = true;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
};

}  // namespace as2_platform_mavlink

#endif  // AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
