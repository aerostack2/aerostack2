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
 * @file pixhawk_platform.cpp
 *
 * MavlinkPlatform class implementation
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 */

#include "as2_platform_mavlink/mavlink_platform.hpp"

void notImplemented()
{
  throw std::runtime_error("NOT IMPLEMENTED");
}


namespace as2_platform_mavlink
{

MavlinkPlatform::MavlinkPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  configureSensors();

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  this->declare_parameter<float>("max_thrust");
  max_thrust_ = this->get_parameter("max_thrust").as_double();

  this->declare_parameter<float>("min_thrust");
  min_thrust_ = this->get_parameter("min_thrust").as_double();

  this->declare_parameter<bool>("external_odom");
  external_odom_ = this->get_parameter("external_odom").as_bool();

  RCLCPP_INFO(this->get_logger(), "Max thrust: %f", max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Min thrust: %f", min_thrust_);
  RCLCPP_INFO(
    this->get_logger(), "Simulation mode: %s",
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");

  // declare mavlink_ subscribers

  mavlink_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "mavros/state", rclcpp::SensorDataQoS(),
    std::bind(&MavlinkPlatform::mavlinkStateCb, this, std::placeholders::_1));

  mavlink_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "mavros/local_position/odom", rclcpp::SensorDataQoS(),
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      msg->child_frame_id = base_link_frame_id_;
      msg->header.frame_id = odom_frame_id_;
      this->odometry_raw_estimation_ptr_->updateData(*msg);
    });

  // declare mavlink services

  mavlink_arm_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::CommandBool>>(
    "mavros/cmd/arming", this);
  mavlink_set_mode_client_ =
    std::make_shared<as2::SynchronousServiceClient<mavros_msgs::srv::SetMode>>(
    "mavros/set_mode", this);

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  if (external_odom_) {
    mavlink_vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/vision_pose/pose", 10);
    mavlink_vision_speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "mavros/vision_speed/speed_twist", 10);

    // In real flights, the odometry is published by the onboard computer.
    external_odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name(as2_names::topics::self_localization::twist),
      as2_names::topics::self_localization::qos,
      std::bind(&MavlinkPlatform::externalOdomCb, this, std::placeholders::_1));

    static auto px4_publish_vo_timer = this->create_wall_timer(
      std::chrono::milliseconds(10), [this]() {this->mavlink_publishVisualOdometry();});
  }

  // declare mavlink_ publishers

  mavlink_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "mavros/setpoint_position/local", 10);
  mavlink_acro_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "mavros/setpoint_raw/attitude", 10);
  mavlink_twist_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "mavros/setpoint_velocity/cmd_vel", 10);
}

void MavlinkPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  gps_sensor_ptr_ = std::make_unique<as2::sensors::GPS>("gps", this);

  odometry_raw_estimation_ptr_ =
    std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

bool MavlinkPlatform::ownSetArmingState(bool state)
{
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  auto response = std::make_shared<mavros_msgs::srv::CommandBool::Response>();
  request->value = state;

  auto result = mavlink_arm_client_->sendRequest(request, response);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to arm/disarm the vehicle");
    return false;
  }
  return response->success;
}

bool MavlinkPlatform::ownSetOffboardControl(bool offboard)
{
  // TODO(miferco97): CREATE A DEFAULT CONTROL MODE FOR BEING ABLE TO SWITCH TO OFFBOARD
  // MODE BEFORE RUNNING THE CONTROLLER

  if (offboard == false) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Turning into MANUAL Mode is not allowed from the onboard computer");
    return false;
  }

  RCLCPP_DEBUG(this->get_logger(), "Switching to OFFBOARD mode");
  auto out = mavlink_callOffboardControlMode();
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode");
    return false;
  }
  if (!getArmingState()) {
    this->ownSetArmingState(true);
  }
  return true;
}

bool MavlinkPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
        RCLCPP_INFO(this->get_logger(), "POSITION_MODE ENABLED");
      } break;
    case as2_msgs::msg::ControlMode::SPEED: {
        RCLCPP_INFO(this->get_logger(), "SPEED_MODE ENABLED");
      } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
        RCLCPP_INFO(this->get_logger(), "ATTITUDE_MODE ENABLED");
      } break;
    case as2_msgs::msg::ControlMode::ACRO: {
        RCLCPP_INFO(this->get_logger(), "ACRO_MODE ENABLED");
      } break;
    default: {
        RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
        has_mode_settled_ = false;
        return false;
      }
  }
  has_mode_settled_ = true;
  return true;
}

void MavlinkPlatform::sendCommand()
{
  if (!getArmingState() || !getOffboardMode() || !has_mode_settled_) {
    mavlink_publishRatesSetpoint(0.0, 0.0, 0.0, 1.0 * min_thrust_);
  }
  ownSendCommand();
  return;
}

bool MavlinkPlatform::ownSendCommand()
{
  as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();

  auto thrust_normalized = this->command_thrust_msg_.thrust / max_thrust_;
  thrust_normalized = std::clamp<float>(thrust_normalized, min_thrust_, 1.0);

  // Switch case to set setpoint
  switch (platform_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
        if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
          mavlink_publishPoseSetpoint(this->command_pose_msg_);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "NOT IMPLEMENTED FOR POSITION WITH YAW_RATE SETPOINT, sending  yaw 0.0");
          command_pose_msg_.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
          mavlink_publishPoseSetpoint(this->command_pose_msg_);
        }
      } break;
    case as2_msgs::msg::ControlMode::SPEED: {
        if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
          RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED FOR SPEED WITH YAW_ANGLE SETPOINT");
          this->command_twist_msg_.twist.angular.z = 0.0;
          mavlink_publishTwistSetpoint(this->command_twist_msg_);
        } else {
          mavlink_publishTwistSetpoint(this->command_twist_msg_);
        }
      } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
        mavlink_publishAttitudeSetpoint(
          this->command_pose_msg_.pose.orientation,
          thrust_normalized);
      } break;
    case as2_msgs::msg::ControlMode::ACRO: {
        mavlink_publishRatesSetpoint(
          this->command_twist_msg_.twist.angular.x,
          this->command_twist_msg_.twist.angular.y,
          this->command_twist_msg_.twist.angular.z,
          thrust_normalized);
      } break;
    default:
      return false;
  }
  return true;
}
void MavlinkPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(this->get_logger(), "KILL SWITCH TRIGGERED");
  // MAV_CMD_COMPONENT_ARM_DISARM
  auto out = mavlink_callVehicleCommand(400, 0.0, 21196.0);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Failed to kill the vehicle");
    return;
  }
}

void MavlinkPlatform::ownStopPlatform() {RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED");}

void MavlinkPlatform::externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  RCLCPP_DEBUG(this->get_logger(), "External odometry received");
  try {
    auto [pose_msg, twist_msg] = tf_handler_->getState(
      *_twist_msg, base_link_frame_id_,
      odom_frame_id_, base_link_frame_id_);

    mavlink_vision_speed_msg_.header.stamp = twist_msg.header.stamp;
    mavlink_vision_speed_msg_.header.frame_id = twist_msg.header.frame_id;   // BODY_FRAME_FLU
    mavlink_vision_speed_msg_.twist = twist_msg.twist;

    mavlink_vision_pose_msg_.header.stamp = pose_msg.header.stamp;
    mavlink_vision_pose_msg_.header.frame_id = pose_msg.header.frame_id;   // LOCAL_FRAME_FLU
    mavlink_vision_pose_msg_.pose = pose_msg.pose;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

/** -----------------------------------------------------------------*/
/** ------------------------- mavlink_ FUNCTIONS -------------------------*/
/** -----------------------------------------------------------------*/

// /**
//  * @brief Publish the offboard control mode.
//  *
//  */
bool MavlinkPlatform::mavlink_callOffboardControlMode()
{
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  auto response = std::make_shared<mavros_msgs::srv::SetMode::Response>();
  request->custom_mode = "OFFBOARD";
  auto result = mavlink_set_mode_client_->sendRequest(request, response);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode");
    return false;
  }
  return response->mode_sent;
}

void MavlinkPlatform::mavlink_publishRatesSetpoint(
  double droll, double dpitch, double dyaw,
  double dthrust)
{
  auto msg = mavros_msgs::msg::AttitudeTarget();
  msg.header.stamp = this->get_clock()->now();
  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
  msg.body_rate.x = droll;
  msg.body_rate.y = dpitch;
  msg.body_rate.z = dyaw;
  msg.thrust = dthrust;
  mavlink_acro_setpoint_pub_->publish(msg);
}

void MavlinkPlatform::mavlink_publishAttitudeSetpoint(
  const geometry_msgs::msg::Quaternion & q,
  double thrust)
{
  auto msg = mavros_msgs::msg::AttitudeTarget();
  msg.header.stamp = this->get_clock()->now();
  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
    mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  msg.orientation = q;
  msg.thrust = thrust;
  mavlink_acro_setpoint_pub_->publish(msg);
}


void MavlinkPlatform::mavlink_publishTwistSetpoint(const geometry_msgs::msg::TwistStamped & msg)
{
  mavlink_twist_setpoint_pub_->publish(msg);
}
void MavlinkPlatform::mavlink_publishPoseSetpoint(const geometry_msgs::msg::PoseStamped & msg)
{
  mavlink_pose_pub_->publish(msg);
}

}  // namespace as2_platform_mavlink
