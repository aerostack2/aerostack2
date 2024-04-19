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

/*!*******************************************************************************************
 *  \file       as2_platform_dji_psdk.cpp
 *  \brief      DJI PSDK platform implementation
 *  \authors    Rafael Pérez Seguí
 *              Santiago Tapia Fernandez
 ********************************************************************************/

#include "as2_platform_dji_psdk.hpp"

namespace as2_platform_dji_psdk
{

DJIMatricePSDKPlatform::DJIMatricePSDKPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options),
  tf_handler_(this)
{
  // Create publishers
  velocity_command_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
    "psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate", 10);

  gimbal_rotation_pub_ =
    this->create_publisher<psdk_interfaces::msg::GimbalRotation>("psdk_ros2/gimbal_rotation", 10);

  gimbal_attitude_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "sensor_measurements/gimbal/attitude", 10);

  // Create subscriptions
  position_fused_sub_ = this->create_subscription<psdk_interfaces::msg::PositionFused>(
    "psdk_ros2/position_fused", 10,
    std::bind(&DJIMatricePSDKPlatform::positionFusedCallback, this, std::placeholders::_1));

  attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "psdk_ros2/attitude", 10,
    std::bind(&DJIMatricePSDKPlatform::attitudeCallback, this, std::placeholders::_1));

  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "psdk_ros2/velocity_ground_fused", 10,
    std::bind(&DJIMatricePSDKPlatform::velocityCallback, this, std::placeholders::_1));

  gimbal_control_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
    "platform/gimbal/gimbal_command", 10,
    std::bind(&DJIMatricePSDKPlatform::gimbalControlCallback, this, std::placeholders::_1));

  // Create services
  turn_on_motors_srv_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/turn_on_motors", this);

  turn_off_motors_srv_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/turn_off_motors", this);

  takeoff_srv_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/takeoff", this);

  land_srv_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>("psdk_ros2/land", this);

  set_local_position_srv_ = std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/set_local_position_ref", this);

  obtain_ctrl_authority_srv_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/obtain_ctrl_authority", this);

  release_ctrl_authority_srv_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    "psdk_ros2/release_ctrl_authority", this);

  camera_setup_streaming_srv_ =
    std::make_shared<as2::SynchronousServiceClient<psdk_interfaces::srv::CameraSetupStreaming>>(
    "psdk_ros2/camera_setup_streaming", this);

  gimbal_reset_srv_ =
    std::make_shared<as2::SynchronousServiceClient<psdk_interfaces::srv::GimbalReset>>(
    "psdk_ros2/gimbal_reset", this);

  RCLCPP_INFO(this->get_logger(), "DJIMatricePSDKPlatform initialized");
}

void DJIMatricePSDKPlatform::configureSensors()
{
  RCLCPP_INFO(this->get_logger(), "DJIMatricePSDKPlatform sensors configuring");
  // Create sensors
  sensor_odom_ptr_ = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
  RCLCPP_INFO(this->get_logger(), "DJIMatricePSDKPlatform odometry sensor configured");

  // Read tf_timeout_threshold
  double tf_timeout_threshold;
  this->declare_parameter<double>("tf_timeout_threshold", 0.5);
  this->get_parameter("tf_timeout_threshold", tf_timeout_threshold);
  tf_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(tf_timeout_threshold));
  RCLCPP_INFO(
    this->get_logger(), "DJIMatricePSDKPlatform tf_timeout_threshold: %f", tf_timeout_threshold);

  // Initialize the camera streaming service
  this->declare_parameter<bool>("enable_camera", false);
  bool enable_camera;
  this->get_parameter("enable_camera", enable_camera);
  RCLCPP_INFO(
    this->get_logger(), "Camera streaming service: %s", enable_camera ? "enabled" : "disabled");
  if (enable_camera) {
    auto request = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Request>();
    request->payload_index = 1;
    request->camera_source = 0;
    request->start_stop = 1;
    RCLCPP_INFO(this->get_logger(), "Starting camera streaming");
    auto response = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Response>();
    bool result = camera_setup_streaming_srv_->sendRequest(request, response);
    bool success = result && response->success;
    if (!success) {
      RCLCPP_INFO(this->get_logger(), "Could not start camera streaming");
    }
  }

  // Initialize the gimbal service
  this->declare_parameter<bool>("enable_gimbal", false);
  this->get_parameter("enable_gimbal", enable_gimbal_);
  RCLCPP_INFO(
    this->get_logger(), "Gimbal reset service: %s", enable_gimbal_ ? "enabled" : "disabled");
  if (enable_gimbal_) {
    auto request = std::make_shared<psdk_interfaces::srv::GimbalReset::Request>();
    request->payload_index = 1;
    request->reset_mode = 1;
    auto response = std::make_shared<psdk_interfaces::srv::GimbalReset::Response>();
    RCLCPP_INFO(this->get_logger(), "Resetting gimbal");
    bool result = gimbal_reset_srv_->sendRequest(request, response);
    bool success = result && response->success;
    if (!success) {
      RCLCPP_INFO(this->get_logger(), "Could not reset gimbal");
      enable_gimbal_ = false;
    } else {
      this->declare_parameter<std::string>("gimbal_base_frame_id", "gimbal_base");
      this->get_parameter("gimbal_base_frame_id", gimbal_base_frame_id_);
      gimbal_base_frame_id_ = as2::tf::generateTfName(this->get_namespace(), gimbal_base_frame_id_);
    }
  }
  last_gimbal_command_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "DJIMatricePSDKPlatform sensors configured");
}

bool DJIMatricePSDKPlatform::ownSetArmingState(bool state)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  bool result = set_local_position_srv_->sendRequest(request, response);
  bool success = result && response->success;
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Could not set local position reference due to '%s'",
      response->message.data());
  }
  return success;
}

bool DJIMatricePSDKPlatform::ownSetOffboardControl(bool offboard)
{
  if (!offboard) {
    // Release control authority
    return setControlAuthority(false);
  }
  return true;
}

bool DJIMatricePSDKPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting control mode to %d", msg.control_mode);
  bool success = false;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
      {
        // Release control authority
        RCLCPP_INFO(this->get_logger(), "UNSET MODE: Releasing control authority");
        success = setControlAuthority(false);
        break;
      }
    case as2_msgs::msg::ControlMode::HOVER:
    case as2_msgs::msg::ControlMode::SPEED:
      {
        // Obtain control authority
        RCLCPP_INFO(this->get_logger(), "HOVER || SPEED MODE: Obtain control authority");
        success = setControlAuthority(true);
        break;
      }
    default:
      RCLCPP_ERROR(this->get_logger(), "Control mode not supported");
      break;
  }
  return success;
}

bool DJIMatricePSDKPlatform::ownSendCommand()
{
  sensor_msgs::msg::Joy velocity_command;
  velocity_command.axes.resize(4);

  as2_msgs::msg::ControlMode current_control_mode;
  current_control_mode = getControlMode();

  switch (current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
      {
        return true;
      }
    case as2_msgs::msg::ControlMode::HOVER:
      {
        velocity_command.axes[0] = 0.0;  //
        velocity_command.axes[1] = 0.0;
        velocity_command.axes[2] = 0.0;
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        velocity_command.axes[0] = command_twist_msg_.twist.linear.x;  // X velocity (m/s)
        velocity_command.axes[1] = command_twist_msg_.twist.linear.y;  // Y velocity (m/s)
        velocity_command.axes[2] = command_twist_msg_.twist.linear.z;  // Z velocity (m/s)
        break;
      }
    default:
      RCLCPP_ERROR(this->get_logger(), "Control mode not supported");
      return false;
  }

  switch (current_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      {
        velocity_command.axes[3] = command_twist_msg_.twist.angular.z;  // Yaw rate (rad/s)
        break;
      }
    default:
      RCLCPP_ERROR(this->get_logger(), "Yaw mode not supported");
      return false;
  }

  velocity_command_pub_->publish(velocity_command);
  return true;
}

void DJIMatricePSDKPlatform::ownStopPlatform()
{
  // Send hover to platform here
  setControlAuthority(false);
}

void DJIMatricePSDKPlatform::ownKillSwitch()
{
  // Send kill switch to platform here
  setControlAuthority(false);
}

bool DJIMatricePSDKPlatform::ownTakeoff()
{
  // Send takeoff to platform here
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  RCLCPP_INFO(this->get_logger(), "Sending takeoff");
  bool result = takeoff_srv_->sendRequest(request, response);
  bool success = result && response->success;
  if (!success) {
    RCLCPP_INFO(this->get_logger(), "Could not takeoff due to '%s'", response->message.data());
  }
  return success;
}

bool DJIMatricePSDKPlatform::ownLand()
{
  // Send land to platform here
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  RCLCPP_INFO(this->get_logger(), "Sending land");
  bool result = land_srv_->sendRequest(request, response);
  bool success = result && response->success;
  if (!success) {
    RCLCPP_INFO(this->get_logger(), "Could not land due to '%s'", response->message.data());
  }
  return success;
}

void DJIMatricePSDKPlatform::positionFusedCallback(
  const psdk_interfaces::msg::PositionFused::SharedPtr msg)
{
  // Update the odometry sensor
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = as2::tf::generateTfName(this->get_namespace(), "odom");
  odom_msg.child_frame_id = as2::tf::generateTfName(this->get_namespace(), "base_link");
  odom_msg.pose.pose.position.x = msg->position.x;
  odom_msg.pose.pose.position.y = msg->position.y;
  odom_msg.pose.pose.position.z = msg->position.z;
  odom_msg.pose.pose.orientation.x = current_attitude_.x;
  odom_msg.pose.pose.orientation.y = current_attitude_.y;
  odom_msg.pose.pose.orientation.z = current_attitude_.z;
  odom_msg.pose.pose.orientation.w = current_attitude_.w;

  // convert ENU to FLU
  Eigen::Vector3d vel_ENU = Eigen::Vector3d(
    current_lineal_velocity_.x, current_lineal_velocity_.y, current_lineal_velocity_.z);
  auto flu_speed = as2::frame::transform(odom_msg.pose.pose.orientation, vel_ENU);
  odom_msg.twist.twist.linear.x = flu_speed.x();
  odom_msg.twist.twist.linear.y = flu_speed.y();
  odom_msg.twist.twist.linear.z = flu_speed.z();

  odom_msg.twist.twist.angular.x = current_angular_velocity_.x;
  odom_msg.twist.twist.angular.y = current_angular_velocity_.y;
  odom_msg.twist.twist.angular.z = current_angular_velocity_.z;

  sensor_odom_ptr_->updateData(odom_msg);
}

void DJIMatricePSDKPlatform::attitudeCallback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  current_attitude_ = msg->quaternion;
}

void DJIMatricePSDKPlatform::velocityCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_lineal_velocity_ = msg->vector;
}

void DJIMatricePSDKPlatform::angularVelocityCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  current_angular_velocity_ = msg->vector;
}

void DJIMatricePSDKPlatform::gimbalControlCallback(
  const as2_msgs::msg::GimbalControl::SharedPtr msg)
{
  if (!enable_gimbal_) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Gimbal control not enabled");
    return;
  }

  double time_diff = (this->now() - last_gimbal_command_time_).seconds();
  if (time_diff < GIMBAL_COMMAND_TIME) {
    // Wait for the gimbal to reach the desired position
    return;
  }

  psdk_interfaces::msg::GimbalRotation gimbal_command_msg;
  gimbal_command_msg.payload_index = 1;  // Main payload

  switch (msg->control_mode) {
    case as2_msgs::msg::GimbalControl::POSITION_MODE:
      gimbal_command_msg.rotation_mode = 1;  // Absolute
      break;
    case as2_msgs::msg::GimbalControl::SPEED_MODE:
      gimbal_command_msg.rotation_mode = 2;  // Speed
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Gimbal control mode not supported");
      break;
  }

  gimbal_command_msg.time = GIMBAL_COMMAND_TIME;  // In seconds, expected time to target rotation

  // Desired roll, pitch and yaw in msg frame
  geometry_msgs::msg::QuaternionStamped desired_base_link_orientation;
  desired_base_link_orientation.header.stamp = msg->target.header.stamp;
  desired_base_link_orientation.header.frame_id = msg->target.header.frame_id;
  as2::frame::eulerToQuaternion(
    msg->target.vector.x, msg->target.vector.y, msg->target.vector.z,
    desired_base_link_orientation.quaternion);

  // Transform desired orientation to earth frame
  geometry_msgs::msg::QuaternionStamped desired_earth_orientation;
  desired_earth_orientation = desired_base_link_orientation;

  std::string target_frame = "earth";  // Earth frame
  if (!tf_handler_.tryConvert(desired_earth_orientation, target_frame, tf_timeout_)) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not transform desired gimbal orientation to earth frame");
    return;
  }
  double desired_roll_earth, desired_pitch_earth, desired_yaw_earth;
  as2::frame::quaternionToEuler(
    desired_earth_orientation.quaternion, desired_roll_earth, desired_pitch_earth,
    desired_yaw_earth);

  gimbal_command_msg.roll = desired_roll_earth;
  gimbal_command_msg.pitch = desired_pitch_earth;
  gimbal_command_msg.yaw = desired_yaw_earth;

  // Check if gimbal_command_msg_ is different from the last command
  if (
    gimbal_command_msg_.rotation_mode == gimbal_command_msg.rotation_mode &&
    gimbal_command_msg_.roll == gimbal_command_msg.roll &&
    gimbal_command_msg_.pitch == gimbal_command_msg.pitch &&
    gimbal_command_msg_.yaw == gimbal_command_msg.yaw)
  {
    return;
  }
  gimbal_command_msg_ = gimbal_command_msg;
  gimbal_rotation_pub_->publish(gimbal_command_msg_);
  last_gimbal_command_time_ = this->now();
}

bool DJIMatricePSDKPlatform::setControlAuthority(bool state)
{
  // Avoid sending the same command
  if (ctl_authority_ == state) {
    return true;
  }

  bool success = false;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();

  if (state) {
    // Obtain control authority
    RCLCPP_INFO(this->get_logger(), "Sending obtain control authority");
    bool result = obtain_ctrl_authority_srv_->sendRequest(request, response);
    success = result && response->success;
    if (!success) {
      RCLCPP_INFO(
        this->get_logger(), "Could not obtain control authority due to '%s'",
        response->message.data());
      return success;
    }
  } else {
    // Release control authority
    RCLCPP_INFO(this->get_logger(), "Sending release control authority");
    bool result = release_ctrl_authority_srv_->sendRequest(request, response);
    success = result && response->success;
    if (!success) {
      RCLCPP_INFO(
        this->get_logger(), "Could not release control authority due to '%s'",
        response->message.data());
    }
  }
  if (success) {
    ctl_authority_ = state;
  }
  return success;
}

}  // namespace as2_platform_dji_psdk
