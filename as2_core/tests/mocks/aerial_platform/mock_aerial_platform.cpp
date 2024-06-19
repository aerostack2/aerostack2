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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file mock_aerial_platform.cpp
 *
 * Class to test the aerial platform node implementation
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "mock_aerial_platform.hpp"

namespace as2
{

namespace mock
{

PlatformMockNode::PlatformMockNode(
  const std::string & name_space)
: rclcpp::Node("platform_node_test", name_space)
{
  // Services clients
  arm_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
    as2_names::services::platform::set_arming_state);

  offboard_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
    as2_names::services::platform::set_offboard_mode);

  state_machine_srv_cli_ = this->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(
    as2_names::services::platform::set_platform_state_machine_event);

  takeoff_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
    as2_names::services::platform::takeoff);

  land_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
    as2_names::services::platform::land);

  control_mode_srv_cli_ = this->create_client<as2_msgs::srv::SetControlMode>(
    as2_names::services::platform::set_platform_control_mode);

  // Suscribers
  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info,
    as2_names::topics::platform::qos,
    [this](const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
      platform_info_ = *msg;
    });

  ground_truth_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::ground_truth::pose,
    as2_names::topics::ground_truth::qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      ground_truth_pose_ = *msg;
    });

  ground_truth_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::ground_truth::twist,
    as2_names::topics::ground_truth::qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      ground_truth_twist_ = *msg;
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    as2_names::topics::sensor_measurements::odom,
    as2_names::topics::sensor_measurements::qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_ = *msg;
    });

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    as2_names::topics::sensor_measurements::gps,
    as2_names::topics::sensor_measurements::qos,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      gps_ = *msg;
    });

  // Publishers
  trajectory_command_pub_ = this->create_publisher<as2_msgs::msg::TrajectoryPoint>(
    as2_names::topics::actuator_command::trajectory,
    as2_names::topics::actuator_command::qos);

  pose_command_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::actuator_command::pose,
    as2_names::topics::actuator_command::qos);

  twist_command_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::actuator_command::twist,
    as2_names::topics::actuator_command::qos);

  thrust_command_pub_ = this->create_publisher<as2_msgs::msg::Thrust>(
    as2_names::topics::actuator_command::thrust,
    as2_names::topics::actuator_command::qos);

  alert_event_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
    as2_names::topics::global::alert_event,
    as2_names::topics::global::qos);

  last_print_state_time_ = this->now();
}

PlatformMockNode::~PlatformMockNode()
{
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture PlatformMockNode::setArmingStateSrvCall(
  bool arm)
{
  RCLCPP_INFO(this->get_logger(), "Setting arming state to %s", arm ? "true" : "false");
  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = arm;
  return callService<std_srvs::srv::SetBool>(request, arm_srv_cli_);
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture PlatformMockNode::setOffboardControlSrvCall(
  bool offboard)
{
  RCLCPP_INFO(this->get_logger(), "Setting offboard control to %s", offboard ? "true" : "false");
  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = offboard;
  return callService<std_srvs::srv::SetBool>(request, offboard_srv_cli_);
}

rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedFuture
PlatformMockNode::setPlatformStateMachineEventSrvCall(
  as2_msgs::msg::PlatformStateMachineEvent request_state)
{
  RCLCPP_INFO(
    this->get_logger(), "Setting platform state machine event to %d", request_state.event);
  auto request = std::make_shared<as2_msgs::srv::SetPlatformStateMachineEvent::Request>();
  request->event = request_state;
  return callService<as2_msgs::srv::SetPlatformStateMachineEvent>(
    request,
    state_machine_srv_cli_);
}

rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedFuture PlatformMockNode::setControlModeSrvCall(
  as2_msgs::msg::ControlMode control_mode)
{
  RCLCPP_INFO(
    this->get_logger(), "Setting control mode: [%s]",
    as2::control_mode::controlModeToString(control_mode).c_str());
  auto request = std::make_shared<as2_msgs::srv::SetControlMode::Request>();
  request->control_mode = control_mode;
  return callService<as2_msgs::srv::SetControlMode>(request, control_mode_srv_cli_);
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture PlatformMockNode::takeoffSrvCall()
{
  RCLCPP_INFO(this->get_logger(), "Taking off");
  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  return callService<std_srvs::srv::SetBool>(request, takeoff_srv_cli_);
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture PlatformMockNode::landSrvCall()
{
  RCLCPP_INFO(this->get_logger(), "Landing");
  auto request =
    std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  return callService<std_srvs::srv::SetBool>(request, land_srv_cli_);
}


bool PlatformMockNode::takeoffPlatform(const bool spin_executor)
{
  // Arm
  if (!setArmingState(true, spin_executor)) {
    return false;
  }

  // Set offboard
  if (!setOffboardControl(true, spin_executor)) {
    return false;
  }

  // Send transition event TAKE_OFF
  as2_msgs::msg::PlatformStateMachineEvent takeoff_event;
  takeoff_event.event = as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF;
  if (!setPlatformStateMachineEvent(takeoff_event, spin_executor)) {
    return false;
  }

  // Takeoff
  if (!takeoff(spin_executor)) {
    return false;
  }

  return true;
}

bool PlatformMockNode::landPlatform(const bool spin_executor)
{
  // Send transition event LAND
  as2_msgs::msg::PlatformStateMachineEvent landing_event;
  landing_event.event = as2_msgs::msg::PlatformStateMachineEvent::LAND;
  if (!setPlatformStateMachineEvent(landing_event, spin_executor)) {
    return false;
  }

  // Land
  if (!land(spin_executor)) {
    return false;
  }

  // Disarm
  if (!setArmingState(false, spin_executor)) {
    return false;
  }
  return true;
}

bool PlatformMockNode::takeoff(const bool spin_executor)
{
  auto takeoff_future = takeoffSrvCall();
  if (!waitResponse<std_srvs::srv::SetBool>(takeoff_future, spin_executor)) {
    RCLCPP_ERROR(this->get_logger(), "Error taking off");
    return false;
  }
  return true;
}

bool PlatformMockNode::land(const bool spin_executor)
{
  auto land_future = landSrvCall();
  if (!waitResponse<std_srvs::srv::SetBool>(land_future, spin_executor)) {
    RCLCPP_ERROR(this->get_logger(), "Error landing");
    return false;
  }
  return true;
}

bool PlatformMockNode::setArmingState(const bool arm, const bool spin_executor)
{
  auto arm_future = setArmingStateSrvCall(arm);
  if (!waitResponse<std_srvs::srv::SetBool>(arm_future, spin_executor)) {
    RCLCPP_ERROR(this->get_logger(), "Error setting arming state");
    return false;
  }
  return true;
}

bool PlatformMockNode::setOffboardControl(const bool offboard, const bool spin_executor)
{
  auto offboard_future = setOffboardControlSrvCall(offboard);
  if (!waitResponse<std_srvs::srv::SetBool>(offboard_future, spin_executor)) {
    RCLCPP_ERROR(this->get_logger(), "Error setting offboard control");
    return false;
  }
  return true;
}

bool PlatformMockNode::setPlatformStateMachineEvent(
  const as2_msgs::msg::PlatformStateMachineEvent & state_machine_event,
  const bool spin_executor)
{
  auto state_machine_future = setPlatformStateMachineEventSrvCall(state_machine_event);
  if (!waitResponse<as2_msgs::srv::SetPlatformStateMachineEvent>(
      state_machine_future,
      spin_executor))
  {
    RCLCPP_ERROR(this->get_logger(), "Error setting platform state machine event");
    return false;
  }
  return true;
}

bool PlatformMockNode::setControlMode(
  const as2_msgs::msg::ControlMode & control_mode,
  const bool spin_executor)
{
  auto control_mode_future = setControlModeSrvCall(control_mode);
  if (!waitResponse<as2_msgs::srv::SetControlMode>(control_mode_future, spin_executor)) {
    RCLCPP_ERROR(this->get_logger(), "Error setting control mode");
    return false;
  }
  return true;
}

rclcpp::Time PlatformMockNode::getTime()
{
  return this->now();
}

const as2_msgs::msg::PlatformInfo & PlatformMockNode::getPlatformInfo() const
{
  return platform_info_;
}

const geometry_msgs::msg::PoseStamped & PlatformMockNode::getGroundThruthPose() const
{
  return ground_truth_pose_;
}

const geometry_msgs::msg::TwistStamped & PlatformMockNode::getGroundThruthTwist() const
{
  return ground_truth_twist_;
}

const nav_msgs::msg::Odometry & PlatformMockNode::getOdometry() const
{
  return odom_;
}

const sensor_msgs::msg::NavSatFix & PlatformMockNode::getGps() const
{
  return gps_;
}

void PlatformMockNode::setCommandTrajectoryPoint(
  const as2_msgs::msg::TrajectoryPoint & trajectory_point)
{
  trajectory_command_ = trajectory_point;
}

void PlatformMockNode::setCommandPose(const geometry_msgs::msg::PoseStamped & pose)
{
  pose_command_ = pose;
}

void PlatformMockNode::setCommandTwist(const geometry_msgs::msg::TwistStamped & twist)
{
  twist_command_ = twist;
}

void PlatformMockNode::setCommandThrust(const as2_msgs::msg::Thrust & thrust)
{
  thrust_command_ = thrust;
}

void PlatformMockNode::sendCommands()
{
  // Update time stamp
  auto current_time = this->now();
  trajectory_command_.header.stamp = current_time;
  pose_command_.header.stamp = current_time;
  twist_command_.header.stamp = current_time;
  thrust_command_.header.stamp = current_time;

  trajectory_command_pub_->publish(trajectory_command_);
  pose_command_pub_->publish(pose_command_);
  twist_command_pub_->publish(twist_command_);
  thrust_command_pub_->publish(thrust_command_);
}

void PlatformMockNode::setCommandSendTimerState(const bool enable, const double frequency)
{
  if (!enable) {
    command_timer_.reset();
    return;
  }
  // Timer
  command_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::duration<double>(1.0 / frequency),
    std::bind(&PlatformMockNode::sendCommands, this));
}

void PlatformMockNode::publishAlertEvent(const as2_msgs::msg::AlertEvent & alert_event)
{
  alert_event_pub_->publish(alert_event);
}

void PlatformMockNode::printState(const double print_state_period)
{
  rclcpp::Time current_time = this->now();
  if ((current_time - last_print_state_time_).seconds() < print_state_period) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "\nCURRENT_STATE");
  RCLCPP_INFO(this->get_logger(), "Armed: %s", platform_info_.armed ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Offboard: %s", platform_info_.offboard ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "Control mode: [%s]",
    as2::control_mode::controlModeToString(platform_info_.current_control_mode).c_str());

  RCLCPP_INFO(
    this->get_logger(), "Ground truth pose position: [%f, %f, %f]",
    ground_truth_pose_.pose.position.x,
    ground_truth_pose_.pose.position.y,
    ground_truth_pose_.pose.position.z);
  RCLCPP_INFO(
    this->get_logger(), "Ground truth pose orientation: [%f, %f, %f, %f]",
    ground_truth_pose_.pose.orientation.x,
    ground_truth_pose_.pose.orientation.y,
    ground_truth_pose_.pose.orientation.z,
    ground_truth_pose_.pose.orientation.w);

  RCLCPP_INFO(
    this->get_logger(), "Ground truth twist linear: [%f, %f, %f]",
    ground_truth_twist_.twist.linear.x,
    ground_truth_twist_.twist.linear.y,
    ground_truth_twist_.twist.linear.z);

  RCLCPP_INFO(
    this->get_logger(), "Ground truth twist angular: [%f, %f, %f]",
    ground_truth_twist_.twist.angular.x,
    ground_truth_twist_.twist.angular.y,
    ground_truth_twist_.twist.angular.z);

  RCLCPP_INFO(
    this->get_logger(), "Odometry pose position: [%f, %f, %f]",
    odom_.pose.pose.position.x,
    odom_.pose.pose.position.y,
    odom_.pose.pose.position.z);

  RCLCPP_INFO(
    this->get_logger(), "Odometry pose orientation: [%f, %f, %f, %f]",
    odom_.pose.pose.orientation.x,
    odom_.pose.pose.orientation.y,
    odom_.pose.pose.orientation.z,
    odom_.pose.pose.orientation.w);

  RCLCPP_INFO(
    this->get_logger(), "Odometry twist linear: [%f, %f, %f]",
    odom_.twist.twist.linear.x,
    odom_.twist.twist.linear.y,
    odom_.twist.twist.linear.z);

  RCLCPP_INFO(
    this->get_logger(), "Odometry twist angular: [%f, %f, %f]",
    odom_.twist.twist.angular.x,
    odom_.twist.twist.angular.y,
    odom_.twist.twist.angular.z);

  RCLCPP_INFO(this->get_logger(), "GPS latitude: %f", gps_.latitude);
  RCLCPP_INFO(this->get_logger(), "GPS longitude: %f", gps_.longitude);
  RCLCPP_INFO(this->get_logger(), "GPS altitude: %f\n", gps_.altitude);
  last_print_state_time_ = current_time;
}

}  // namespace mock
}  // namespace as2
