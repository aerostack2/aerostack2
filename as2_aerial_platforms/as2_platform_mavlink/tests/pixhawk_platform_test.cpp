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
 * @file mavlink_platform_test.cpp
 *
 * Pixhawk test
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */


#include <iostream>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mavlink_platform.hpp"
#include "as2_core/mocks/aerial_platform/mock_aerial_platform.hpp"
#include "as2_core/mocks/executor_thread_util/executor_thread_util.hpp"
#include <as2_msgs/msg/platform_state_machine_event.hpp>

namespace as2_platform_mavlink
{
std::shared_ptr<MavlinkPlatform> get_node(
  const std::string & name_space = "test_mavlink_platform")
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_mavlink");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_config_file.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "--params-file",
    platform_config_file,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<MavlinkPlatform>(node_options);
}

void spinForTime(
  double seconds, rclcpp::executors::SingleThreadedExecutor & executor,
  std::shared_ptr<as2::mock::PlatformMockNode> test_node)
{
  // Current ros2 time
  auto start_time = test_node->getTime();
  while ((test_node->getTime() - start_time).seconds() < seconds) {
    // Spin
    executor.spin_some();

    // Print state
    // test_node->printState(seconds * 0.5);

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return;
}

void spinForTime(
  double seconds,
  std::shared_ptr<as2::mock::PlatformMockNode> test_node)
{
  // Current ros2 time
  auto start_time = test_node->getTime();
  while ((test_node->getTime() - start_time).seconds() < seconds) {
    // Print state
    // test_node->printState(seconds * 0.5);

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return;
}

int platform_test()
{
  std::string ros_namespace = "drone0";
  auto test_node = std::make_shared<as2::mock::PlatformMockNode>(ros_namespace);
  auto tello_node = get_node(ros_namespace);

  // Executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Add nodes to executor
  executor->add_node(test_node);
  executor->add_node(tello_node);

  // Executor thread
  as2::mock::ExecutorThreadUtil executor_thread_util(executor, 200.0);
  executor_thread_util.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Enable test command send timer
  test_node->setCommandSendTimerState(true, 100.0);

  std::string base_frame = as2::tf::generateTfName(
    test_node->get_namespace(), "base_link");

  // Wait 2 seconds
  spinForTime(2.0, test_node);

  // Arm
  if (!test_node->setArmingState(true, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Set offboard
  if (!test_node->setOffboardControl(true, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Send transition event TAKE_OFF
  as2_msgs::msg::PlatformStateMachineEvent takeoff_event;
  takeoff_event.event = as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF;
  if (!test_node->setPlatformStateMachineEvent(takeoff_event, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Set takeoff_twist actuator command
  geometry_msgs::msg::TwistStamped takeoff_twist;
  takeoff_twist.header.frame_id = base_frame;
  takeoff_twist.twist.linear.x = 0.0;
  takeoff_twist.twist.linear.y = 0.0;
  takeoff_twist.twist.linear.z = 0.5;
  takeoff_twist.twist.angular.x = 0.0;
  takeoff_twist.twist.angular.y = 0.0;
  takeoff_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(takeoff_twist);
  spinForTime(0.5, test_node);

  // Set control mode to speed for takeoff
  as2_msgs::msg::ControlMode control_mode;
  control_mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  if (!test_node->setControlMode(control_mode, false)) {
    return -1;
  }
  spinForTime(5.0, test_node);

  // Send transition event TOOK_OFF
  as2_msgs::msg::PlatformStateMachineEvent tookoff_event;
  tookoff_event.event = as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF;
  if (!test_node->setPlatformStateMachineEvent(tookoff_event, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Hover
  geometry_msgs::msg::TwistStamped hover_twist;
  hover_twist.header.frame_id = base_frame;
  hover_twist.twist.linear.x = 0.0;
  hover_twist.twist.linear.y = 0.0;
  hover_twist.twist.linear.z = 0.0;
  hover_twist.twist.angular.x = 0.0;
  hover_twist.twist.angular.y = 0.0;
  hover_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(hover_twist);
  spinForTime(5.0, test_node);

  // Move forward
  geometry_msgs::msg::TwistStamped forward_twist;
  forward_twist.header.frame_id = as2::tf::generateTfName(
    test_node->get_namespace(), base_frame);
  forward_twist.twist.linear.x = 0.5;
  forward_twist.twist.linear.y = 0.0;
  forward_twist.twist.linear.z = 0.0;
  forward_twist.twist.angular.x = 0.0;
  forward_twist.twist.angular.y = 0.0;
  forward_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(forward_twist);
  spinForTime(5.0, test_node);

  // Move backward
  geometry_msgs::msg::TwistStamped backward_twist;
  backward_twist.header.frame_id = base_frame;
  backward_twist.twist.linear.x = -0.5;
  backward_twist.twist.linear.y = 0.0;
  backward_twist.twist.linear.z = 0.0;
  backward_twist.twist.angular.x = 0.0;
  backward_twist.twist.angular.y = 0.0;
  backward_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(backward_twist);
  spinForTime(5.0, test_node);

  // Send transition event LAND
  as2_msgs::msg::PlatformStateMachineEvent land_event;
  land_event.event = as2_msgs::msg::PlatformStateMachineEvent::LAND;
  if (!test_node->setPlatformStateMachineEvent(land_event, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Set land_twist actuator command
  geometry_msgs::msg::TwistStamped land_twist;
  land_twist.header.frame_id = base_frame;
  land_twist.twist.linear.x = 0.0;
  land_twist.twist.linear.y = 0.0;
  land_twist.twist.linear.z = -0.5;
  land_twist.twist.angular.x = 0.0;
  land_twist.twist.angular.y = 0.0;
  land_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(land_twist);
  spinForTime(10.0, test_node);

  // Send transition event LANDED
  as2_msgs::msg::PlatformStateMachineEvent landed_event;
  landed_event.event = as2_msgs::msg::PlatformStateMachineEvent::LANDED;
  if (!test_node->setPlatformStateMachineEvent(landed_event, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Send disarm command
  if (!test_node->setArmingState(false, false)) {
    return -1;
  }
  spinForTime(0.5, test_node);

  // Clean
  executor_thread_util.stop();
  executor->remove_node(test_node);
  executor->remove_node(tello_node);

  return 0;
}
}  // namespace as2_platform_mavlink


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto result = as2_platform_mavlink::platform_test();

  rclcpp::shutdown();
  return result;
}
