// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       motion_reference_handlers_gtest.cpp
 *  \brief      Tests for the destination of the motion references, either the motion
 *              controller or the aerial platform.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "as2_motion_reference_handlers/position_motion.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "gtest/gtest.h"

namespace as2
{

namespace
{

using std::chrono_literals::operator""ms;

/**
 * @class DestinationsMock, node playing both destinations of the motion
 * references, the aerial platform and the motion controller, to tell which one
 * the handlers are talking to
 */
class DestinationsMock : public rclcpp::Node
{
public:
  DestinationsMock()
  : rclcpp::Node("destinations_mock")
  {
    // Platform side
    platform_mode_srv_ = create_service<as2_msgs::srv::SetControlMode>(
      as2_names::services::platform::set_platform_control_mode,
      [this](
        const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
        as2_msgs::srv::SetControlMode::Response::SharedPtr response) {
        requested_mode_ = request->control_mode;
        platform_mode_requests_++;
        response->success = true;
      });

    platform_info_pub_ = create_publisher<as2_msgs::msg::PlatformInfo>(
      as2_names::topics::platform::info, as2_names::topics::platform::qos);

    actuator_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::actuator_command::pose, as2_names::topics::actuator_command::qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_command_ = *msg;
        actuator_pose_commands_++;
      });

    actuator_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        (void)msg;
        actuator_twist_commands_++;
      });

    // Controller side
    controller_mode_srv_ = create_service<as2_msgs::srv::SetControlMode>(
      as2_names::services::controller::set_control_mode,
      [this](
        const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
        as2_msgs::srv::SetControlMode::Response::SharedPtr response) {
        requested_mode_ = request->control_mode;
        controller_mode_requests_++;
        response->success = true;
      });

    controller_info_pub_ = create_publisher<as2_msgs::msg::ControllerInfo>(
      as2_names::topics::controller::info, rclcpp::QoS(1));

    reference_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::motion_reference::pose, as2_names::topics::motion_reference::qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_command_ = *msg;
        reference_pose_commands_++;
      });

    reference_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::motion_reference::twist, as2_names::topics::motion_reference::qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        (void)msg;
        reference_twist_commands_++;
      });

    info_timer_ = create_wall_timer(
      100ms, [this]() {
        as2_msgs::msg::PlatformInfo platform_info;
        platform_info.header.stamp = now();
        platform_info.current_control_mode = requested_mode_;
        platform_info_pub_->publish(platform_info);

        as2_msgs::msg::ControllerInfo controller_info;
        controller_info.header.stamp = now();
        controller_info.input_control_mode = requested_mode_;
        controller_info_pub_->publish(controller_info);
      });
  }

  as2_msgs::msg::ControlMode requested_mode_;
  geometry_msgs::msg::PoseStamped pose_command_;
  std::atomic_int platform_mode_requests_ = {0};
  std::atomic_int controller_mode_requests_ = {0};
  std::atomic_int actuator_pose_commands_ = {0};
  std::atomic_int actuator_twist_commands_ = {0};
  std::atomic_int reference_pose_commands_ = {0};
  std::atomic_int reference_twist_commands_ = {0};

private:
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr platform_mode_srv_;
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr controller_mode_srv_;
  rclcpp::Publisher<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_pub_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr actuator_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr actuator_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_twist_sub_;
  rclcpp::TimerBase::SharedPtr info_timer_;
};

std::shared_ptr<as2::Node> makeHandlerNode(const bool use_actuator_commands)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_actuator_commands", use_actuator_commands);
  return std::make_shared<as2::Node>("motion_reference_test_node", options);
}

/**
 * @brief Wait until a condition holds, spinning nothing: the nodes are spun by
 * the executor thread of the test
 */
bool waitFor(const std::function<bool()> & condition, const int timeout_ms = 5000)
{
  for (int waited_ms = 0; waited_ms < timeout_ms; waited_ms += 10) {
    if (condition()) {return true;}
    std::this_thread::sleep_for(10ms);
  }
  return condition();
}

}  // namespace

TEST(MotionReferenceHandlersTest, ActuatorCommandsGoToThePlatform)
{
  auto destinations = std::make_shared<DestinationsMock>();
  auto node = makeHandlerNode(true);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(destinations);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  as2::motionReferenceHandlers::PositionMotion position_handler(node.get());

  EXPECT_TRUE(
    position_handler.sendPositionCommandWithYawAngle(
      "earth", 1.0, 2.0, 3.0, 0.0, "earth", 0.5, 0.5, 0.5));

  EXPECT_TRUE(
    waitFor(
      [&destinations]() {
        const bool pose_received = destinations->actuator_pose_commands_ > 0;
        const bool twist_received = destinations->actuator_twist_commands_ > 0;
        return pose_received && twist_received;
      }));

  // The mode is negotiated with the platform, and the commands reach it
  EXPECT_EQ(destinations->platform_mode_requests_, 1);
  EXPECT_EQ(destinations->controller_mode_requests_, 0);
  EXPECT_EQ(
    destinations->requested_mode_.control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_EQ(destinations->requested_mode_.yaw_mode, as2_msgs::msg::ControlMode::YAW_ANGLE);
  EXPECT_EQ(destinations->pose_command_.pose.position.x, 1.0);

  // The motion controller is out of the chain
  EXPECT_EQ(destinations->reference_pose_commands_, 0);
  EXPECT_EQ(destinations->reference_twist_commands_, 0);

  executor.cancel();
  executor_thread.join();
}

TEST(MotionReferenceHandlersTest, MotionReferencesGoToTheControllerByDefault)
{
  auto destinations = std::make_shared<DestinationsMock>();
  auto node = makeHandlerNode(false);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(destinations);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  as2::motionReferenceHandlers::PositionMotion position_handler(node.get());

  EXPECT_TRUE(
    position_handler.sendPositionCommandWithYawAngle(
      "earth", 1.0, 2.0, 3.0, 0.0, "earth", 0.5, 0.5, 0.5));

  EXPECT_TRUE(
    waitFor(
      [&destinations]() {
        const bool pose_received = destinations->reference_pose_commands_ > 0;
        const bool twist_received = destinations->reference_twist_commands_ > 0;
        return pose_received && twist_received;
      }));

  // The mode is negotiated with the controller, and the commands reach it
  EXPECT_EQ(destinations->controller_mode_requests_, 1);
  EXPECT_EQ(destinations->platform_mode_requests_, 0);
  EXPECT_EQ(destinations->pose_command_.pose.position.x, 1.0);

  // The platform only gets commands through the controller
  EXPECT_EQ(destinations->actuator_pose_commands_, 0);
  EXPECT_EQ(destinations->actuator_twist_commands_, 0);

  executor.cancel();
  executor_thread.join();
}

}  // namespace as2

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
