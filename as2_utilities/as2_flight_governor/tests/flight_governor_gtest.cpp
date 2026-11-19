// Copyright 2026 Vishwa
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
//    * Neither the name of the copyright holder nor the names of its
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
* @file flight_governor_gtest.cpp
*
* End-to-end tests of the flight governor node over ROS 2 topics.
*
* @author Vishwa
*/

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/topics.hpp"
#include "as2_flight_governor/flight_governor.hpp"

using namespace std::chrono_literals;
namespace topics = as2_names::topics;
using as2_flight_governor::FlightGovernor;
using as2_msgs::msg::ControlMode;
using as2_msgs::msg::GovernorVerdict;

class FlightGovernorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int counter = 0;
    const std::string ns = "/governor_test_" + std::to_string(counter++);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    options.parameter_overrides(
    {
      rclcpp::Parameter("policies_file", std::string(CONFIG_DIR) + "/policies.yaml"),
      rclcpp::Parameter("state_timeout", 1.0),
    });
    governor_ = std::make_shared<FlightGovernor>(options);
    tester_ = std::make_shared<rclcpp::Node>("tester", ns);

    state_pub_ = tester_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topics::self_localization::pose, topics::self_localization::qos);
    info_pub_ = tester_->create_publisher<as2_msgs::msg::ControllerInfo>(
      topics::controller::info, topics::controller::qos_info);
    pose_pub_ = tester_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topics::motion_reference::pose, topics::motion_reference::qos);
    twist_pub_ = tester_->create_publisher<geometry_msgs::msg::TwistStamped>(
      topics::motion_reference::twist, topics::motion_reference::qos);

    const std::string out = as2_flight_governor::kOutputPrefix;
    pose_sub_ = tester_->create_subscription<geometry_msgs::msg::PoseStamped>(
      out + topics::motion_reference::pose, topics::motion_reference::qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {pose_out_ = *msg;});
    twist_sub_ = tester_->create_subscription<geometry_msgs::msg::TwistStamped>(
      out + topics::motion_reference::twist, topics::motion_reference::qos,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {twist_out_ = *msg;});
    verdict_sub_ = tester_->create_subscription<GovernorVerdict>(
      out + "verdict", rclcpp::QoS(10),
      [this](GovernorVerdict::SharedPtr msg) {verdict_ = *msg;});
    reset_client_ = tester_->create_client<std_srvs::srv::Trigger>(out + "reset");

    executor_.add_node(governor_);
    executor_.add_node(tester_);
    // Let discovery settle so that the first messages are not lost.
    spinFor(300ms);
  }

  void TearDown() override
  {
    executor_.remove_node(governor_);
    executor_.remove_node(tester_);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
    }
  }

  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 3s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      if (done()) {return true;}
    }
    return false;
  }

  void setMode(int8_t mode)
  {
    as2_msgs::msg::ControllerInfo info;
    info.input_control_mode.control_mode = mode;
    ASSERT_TRUE(
      spinUntil(
        [&]() {
          info_pub_->publish(info);
          return governor_->count_publishers(topics::controller::info) > 0;
        }));
    spinFor(100ms);
  }

  void publishState(double x, double y, double z, double age = 0.0)
  {
    geometry_msgs::msg::PoseStamped state;
    state.header.frame_id = governor_->getOdomFrameId();
    state.header.stamp = tester_->now() - rclcpp::Duration::from_seconds(age);
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;
    state.pose.orientation.w = 1.0;
    state_pub_->publish(state);
    spinFor(50ms);
  }

  /// Publish a twist reference until the governor forwards something.
  bool sendTwist(double x, double y, double z)
  {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = governor_->getOdomFrameId();
    twist.twist.linear.x = x;
    twist.twist.linear.y = y;
    twist.twist.linear.z = z;
    twist_out_.reset();
    verdict_.reset();
    return spinUntil(
      [&]() {
        twist.header.stamp = tester_->now();
        twist_pub_->publish(twist);
        return twist_out_.has_value() && verdict_.has_value();
      });
  }

  bool sendPose(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = governor_->getOdomFrameId();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    pose_out_.reset();
    verdict_.reset();
    return spinUntil(
      [&]() {
        pose.header.stamp = tester_->now();
        pose_pub_->publish(pose);
        return pose_out_.has_value() && verdict_.has_value();
      });
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<FlightGovernor> governor_;
  std::shared_ptr<rclcpp::Node> tester_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<GovernorVerdict>::SharedPtr verdict_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

  std::optional<geometry_msgs::msg::PoseStamped> pose_out_;
  std::optional<geometry_msgs::msg::TwistStamped> twist_out_;
  std::optional<GovernorVerdict> verdict_;
};

TEST_F(FlightGovernorTest, HoldsBackReferencesWithoutControllerInfo) {
  publishState(5, -5, 1);
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = governor_->getOdomFrameId();
  twist.twist.linear.x = 1.0;
  for (int i = 0; i < 20; ++i) {
    twist_pub_->publish(twist);
    spinFor(20ms);
  }
  EXPECT_FALSE(twist_out_.has_value());
}

TEST_F(FlightGovernorTest, SpeedModeClampsVelocity) {
  setMode(ControlMode::SPEED);
  publishState(5, -5, 1);
  ASSERT_TRUE(sendTwist(3, 0, 0));
  EXPECT_NEAR(twist_out_->twist.linear.x, 2.0, 1e-6);  // global_speed: 2 m/s
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::CLAMP);
  EXPECT_TRUE(verdict_->governed);
  EXPECT_EQ(verdict_->policies, std::vector<std::string>{"global_speed"});
}

TEST_F(FlightGovernorTest, SpeedModeAllowIsExactPassThrough) {
  setMode(ControlMode::SPEED);
  publishState(5, -5, 1);
  ASSERT_TRUE(sendTwist(0.3, -0.2, 0.1));
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::ALLOW);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.y, -0.2);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.z, 0.1);
}

TEST_F(FlightGovernorTest, PositionModeCapsUnlimitedSpeedLimits) {
  setMode(ControlMode::POSITION);
  publishState(5, -5, 1);
  // 0 means "no limit" to the controller, and a single 0 disables every axis.
  ASSERT_TRUE(sendTwist(0, 0, 0));
  const auto & v = twist_out_->twist.linear;
  EXPECT_GT(v.x, 0.0);
  EXPECT_GT(v.y, 0.0);
  EXPECT_GT(v.z, 0.0);
  EXPECT_NEAR(std::hypot(v.x, v.y), 2.0, 1e-6);
  EXPECT_NEAR(v.z, 1.0, 1e-6);
  EXPECT_EQ(verdict_->command_type, "speed_limit");
}

TEST_F(FlightGovernorTest, PositionModeClampsTargetIntoGeofence) {
  setMode(ControlMode::POSITION);
  publishState(5, -5, 1);
  ASSERT_TRUE(sendPose(50, -5, 1));
  EXPECT_NEAR(pose_out_->pose.position.x, 10.0, 1e-6);
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::CLAMP);
  EXPECT_EQ(verdict_->policies, std::vector<std::string>{"arena"});
}

TEST_F(FlightGovernorTest, StaleStateVetoesUntilReset) {
  setMode(ControlMode::SPEED);
  publishState(5, -5, 1, 5.0);  // 5 s old, timeout is 1 s
  ASSERT_TRUE(sendTwist(1, 0, 0));
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::VETO);
  EXPECT_TRUE(verdict_->latched);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.x, 0.0);

  // Fresh state alone does not clear a latched veto.
  publishState(5, -5, 1);
  ASSERT_TRUE(sendTwist(1, 0, 0));
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::VETO);

  ASSERT_TRUE(reset_client_->wait_for_service(2s));
  auto future = reset_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(
    spinUntil(
      [&]() {return future.wait_for(0s) == std::future_status::ready;}));
  EXPECT_TRUE(future.get()->success);

  publishState(5, -5, 1);
  ASSERT_TRUE(sendTwist(1, 0, 0));
  EXPECT_EQ(verdict_->verdict, GovernorVerdict::ALLOW);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.x, 1.0);
}

TEST_F(FlightGovernorTest, UngovernedModeIsFlagged) {
  setMode(ControlMode::ATTITUDE);
  publishState(5, -5, 1);
  ASSERT_TRUE(sendTwist(9, 9, 9));
  EXPECT_FALSE(verdict_->governed);
  EXPECT_DOUBLE_EQ(twist_out_->twist.linear.x, 9.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
