// Copyright 2024 Universidad Politécnica de Madrid
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
 * @file raw_odometry_integration_gtest.cpp
 *
 * Integration tests for the raw_odometry plugin loaded into a real
 * StateEstimator ROS2 node. Tests verify the odometry passthrough path,
 * the GPS-based earth->map path, and the set_origin/get_origin services.
 */

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helper: create a StateEstimator node with the raw_odometry plugin
// ---------------------------------------------------------------------------

static std::shared_ptr<as2_state_estimator::StateEstimator> getRawOdometryNode(
  const std::string & node_name_prefix,
  const std::vector<std::string> & extra_param_overrides = {})
{
  const std::string ns = node_name_prefix;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_cfg = package_path + "/config/state_estimator_default.yaml";
  const std::string plugin_cfg =
    package_path + "/plugins/raw_odometry/config/plugin_default.yaml";

  std::vector<std::string> args = {
    "--ros-args",
    "-r", "__ns:=/" + ns,
    "-p", "namespace:=" + ns,
    "-p", "plugin_name:=raw_odometry",
    "--params-file", state_estimator_cfg,
    "--params-file", plugin_cfg,
  };
  for (const auto & override_arg : extra_param_overrides) {
    args.push_back("-p");
    args.push_back(override_arg);
  }

  auto opts = rclcpp::NodeOptions().arguments(args);
  auto node = std::make_shared<as2_state_estimator::StateEstimator>(opts);
  // The PluginWrapper calls StateEstimator::getInstance() to get the current node.
  // Since we created the node directly (not via getInstance()), we must register
  // it as the singleton so that the deferred setup() can find it.
  as2_state_estimator::StateEstimator::instance_ = node;
  return node;
}

// spin_some(duration) returns almost immediately when there is no ready work, so an explicit
// sleep is needed to guarantee each iteration actually consumes real wall-clock time (needed
// e.g. to let StateEstimator's 1s deferred setup() timer mature while nothing else is ready).
static void spinSome(
  rclcpp::executors::MultiThreadedExecutor & exec,
  int iterations = 10)
{
  for (int i = 0; i < iterations; ++i) {
    exec.spin_some(50ms);
    std::this_thread::sleep_for(50ms);
  }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(RawOdometryIntegrationTest, PluginLoads_NoThrow)
{
  EXPECT_NO_THROW(getRawOdometryNode("ro_load"));
}

TEST(RawOdometryIntegrationTest, PluginSpins_NoThrow)
{
  auto node = getRawOdometryNode("ro_spin");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  EXPECT_NO_THROW(spinSome(exec, 5));
}

TEST(RawOdometryIntegrationTest, Odometry_SetsEarthToMap_TfAvailable)
{
  auto node = getRawOdometryNode("ro_odom");
  auto pub_node = rclcpp::Node::make_shared("ro_odom_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/ro_odom/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_odom_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "ro_odom/odom";
  odom.child_frame_id = "ro_odom/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string base_frame = "ro_odom/base_link";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    odom.header.stamp = pub_node->now();
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) <<
    "earth->base_link TF chain should be available after publishing odometry";
}

TEST(RawOdometryIntegrationTest, Odometry_TwistPassthroughExact)
{
  auto node = getRawOdometryNode("ro_twist");
  auto pub_node = rclcpp::Node::make_shared("ro_twist_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/ro_twist/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto sub_node = rclcpp::Node::make_shared("ro_twist_sub");
  geometry_msgs::msg::TwistStamped::SharedPtr last_twist;
  auto twist_sub = sub_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/ro_twist/self_localization/twist", rclcpp::SensorDataQoS(),
    [&last_twist](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      last_twist = msg;
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(sub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "ro_twist/odom";
  odom.child_frame_id = "ro_twist/base_link";
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 1.5;
  odom.twist.twist.linear.y = -2.25;
  odom.twist.twist.linear.z = 0.5;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.1;

  bool twist_received = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!twist_received && pub_node->now() < deadline) {
    odom.header.stamp = pub_node->now();
    odom_pub->publish(odom);
    spinSome(exec, 5);
    twist_received = (last_twist != nullptr);
  }

  ASSERT_TRUE(twist_received) << "self_localization/twist should receive a message";
  EXPECT_NEAR(last_twist->twist.linear.x, 1.5, 1e-6);
  EXPECT_NEAR(last_twist->twist.linear.y, -2.25, 1e-6);
  EXPECT_NEAR(last_twist->twist.linear.z, 0.5, 1e-6);
  EXPECT_NEAR(last_twist->twist.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(last_twist->twist.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(last_twist->twist.angular.z, 0.1, 1e-6);
}

TEST(RawOdometryIntegrationTest, OdometryWrongFrameIds_StillProcessed_NoThrow)
{
  auto node = getRawOdometryNode("ro_wrong_frame");
  auto pub_node = rclcpp::Node::make_shared("ro_wrong_frame_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/ro_wrong_frame/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_wrong_frame_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "wrong_odom";  // mismatched — plugin only warns, still processes
  odom.child_frame_id = "wrong_base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string base_frame = "ro_wrong_frame/base_link";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  EXPECT_NO_THROW(
  {
    while (!tf_available && pub_node->now() < deadline) {
      odom.header.stamp = pub_node->now();
      odom_pub->publish(odom);
      spinSome(exec, 5);
      tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
    }
  });

  EXPECT_TRUE(tf_available) <<
    "raw_odometry should still process odometry with mismatched frame ids (warn-only)";
}

// earth→map and map→odom are both identity for the default (non-GPS) configuration, so the
// odometry pose must come back out of the TF tree unchanged at earth→base_link.
TEST(RawOdometryIntegrationTest, Odometry_EarthToBaseLinkMatchesOdometryPose)
{
  const std::string ns = "ro_odom_values";
  auto node = getRawOdometryNode(ns);
  auto pub_node = rclcpp::Node::make_shared("ro_odom_values_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_odom_values_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  const std::string base_frame = ns + "/base_link";
  const double kSqrtHalf = 0.70710678118654752;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = 1.5;
  odom.pose.pose.position.y = -2.5;
  odom.pose.pose.position.z = 3.5;
  odom.pose.pose.orientation.z = kSqrtHalf;  // 90° yaw
  odom.pose.pose.orientation.w = kSqrtHalf;

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    odom.header.stamp = pub_node->now();
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth→base_link TF should be available after publishing odometry";

  auto earth_to_base = tf_buffer->lookupTransform("earth", base_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_base.transform.translation.x, 1.5, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.translation.y, -2.5, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.translation.z, 3.5, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.rotation.z, kSqrtHalf, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.rotation.w, kSqrtHalf, 1e-6);
}

// set_earth_map=true pins earth→map to the configured parameters instead of leaving it identity.
TEST(RawOdometryIntegrationTest, ManualEarthToMap_UsesConfiguredTransform)
{
  const std::string ns = "ro_manual_earth_map";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.earth_map_transform.set_earth_map:=true",
    "raw_odometry.earth_map_transform.position.x:=10.0",
    "raw_odometry.earth_map_transform.position.y:=20.0",
    "raw_odometry.earth_map_transform.position.z:=30.0",
  });
  auto pub_node = rclcpp::Node::make_shared("ro_manual_earth_map_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_manual_earth_map_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  const std::string map_frame = ns + "/map";
  const std::string base_frame = ns + "/base_link";

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.w = 1.0;

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    odom.header.stamp = pub_node->now();
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth→base_link TF should be available after publishing odometry";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_map.transform.translation.x, 10.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.y, 20.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, 30.0, 1e-6);

  // The odometry pose is relative to odom, so earth→base_link is the configured offset plus it.
  auto earth_to_base = tf_buffer->lookupTransform("earth", base_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_base.transform.translation.x, 11.0, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.translation.y, 22.0, 1e-6);
  EXPECT_NEAR(earth_to_base.transform.translation.z, 33.0, 1e-6);
}

// A GPS fix with no prior set_origin auto-adopts its own coordinates as the origin, which
// makes any later set_origin call fail — pinned here because the ordering is easy to get wrong.
TEST(RawOdometryIntegrationTest, Gps_FixBeforeSetOrigin_LaterSetOriginRejected)
{
  const std::string ns = "ro_gps_fix_first";
  auto node = getRawOdometryNode(ns, {"raw_odometry.use_gps:=true"});
  auto pub_node = rclcpp::Node::make_shared("ro_gps_fix_first_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());

  auto client_node = rclcpp::Node::make_shared("ro_gps_fix_first_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(get_origin_client->wait_for_service(2s));

  auto call = [&](auto & client, auto request) {
      auto future = client->async_send_request(request);
      bool ready = false;
      auto deadline = client_node->now() + rclcpp::Duration(2, 0);
      while (!ready && client_node->now() < deadline) {
        spinSome(exec, 2);
        ready = future.wait_for(0s) == std::future_status::ready;
      }
      EXPECT_TRUE(ready);
      return future.get();
    };

  // Publish a fix first: the plugin adopts it as the origin.
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 40.0;
  fix.longitude = -3.0;
  fix.altitude = 650.0;
  for (int i = 0; i < 5; ++i) {
    fix.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    spinSome(exec, 2);
  }

  auto get_response = call(
    get_origin_client, std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  ASSERT_TRUE(get_response->success) << "the first GPS fix should have set the origin";
  EXPECT_NEAR(get_response->origin.latitude, 40.0, 1e-9);
  EXPECT_NEAR(get_response->origin.longitude, -3.0, 1e-9);
  EXPECT_NEAR(get_response->origin.altitude, 650.0, 1e-9);

  auto set_request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
  set_request->origin.latitude = 12.3;
  EXPECT_FALSE(call(set_origin_client, set_request)->success) <<
    "set_origin must be rejected once a GPS fix has already established the origin";
}

TEST(RawOdometryIntegrationTest, Gps_EnabledDefault_SetsEarthToMap_TfAvailable)
{
  auto node = getRawOdometryNode("ro_gps_default", {"raw_odometry.use_gps:=true"});
  auto pub_node = rclcpp::Node::make_shared("ro_gps_default_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/ro_gps_default/sensor_measurements/gps", rclcpp::SensorDataQoS());
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/ro_gps_default/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_gps_default_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 40.0;
  fix.longitude = -3.0;
  fix.altitude = 650.0;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "ro_gps_default/odom";
  odom.child_frame_id = "ro_gps_default/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string base_frame = "ro_gps_default/base_link";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    fix.header.stamp = pub_node->now();
    odom.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) <<
    "earth->base_link TF chain should be available after a GPS fix (use_gps=true)";
}

TEST(RawOdometryIntegrationTest, Gps_SetOriginThenFix_TranslationCorrect)
{
  const std::string ns = "ro_gps_origin";
  auto node = getRawOdometryNode(ns, {"raw_odometry.use_gps:=true"});
  auto pub_node = rclcpp::Node::make_shared("ro_gps_origin_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto client_node = rclcpp::Node::make_shared("ro_gps_origin_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");

  auto tf_node = rclcpp::Node::make_shared("ro_gps_origin_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(client_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(set_origin_client->wait_for_service(2s));

  auto request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
  request->origin.latitude = 0.0;
  request->origin.longitude = 0.0;
  request->origin.altitude = 0.0;

  auto future = set_origin_client->async_send_request(request);
  bool response_ready = false;
  auto rpc_deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!response_ready && pub_node->now() < rpc_deadline) {
    spinSome(exec, 2);
    response_ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(response_ready) << "set_origin service call should complete";
  EXPECT_TRUE(future.get()->success);

  // Fix ~0.0001 deg north of origin (~11.1 m north, GeographicLib LocalCartesian is East-North-Up)
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 0.0001;
  fix.longitude = 0.0;
  fix.altitude = 0.0;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string map_frame = ns + "/map";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    fix.header.stamp = pub_node->now();
    odom.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth->map TF should be available after the GPS fix";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  const double x = earth_to_map.transform.translation.x;
  const double y = earth_to_map.transform.translation.y;
  const double z = earth_to_map.transform.translation.z;

  EXPECT_GT(y, 5.0) << "north offset (y, ENU) should be positive and non-trivial";
  EXPECT_LT(y, 20.0) << "north offset (y, ENU) should be in the expected ~11 m ballpark";
  EXPECT_NEAR(x, 0.0, 5.0) << "east offset (x, ENU) should be near zero for a due-north fix";
  EXPECT_NEAR(z, 0.0, 5.0) << "up offset (z, ENU) should be near zero for equal altitudes";
}

TEST(RawOdometryIntegrationTest, Gps_GetOrigin_ReturnsSetValue)
{
  const std::string ns = "ro_gps_getorigin";
  auto node = getRawOdometryNode(ns, {"raw_odometry.use_gps:=true"});
  auto client_node = rclcpp::Node::make_shared("ro_gps_getorigin_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(set_origin_client->wait_for_service(2s));
  auto set_request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
  set_request->origin.latitude = 12.3;
  set_request->origin.longitude = 45.6;
  set_request->origin.altitude = 78.9;
  auto set_future = set_origin_client->async_send_request(set_request);
  bool set_ready = false;
  auto deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!set_ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    set_ready = set_future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(set_ready);
  ASSERT_TRUE(set_future.get()->success);

  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  auto get_future = get_origin_client->async_send_request(
    std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  bool get_ready = false;
  deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!get_ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    get_ready = get_future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(get_ready);
  auto response = get_future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NEAR(response->origin.latitude, 12.3, 1e-9);
  EXPECT_NEAR(response->origin.longitude, 45.6, 1e-9);
  EXPECT_NEAR(response->origin.altitude, 78.9, 1e-9);
}

TEST(RawOdometryIntegrationTest, Gps_SetOriginTwice_SecondCallFails)
{
  const std::string ns = "ro_gps_set_twice";
  auto node = getRawOdometryNode(ns, {"raw_odometry.use_gps:=true"});
  auto client_node = rclcpp::Node::make_shared("ro_gps_set_twice_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(set_origin_client->wait_for_service(2s));

  auto call_set_origin = [&](double lat) {
      auto request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
      request->origin.latitude = lat;
      auto future = set_origin_client->async_send_request(request);
      bool ready = false;
      auto deadline = client_node->now() + rclcpp::Duration(2, 0);
      while (!ready && client_node->now() < deadline) {
        spinSome(exec, 2);
        ready = future.wait_for(0s) == std::future_status::ready;
      }
      EXPECT_TRUE(ready);
      return future.get()->success;
    };

  EXPECT_TRUE(call_set_origin(1.0));
  EXPECT_FALSE(call_set_origin(2.0)) << "second set_origin call should be rejected";
}

TEST(RawOdometryIntegrationTest, Gps_GetOriginBeforeSet_ReturnsFailure)
{
  const std::string ns = "ro_gps_get_before_set";
  auto node = getRawOdometryNode(ns, {"raw_odometry.use_gps:=true"});
  auto client_node = rclcpp::Node::make_shared("ro_gps_get_before_set_client");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  auto future = get_origin_client->async_send_request(
    std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  bool ready = false;
  auto deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(ready);
  EXPECT_FALSE(future.get()->success);
}

// gps_origin.set_origin="manual" pins the geodetic datum from parameters, so the first fix is no
// longer adopted as the origin and earth->map comes out as the offset from the configured datum.
// Same expectation as Gps_SetOriginThenFix_TranslationCorrect, reached without the service.
TEST(RawOdometryIntegrationTest, Gps_ParamOrigin_TranslationFromConfiguredDatum)
{
  const std::string ns = "ro_gps_param_origin";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.gps_origin.set_origin:=manual",
    "raw_odometry.gps_origin.coordinates.latitude:=0.0",
    "raw_odometry.gps_origin.coordinates.longitude:=0.0",
    "raw_odometry.gps_origin.coordinates.altitude:=0.0",
  });
  auto pub_node = rclcpp::Node::make_shared("ro_gps_param_origin_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("ro_gps_param_origin_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  // Fix ~0.0001 deg north of the configured datum (~11.1 m north, ENU).
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 0.0001;
  fix.longitude = 0.0;
  fix.altitude = 0.0;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string map_frame = ns + "/map";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    fix.header.stamp = pub_node->now();
    odom.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth->map TF should be available after the GPS fix";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_GT(earth_to_map.transform.translation.y, 5.0) <<
    "the first fix must be offset from the configured datum, not adopted as the origin";
  EXPECT_LT(earth_to_map.transform.translation.y, 20.0) <<
    "north offset (y, ENU) should be in the expected ~11 m ballpark";
  EXPECT_NEAR(earth_to_map.transform.translation.x, 0.0, 5.0) <<
    "east offset (x, ENU) should be near zero for a due-north fix";
}

TEST(RawOdometryIntegrationTest, Gps_ParamOrigin_GetOriginReturnsConfiguredValue)
{
  const std::string ns = "ro_gps_param_get";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.gps_origin.set_origin:=manual",
    "raw_odometry.gps_origin.coordinates.latitude:=12.3",
    "raw_odometry.gps_origin.coordinates.longitude:=45.6",
    "raw_odometry.gps_origin.coordinates.altitude:=78.9",
  });
  auto client_node = rclcpp::Node::make_shared("ro_gps_param_get_client");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  auto future = get_origin_client->async_send_request(
    std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  bool ready = false;
  auto deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(ready);

  // No GPS fix was ever published: the origin comes purely from the parameters.
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NEAR(response->origin.latitude, 12.3, 1e-9);
  EXPECT_NEAR(response->origin.longitude, 45.6, 1e-9);
  EXPECT_NEAR(response->origin.altitude, 78.9, 1e-9);
}

// Pinning the datum in parameters consumes the one-shot set_origin, exactly as a service call
// or a GPS fix would. Worth pinning so the interaction between the two ways is not a surprise.
TEST(RawOdometryIntegrationTest, Gps_ParamOrigin_SetOriginServiceRejected)
{
  const std::string ns = "ro_gps_param_reject";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.gps_origin.set_origin:=manual",
    "raw_odometry.gps_origin.coordinates.latitude:=40.0",
  });
  auto client_node = rclcpp::Node::make_shared("ro_gps_param_reject_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  ASSERT_TRUE(set_origin_client->wait_for_service(2s));
  auto request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
  request->origin.latitude = 12.3;
  auto future = set_origin_client->async_send_request(request);
  bool ready = false;
  auto deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(ready);
  EXPECT_FALSE(future.get()->success) <<
    "set_origin must be rejected once the parameters have established the origin";
}

// use_gps and earth_map_transform compose rather than exclude each other: the datum still
// comes from GPS (and is still served by get_origin), but map is pinned by the parameters
// instead of being placed at the first fix. This is the only way to give map a heading.
TEST(RawOdometryIntegrationTest, Gps_ManualEarthToMap_PinsMapAndKeepsDatum)
{
  const std::string ns = "ro_gps_manual_map";
  const double kHalfPi = 1.5707963267948966;
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.earth_map_transform.set_earth_map:=true",
    "raw_odometry.earth_map_transform.position.x:=10.0",
    "raw_odometry.earth_map_transform.position.y:=20.0",
    "raw_odometry.earth_map_transform.position.z:=30.0",
    "raw_odometry.earth_map_transform.orientation.yaw:=" + std::to_string(kHalfPi),
  });
  auto pub_node = rclcpp::Node::make_shared("ro_gps_manual_map_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto client_node = rclcpp::Node::make_shared("ro_gps_manual_map_client");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  auto tf_node = rclcpp::Node::make_shared("ro_gps_manual_map_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(client_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 40.0;
  fix.longitude = -3.0;
  fix.altitude = 650.0;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string map_frame = ns + "/map";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    fix.header.stamp = pub_node->now();
    odom.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth->map TF should be available";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  const double kSqrtHalf = 0.70710678118654752;
  // The configured transform must survive the fix, which would otherwise have placed map at
  // the datum (translation 0,0,0, identity rotation) since the fix is the datum here.
  EXPECT_NEAR(earth_to_map.transform.translation.x, 10.0, 1e-6) <<
    "the GPS fix must not overwrite an earth->map pinned by earth_map_transform";
  EXPECT_NEAR(earth_to_map.transform.translation.y, 20.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, 30.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.rotation.z, kSqrtHalf, 1e-6) <<
    "earth->map should carry the configured 90 deg yaw";
  EXPECT_NEAR(earth_to_map.transform.rotation.w, kSqrtHalf, 1e-6);

  // The datum is still adopted from the fix and still served, so GPS-referenced missions work.
  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  auto future = get_origin_client->async_send_request(
    std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  bool ready = false;
  deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(ready);
  auto response = future.get();
  EXPECT_TRUE(response->success) << "pinning earth->map must not stop the datum being recorded";
  EXPECT_NEAR(response->origin.latitude, 40.0, 1e-9);
  EXPECT_NEAR(response->origin.longitude, -3.0, 1e-9);
}

// set_origin="service" is the mode that defers to an operator: the first fix is stored but
// not adopted, so earth->map stays unpublished until the service supplies a datum. Only this
// plugin reads a single fix ever, so the stored one has to survive until the call arrives.
TEST(RawOdometryIntegrationTest, Gps_ServiceMode_FixHeldUntilServiceCall)
{
  const std::string ns = "ro_gps_service_mode";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.gps_origin.set_origin:=service",
  });
  auto pub_node = rclcpp::Node::make_shared("ro_gps_service_mode_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto client_node = rclcpp::Node::make_shared("ro_gps_service_mode_client");
  auto set_origin_client = client_node->create_client<as2_msgs::srv::SetOrigin>(
    "/" + ns + "/set_origin");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  auto tf_node = rclcpp::Node::make_shared("ro_gps_service_mode_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(client_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  auto call = [&](auto & client, auto request) {
      auto future = client->async_send_request(request);
      bool ready = false;
      auto rpc_deadline = client_node->now() + rclcpp::Duration(2, 0);
      while (!ready && client_node->now() < rpc_deadline) {
        spinSome(exec, 2);
        ready = future.wait_for(0s) == std::future_status::ready;
      }
      EXPECT_TRUE(ready);
      return future.get();
    };

  // Fix ~0.0001 deg north of the datum the service will set (~11.1 m north, ENU).
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 0.0001;
  fix.longitude = 0.0;
  fix.altitude = 0.0;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_link";
  odom.pose.pose.orientation.w = 1.0;

  const std::string map_frame = ns + "/map";

  for (int i = 0; i < 6; ++i) {
    fix.header.stamp = pub_node->now();
    odom.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    odom_pub->publish(odom);
    spinSome(exec, 5);
  }

  EXPECT_FALSE(tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero)) <<
    "in service mode the fix must not place map on its own";
  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  EXPECT_FALSE(
    call(get_origin_client, std::make_shared<as2_msgs::srv::GetOrigin::Request>())->success) <<
    "in service mode the fix must not be adopted as the origin";

  // Now supply the datum. The fix stored earlier is consumed to place map.
  ASSERT_TRUE(set_origin_client->wait_for_service(2s));
  auto set_request = std::make_shared<as2_msgs::srv::SetOrigin::Request>();
  set_request->origin.latitude = 0.0;
  set_request->origin.longitude = 0.0;
  set_request->origin.altitude = 0.0;
  EXPECT_TRUE(call(set_origin_client, set_request)->success);

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    odom.header.stamp = pub_node->now();
    odom_pub->publish(odom);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }
  ASSERT_TRUE(tf_available) << "earth->map should appear once the service sets the origin";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_GT(earth_to_map.transform.translation.y, 5.0) <<
    "map should be placed at the stored fix, offset north of the datum the service supplied";
  EXPECT_LT(earth_to_map.transform.translation.y, 20.0);
}

TEST(RawOdometryIntegrationTest, Gps_InvalidOriginMode_FallsBackToFirstGps)
{
  const std::string ns = "ro_gps_bad_mode";
  auto node = getRawOdometryNode(
    ns, {
    "raw_odometry.use_gps:=true",
    "raw_odometry.gps_origin.set_origin:=not_a_mode",
  });
  auto pub_node = rclcpp::Node::make_shared("ro_gps_bad_mode_pub");
  auto gps_pub = pub_node->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/" + ns + "/sensor_measurements/gps", rclcpp::SensorDataQoS());

  auto client_node = rclcpp::Node::make_shared("ro_gps_bad_mode_client");
  auto get_origin_client = client_node->create_client<as2_msgs::srv::GetOrigin>(
    "/" + ns + "/get_origin");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(client_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin services/subs
  // aren't created until then) before relying on service/topic discovery.
  spinSome(exec, 30);

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 40.0;
  fix.longitude = -3.0;
  fix.altitude = 650.0;
  for (int i = 0; i < 5; ++i) {
    fix.header.stamp = pub_node->now();
    gps_pub->publish(fix);
    spinSome(exec, 2);
  }

  ASSERT_TRUE(get_origin_client->wait_for_service(2s));
  auto future = get_origin_client->async_send_request(
    std::make_shared<as2_msgs::srv::GetOrigin::Request>());
  bool ready = false;
  auto deadline = client_node->now() + rclcpp::Duration(2, 0);
  while (!ready && client_node->now() < deadline) {
    spinSome(exec, 2);
    ready = future.wait_for(0s) == std::future_status::ready;
  }
  ASSERT_TRUE(ready);

  // An unrecognised mode must warn and behave as first_gps, not silently disable GPS.
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NEAR(response->origin.latitude, 40.0, 1e-9);
  EXPECT_NEAR(response->origin.longitude, -3.0, 1e-9);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  // Explicitly release the last StateEstimator node (and its pluginlib ClassLoader) here,
  // while the process is still in a normal state. Left to the static destructor at program
  // exit, unloading the plugin's shared library races with the dynamic linker's own global
  // teardown and reliably segfaults inside class_loader.
  as2_state_estimator::StateEstimator::instance_.reset();
  rclcpp::shutdown();
  return result;
}
