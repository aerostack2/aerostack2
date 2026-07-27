// Copyright 2025 Universidad Politécnica de Madrid
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
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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
 * @file simple_ekf_integration_gtest.cpp
 *
 * Integration tests for the simple_ekf plugin loaded into a real
 * StateEstimator ROS2 node. Tests verify that the plugin loads, processes
 * messages, and produces TF output without crashing.
 */

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_msgs/msg/platform_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helper: create a StateEstimator node with the simple_ekf plugin
// ---------------------------------------------------------------------------

static std::shared_ptr<as2_state_estimator::StateEstimator> getSimpleEkfNode(
  const std::string & node_name_prefix = "test_simple_ekf",
  const std::vector<std::string> & extra_param_overrides = {})
{
  const std::string ns = node_name_prefix;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_cfg = package_path + "/config/state_estimator_default.yaml";
  const std::string plugin_cfg =
    package_path + "/plugins/simple_ekf/config/plugin_default.yaml";

  std::vector<std::string> args = {
    "--ros-args",
    "-r", "__ns:=/" + ns,
    "-p", "namespace:=" + ns,
    "-p", "plugin_name:=simple_ekf",
    "--params-file", state_estimator_cfg,
    "--params-file", plugin_cfg,
  };
  // Individual -p overrides after --params-file take precedence over the file's values.
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

// Spin the executor for a fixed number of iterations. spin_some(duration) returns almost
// immediately when there is no ready work, so an explicit sleep is needed to guarantee each
// iteration actually consumes real wall-clock time (needed e.g. to let StateEstimator's 1s
// deferred setup() timer mature while nothing else is ready yet).
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

TEST(SimpleEkfIntegrationTest, PluginLoads_NoThrow)
{
  EXPECT_NO_THROW(getSimpleEkfNode("test_load"));
}

TEST(SimpleEkfIntegrationTest, PluginSpins_NoThrow)
{
  auto node = getSimpleEkfNode("test_spin");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  EXPECT_NO_THROW(spinSome(exec, 5));
}

TEST(SimpleEkfIntegrationTest, ImuMessage_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_imu");

  // Helper node to publish IMU messages
  auto pub_node = rclcpp::Node::make_shared("test_imu_publisher");
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
    "/test_imu/sensor_measurements/imu", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);

  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = pub_node->now();
  imu.header.frame_id = "imu";
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.81;
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  // Identity orientation covariance
  imu.orientation_covariance[0] = -1.0;  // indicates orientation not estimated

  EXPECT_NO_THROW(
  {
    imu_pub->publish(imu);
    spinSome(exec, 10);
  });
}

// After publishing a PoseStamped in the "map" frame, earth→map TF must appear.
// The earth frame is global ("earth"); the map frame is namespaced ("test_pose/map").
TEST(SimpleEkfIntegrationTest, PoseStamped_SetsEarthToMap_TfAvailable)
{
  auto node = getSimpleEkfNode("test_pose");

  auto pub_node = rclcpp::Node::make_shared("test_pose_publisher");
  // Match the plugin's SensorDataQoS so publisher and subscriber are compatible
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/test_pose/ground_truth/pose", rclcpp::SensorDataQoS());
  auto imu_pub = pub_node->create_publisher<sensor_msgs::msg::Imu>(
    "/test_pose/sensor_measurements/imu", rclcpp::SensorDataQoS());

  // TF listener uses a dedicated node so it stays independent of the test node
  auto tf_node = rclcpp::Node::make_shared("test_pose_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);

  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  // Prepare messages upfront
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "test_pose/map";  // namespace-prefixed map frame
  pose.pose.orientation.w = 1.0;

  sensor_msgs::msg::Imu imu;
  imu.linear_acceleration.z = 9.81;
  imu.orientation_covariance[0] = -1.0;

  // Spin in a loop (up to 2 s), publishing on every iteration until TF appears.
  // Re-publishing ensures the message is sent after subscriptions are matched.
  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    imu.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    imu_pub->publish(imu);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(
      "test_pose/map", "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) << "earth→map TF should be available after publishing first pose";
}

// The default config only declares the "gazebo" PoseStamped update topic, so every non-pose
// message type has to be wired in explicitly through parameter overrides.
static std::vector<std::string> mocapUpdateTopicOverrides()
{
  return {
    "simple_ekf.update_topics:=[mocap]",
    "simple_ekf.mocap.topic:=mocap/rigid_bodies",
    "simple_ekf.mocap.type:=mocap4r2_msgs/msg/RigidBodies",
    // Passed as a bare integer on purpose: exercises the int→string fallback in onSetup().
    "simple_ekf.mocap.rigid_body_name:=33",
    "simple_ekf.mocap.set_earth_map:=true",
    "simple_ekf.mocap.use_message_covariance:=false",
  };
}

// Mocap poses are treated as being in the earth frame, so the first accepted rigid body
// pose becomes the earth→map transform verbatim.
TEST(SimpleEkfIntegrationTest, Mocap_ValidPose_SetsEarthToMapToPose)
{
  const std::string ns = "test_mocap_valid";
  auto node = getSimpleEkfNode(ns, mocapUpdateTopicOverrides());
  auto pub_node = rclcpp::Node::make_shared("test_mocap_valid_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/" + ns + "/mocap/rigid_bodies", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_mocap_valid_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  body.pose.position.x = 1.0;
  body.pose.position.y = 2.0;
  body.pose.position.z = 3.0;
  body.pose.orientation.w = 1.0;

  const std::string map_frame = ns + "/map";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    mocap4r2_msgs::msg::RigidBodies msg;
    msg.header.stamp = pub_node->now();
    msg.rigidbodies.push_back(body);
    mocap_pub->publish(msg);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }

  ASSERT_TRUE(tf_available) << "earth→map TF should be available after a valid mocap pose";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_map.transform.translation.x, 1.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.y, 2.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, 3.0, 1e-6);
}

// All-zero poses are the mocap "body not detected" sentinel: they must never reach the EKF,
// so earth→map must stay unset (which is only observable now that a mocap topic exists).
TEST(SimpleEkfIntegrationTest, Mocap_OnlyZeroPoses_EarthToMapNeverSet)
{
  const std::string ns = "test_mocap_zero";
  auto node = getSimpleEkfNode(ns, mocapUpdateTopicOverrides());
  auto pub_node = rclcpp::Node::make_shared("test_mocap_zero_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/" + ns + "/mocap/rigid_bodies", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_mocap_zero_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  // All-zero pose: position=(0,0,0), orientation=(0,0,0,0)
  body.pose.orientation.w = 0.0;

  for (int i = 0; i < 5; ++i) {
    mocap4r2_msgs::msg::RigidBodies msg;
    msg.header.stamp = pub_node->now();
    msg.rigidbodies.push_back(body);
    mocap_pub->publish(msg);
    spinSome(exec, 5);
  }

  EXPECT_FALSE(tf_buffer->canTransform("earth", ns + "/map", tf2::TimePointZero)) <<
    "all-zero mocap poses must be rejected, leaving earth→map unset";
}

// A rigid body name that never matches must be ignored entirely.
TEST(SimpleEkfIntegrationTest, Mocap_NonMatchingName_EarthToMapNeverSet)
{
  const std::string ns = "test_mocap_wrong_name";
  auto node = getSimpleEkfNode(ns, mocapUpdateTopicOverrides());
  auto pub_node = rclcpp::Node::make_shared("test_mocap_wrong_name_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/" + ns + "/mocap/rigid_bodies", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_mocap_wrong_name_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "99";  // configured name is "33"
  body.pose.position.x = 1.0;
  body.pose.position.y = 2.0;
  body.pose.position.z = 3.0;
  body.pose.orientation.w = 1.0;

  for (int i = 0; i < 5; ++i) {
    mocap4r2_msgs::msg::RigidBodies msg;
    msg.header.stamp = pub_node->now();
    msg.rigidbodies.push_back(body);
    mocap_pub->publish(msg);
    spinSome(exec, 5);
  }

  EXPECT_FALSE(tf_buffer->canTransform("earth", ns + "/map", tf2::TimePointZero)) <<
    "a non-matching rigid body name must leave earth→map unset";
}

// nav_msgs/msg/Odometry update topic. The pose arrives in the map frame, so earth→map is the
// inverse of the measured map→drone pose.
TEST(SimpleEkfIntegrationTest, Odometry_SetsEarthToMapAsInverseOfPose)
{
  const std::string ns = "test_odom";
  auto node = getSimpleEkfNode(
    ns, {
    "simple_ekf.update_topics:=[odom]",
    "simple_ekf.odom.topic:=sensor_measurements/odom",
    "simple_ekf.odom.type:=nav_msgs/msg/Odometry",
    "simple_ekf.odom.set_earth_map:=true",
    "simple_ekf.odom.use_message_covariance:=false",
  });
  auto pub_node = rclcpp::Node::make_shared("test_odom_pub");
  auto odom_pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(
    "/" + ns + "/sensor_measurements/odom", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_odom_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  const std::string map_frame = ns + "/map";

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = map_frame;
  odom.child_frame_id = ns + "/base_link";
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
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }

  ASSERT_TRUE(tf_available) << "earth→map TF should be available after the first odometry message";

  // Identity rotation, so the inverse of (1,2,3) is simply (-1,-2,-3).
  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_map.transform.translation.x, -1.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.y, -2.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, -3.0, 1e-6);
}

TEST(SimpleEkfIntegrationTest, PoseWithCovariance_SetsEarthToMapAsInverseOfPose)
{
  const std::string ns = "test_pose_cov";
  auto node = getSimpleEkfNode(
    ns, {
    "simple_ekf.update_topics:=[pose_cov]",
    "simple_ekf.pose_cov.topic:=self_localization/pose_cov",
    "simple_ekf.pose_cov.type:=geometry_msgs/msg/PoseWithCovarianceStamped",
    "simple_ekf.pose_cov.set_earth_map:=true",
    // use_message_covariance=true exercises the multiplier branch of getCovarianceWithConfig
    "simple_ekf.pose_cov.use_message_covariance:=true",
  });
  auto pub_node = rclcpp::Node::make_shared("test_pose_cov_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/" + ns + "/self_localization/pose_cov", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_pose_cov_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  const std::string map_frame = ns + "/map";

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = map_frame;
  pose.pose.pose.position.x = 4.0;
  pose.pose.pose.position.y = 5.0;
  pose.pose.pose.position.z = 6.0;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance.fill(1e-4);

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }

  ASSERT_TRUE(tf_available) << "earth→map TF should be available after the first pose message";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_map.transform.translation.x, -4.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.y, -5.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, -6.0, 1e-6);
}

// A first pose already expressed in the earth frame is taken verbatim as earth→map,
// unlike a map-frame pose which is inverted (setEarthToMapFromFirstPose's other branch).
TEST(SimpleEkfIntegrationTest, PoseStamped_EarthFrame_SetsEarthToMapVerbatim)
{
  const std::string ns = "test_pose_earth";
  auto node = getSimpleEkfNode(ns);
  auto pub_node = rclcpp::Node::make_shared("test_pose_earth_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/ground_truth/pose", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_pose_earth_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  const std::string map_frame = ns + "/map";

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.position.x = 7.0;
  pose.pose.position.y = 8.0;
  pose.pose.position.z = 9.0;
  pose.pose.orientation.w = 1.0;

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }

  ASSERT_TRUE(tf_available) << "earth→map TF should be available after the first pose message";

  auto earth_to_map = tf_buffer->lookupTransform("earth", map_frame, tf2::TimePointZero);
  EXPECT_NEAR(earth_to_map.transform.translation.x, 7.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.y, 8.0, 1e-6);
  EXPECT_NEAR(earth_to_map.transform.translation.z, 9.0, 1e-6);
}

// setEarthToMapFromFirstPose only accepts the earth and map frames; anything else is refused.
TEST(SimpleEkfIntegrationTest, PoseStamped_UnknownFrame_EarthToMapNeverSet)
{
  const std::string ns = "test_pose_bad_frame";
  auto node = getSimpleEkfNode(ns);
  auto pub_node = rclcpp::Node::make_shared("test_pose_bad_frame_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/ground_truth/pose", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("test_pose_bad_frame_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = ns + "/odom";  // neither earth nor map
  pose.pose.position.x = 1.0;
  pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 5; ++i) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 5);
  }

  EXPECT_FALSE(tf_buffer->canTransform("earth", ns + "/map", tf2::TimePointZero)) <<
    "a pose in an unsupported frame must not establish earth→map";
}

TEST(SimpleEkfIntegrationTest, PlatformInfoOffboardFalse_NodeDoesNotCrash)
{
  auto node = getSimpleEkfNode("test_offboard");
  auto pub_node = rclcpp::Node::make_shared("test_offboard_pub");

  auto info_pub = pub_node->create_publisher<as2_msgs::msg::PlatformInfo>(
    "/test_offboard/platform/info", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  as2_msgs::msg::PlatformInfo info;
  info.offboard = false;

  EXPECT_NO_THROW(
  {
    info_pub->publish(info);
    // Spin long enough for the timer callback to fire (timer_hz=100 → 10ms per tick)
    spinSome(exec, 20);
  });
}

// ---------------------------------------------------------------------------
// Pre-offboard zero-pose correction and its one-way latch
//
// While the drone has never been offboard, timerCallback() corrects the EKF to (0,0,0) on
// every tick (timer_hz=100) and publishes the result. Each of those publishes emits one
// message on the plugin debug topic, so the *rate* of that topic is a direct, numeric-free
// observable of whether the correction is running.
//
// Note on counting: spin_some() takes at most one message per subscription per call, so the
// observed count saturates at the number of spinSome() iterations. That is fine here — the
// assertions only need to separate "a message nearly every iteration" from "almost none".
// ---------------------------------------------------------------------------

// Publish a first pose in the earth frame so that earth→map gets established (the correction
// is gated on earth_to_map_set_), then return once the plugin has provably processed it.
static bool establishEarthToMap(
  rclcpp::executors::MultiThreadedExecutor & exec,
  const rclcpp::Node::SharedPtr & pub_node,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pose_pub,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const std::string & map_frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.orientation.w = 1.0;

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero);
  }
  return tf_available;
}

TEST(SimpleEkfIntegrationTest, OffboardLatch_ZeroPoseCorrectionStopsForeverAfterFirstOffboard)
{
  const std::string ns = "test_offboard_latch";
  auto node = getSimpleEkfNode(ns);
  auto pub_node = rclcpp::Node::make_shared("test_offboard_latch_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/ground_truth/pose", rclcpp::SensorDataQoS());
  auto info_pub = pub_node->create_publisher<as2_msgs::msg::PlatformInfo>(
    "/" + ns + "/platform/info", rclcpp::SensorDataQoS());

  auto sub_node = rclcpp::Node::make_shared("test_offboard_latch_sub");
  int debug_pose_count = 0;
  auto debug_sub = sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/state_estimation/simple_ekf/pose", 10,
    [&debug_pose_count](geometry_msgs::msg::PoseStamped::SharedPtr) {debug_pose_count++;});

  auto tf_node = rclcpp::Node::make_shared("test_offboard_latch_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(sub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  ASSERT_TRUE(establishEarthToMap(exec, pub_node, pose_pub, tf_buffer, ns + "/map"));

  // Phase 1: never offboard — the 100 Hz correction should be running.
  constexpr int kIterations = 10;
  debug_pose_count = 0;
  spinSome(exec, kIterations);
  const int count_before_offboard = debug_pose_count;
  EXPECT_GE(count_before_offboard, kIterations / 2) <<
    "pre-offboard zero-pose correction should publish continuously";

  // Phase 2: go offboard once, then drop back out of offboard (e.g. after landing).
  as2_msgs::msg::PlatformInfo info;
  info.offboard = true;
  for (int i = 0; i < 5; ++i) {
    info_pub->publish(info);
    spinSome(exec, 2);
  }
  info.offboard = false;
  for (int i = 0; i < 5; ++i) {
    info_pub->publish(info);
    spinSome(exec, 2);
  }

  // Phase 3: the correction must stay off, even though offboard is false again.
  debug_pose_count = 0;
  spinSome(exec, kIterations);
  EXPECT_LE(debug_pose_count, 3) <<
    "once the drone has been offboard, the zero-pose correction must never come back "
    "(saw " << debug_pose_count << " publishes)";
}

// use_arm=true makes the reset logic follow PlatformInfo.armed instead of .offboard.
TEST(SimpleEkfIntegrationTest, UseArm_ArmedStopsZeroPoseCorrection)
{
  const std::string ns = "test_use_arm";
  auto node = getSimpleEkfNode(ns, {"simple_ekf.use_arm:=true"});
  auto pub_node = rclcpp::Node::make_shared("test_use_arm_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/ground_truth/pose", rclcpp::SensorDataQoS());
  auto info_pub = pub_node->create_publisher<as2_msgs::msg::PlatformInfo>(
    "/" + ns + "/platform/info", rclcpp::SensorDataQoS());

  auto sub_node = rclcpp::Node::make_shared("test_use_arm_sub");
  int debug_pose_count = 0;
  auto debug_sub = sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/state_estimation/simple_ekf/pose", 10,
    [&debug_pose_count](geometry_msgs::msg::PoseStamped::SharedPtr) {debug_pose_count++;});

  auto tf_node = rclcpp::Node::make_shared("test_use_arm_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(sub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  ASSERT_TRUE(establishEarthToMap(exec, pub_node, pose_pub, tf_buffer, ns + "/map"));

  constexpr int kIterations = 10;
  debug_pose_count = 0;
  spinSome(exec, kIterations);
  EXPECT_GE(debug_pose_count, kIterations / 2) <<
    "zero-pose correction should run while disarmed";

  // armed=true with offboard=false: only the armed flag may be consulted when use_arm=true.
  as2_msgs::msg::PlatformInfo info;
  info.armed = true;
  info.offboard = false;
  for (int i = 0; i < 5; ++i) {
    info_pub->publish(info);
    spinSome(exec, 2);
  }

  debug_pose_count = 0;
  spinSome(exec, kIterations);
  EXPECT_LE(debug_pose_count, 3) <<
    "with use_arm=true, armed=true must stop the zero-pose correction even though "
    "offboard is false (saw " << debug_pose_count << " publishes)";
}

// ---------------------------------------------------------------------------
// update_rate_hz throttling
// ---------------------------------------------------------------------------

// Publish `n_poses` poses ~100 ms apart and count how many reach the EKF, observed via the
// plugin debug topic. Offboard is asserted first so the 100 Hz zero-pose correction is off
// and every debug message can only come from an accepted pose update.
static int countAcceptedPoseUpdates(
  const std::string & ns,
  const std::vector<std::string> & overrides,
  int n_poses)
{
  auto node = getSimpleEkfNode(ns, overrides);
  auto pub_node = rclcpp::Node::make_shared(ns + "_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/ground_truth/pose", rclcpp::SensorDataQoS());
  auto info_pub = pub_node->create_publisher<as2_msgs::msg::PlatformInfo>(
    "/" + ns + "/platform/info", rclcpp::SensorDataQoS());

  auto sub_node = rclcpp::Node::make_shared(ns + "_sub");
  int debug_pose_count = 0;
  auto debug_sub = sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + ns + "/state_estimation/simple_ekf/pose", 10,
    [&debug_pose_count](geometry_msgs::msg::PoseStamped::SharedPtr) {debug_pose_count++;});

  auto tf_node = rclcpp::Node::make_shared(ns + "_tf_listener");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our own executor below, so the listener must not
  // also spin it on an internal background thread (double-spinning the same node crashes).
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(sub_node);
  exec.add_node(tf_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  as2_msgs::msg::PlatformInfo info;
  info.offboard = true;
  for (int i = 0; i < 5; ++i) {
    info_pub->publish(info);
    spinSome(exec, 2);
  }

  if (!establishEarthToMap(exec, pub_node, pose_pub, tf_buffer, ns + "/map")) {
    return -1;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.orientation.w = 1.0;

  debug_pose_count = 0;
  for (int i = 0; i < n_poses; ++i) {
    pose.pose.position.x = 0.01 * i;
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 2);  // ~100 ms between poses
  }
  spinSome(exec, 4);
  return debug_pose_count;
}

// Same message rate, with and without update_rate_hz: the capped run must let through
// materially fewer updates.
TEST(SimpleEkfIntegrationTest, UpdateRateHz_ThrottlesPoseUpdates)
{
  constexpr int kPoses = 20;  // published at ~10 Hz, so ~2 s of data

  const int unthrottled = countAcceptedPoseUpdates("test_no_throttle", {}, kPoses);
  ASSERT_GE(unthrottled, 0) << "earth→map should have been established (unthrottled run)";

  const int throttled = countAcceptedPoseUpdates(
    "test_throttle", {"simple_ekf.gazebo.update_rate_hz:=2.0"}, kPoses);
  ASSERT_GE(throttled, 0) << "earth→map should have been established (throttled run)";

  EXPECT_GT(unthrottled, 12) <<
    "without a cap, nearly every published pose should reach the EKF";
  EXPECT_GT(throttled, 0) << "throttling must not block updates entirely";
  EXPECT_LT(throttled, unthrottled / 2) <<
    "a 2 Hz cap on a ~10 Hz stream should drop most updates "
    "(throttled=" << throttled << ", unthrottled=" << unthrottled << ")";
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
