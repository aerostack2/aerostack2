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
 * @file ground_truth_integration_gtest.cpp
 *
 * Integration tests for the ground_truth plugin loaded into a real
 * StateEstimator ROS2 node. Tests verify that the plugin loads, processes
 * messages (both the mocap and plain-pose paths), and produces TF/twist
 * output matching its documented behavior.
 */

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helper: create a StateEstimator node with the ground_truth plugin
// ---------------------------------------------------------------------------

static std::shared_ptr<as2_state_estimator::StateEstimator> getGroundTruthNode(
  const std::string & node_name_prefix,
  const std::vector<std::string> & extra_param_overrides = {})
{
  const std::string ns = node_name_prefix;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_cfg = package_path + "/config/state_estimator_default.yaml";
  const std::string plugin_cfg =
    package_path + "/plugins/ground_truth/config/plugin_default.yaml";

  std::vector<std::string> args = {
    "--ros-args",
    "-r", "__ns:=/" + ns,
    "-p", "namespace:=" + ns,
    "-p", "plugin_name:=ground_truth",
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

TEST(GroundTruthIntegrationTest, PluginLoads_NoThrow)
{
  EXPECT_NO_THROW(getGroundTruthNode("gt_load"));
}

TEST(GroundTruthIntegrationTest, PluginSpins_NoThrow)
{
  auto node = getGroundTruthNode("gt_spin");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  EXPECT_NO_THROW(spinSome(exec, 5));
}

// Default config has mocap_sub_topic non-empty ("/mocap/rigid_bodies"), so by default
// ground_truth subscribes to mocap instead of pose_sub_topic. This is an absolute
// (non-namespaced) topic per the default config's leading slash.
TEST(GroundTruthIntegrationTest, MocapZeroPose_Rejected_NodeDoesNotCrash)
{
  auto node = getGroundTruthNode("gt_mocap_zero");
  auto pub_node = rclcpp::Node::make_shared("gt_mocap_zero_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/mocap/rigid_bodies", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBodies msg;
  msg.header.stamp = pub_node->now();
  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  // All-zero pose: sensor-not-detected sentinel — should be skipped, not crash
  body.pose.position.x = 0.0;
  body.pose.position.y = 0.0;
  body.pose.position.z = 0.0;
  body.pose.orientation.x = 0.0;
  body.pose.orientation.y = 0.0;
  body.pose.orientation.z = 0.0;
  body.pose.orientation.w = 0.0;
  msg.rigidbodies.push_back(body);

  EXPECT_NO_THROW({
    mocap_pub->publish(msg);
    spinSome(exec, 10);
  });
}

TEST(GroundTruthIntegrationTest, MocapNonMatchingName_NodeDoesNotCrash)
{
  auto node = getGroundTruthNode("gt_mocap_wrong_name");
  auto pub_node = rclcpp::Node::make_shared("gt_mocap_wrong_name_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/mocap/rigid_bodies", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBodies msg;
  msg.header.stamp = pub_node->now();
  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "99";  // does not match configured rigid_body_name "33"
  body.pose.position.x = 1.0;
  body.pose.position.y = 2.0;
  body.pose.position.z = 3.0;
  body.pose.orientation.w = 1.0;
  msg.rigidbodies.push_back(body);

  EXPECT_NO_THROW({
    mocap_pub->publish(msg);
    spinSome(exec, 10);
  });
}

TEST(GroundTruthIntegrationTest, MocapValidPose_SetsEarthToMap_TfAvailable)
{
  auto node = getGroundTruthNode("gt_mocap_valid");
  auto pub_node = rclcpp::Node::make_shared("gt_mocap_valid_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/mocap/rigid_bodies", 10);

  auto tf_node = rclcpp::Node::make_shared("gt_mocap_valid_tf_listener");
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

  const std::string base_frame = "gt_mocap_valid/base_link";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    mocap4r2_msgs::msg::RigidBodies msg;
    msg.header.stamp = pub_node->now();
    msg.rigidbodies.push_back(body);
    mocap_pub->publish(msg);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) <<
    "earth->base_link TF chain should be available after a valid mocap pose";
}

TEST(GroundTruthIntegrationTest, MocapDuplicatePose_Rejected_NodeDoesNotCrash)
{
  auto node = getGroundTruthNode("gt_mocap_dup");
  auto pub_node = rclcpp::Node::make_shared("gt_mocap_dup_pub");
  auto mocap_pub = pub_node->create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "/mocap/rigid_bodies", 10);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  mocap4r2_msgs::msg::RigidBodies msg;
  msg.header.stamp = pub_node->now();
  mocap4r2_msgs::msg::RigidBody body;
  body.rigid_body_name = "33";
  body.pose.position.x = 1.0;
  body.pose.position.y = 2.0;
  body.pose.position.z = 3.0;
  body.pose.orientation.w = 1.0;
  msg.rigidbodies.push_back(body);

  EXPECT_NO_THROW({
    mocap_pub->publish(msg);
    spinSome(exec, 5);
    // Publish the identical message a second time — should be silently rejected
    mocap_pub->publish(msg);
    spinSome(exec, 5);
  });
}

TEST(GroundTruthIntegrationTest, PlainPose_SetsEarthToMap_TfAvailable)
{
  auto node = getGroundTruthNode("gt_plain_pose", {"ground_truth.mocap_sub_topic:=''"});
  auto pub_node = rclcpp::Node::make_shared("gt_plain_pose_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gt_plain_pose/ground_truth/pose", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("gt_plain_pose_tf_listener");
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
  pose.header.frame_id = "earth";  // ground_truth requires the global earth frame, unnamespaced
  // Non-zero position: last_pose_ defaults to (0,0,0), so a (0,0,0) pose would collide with
  // the duplicate-pose check on the very first message and be silently skipped.
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.position.z = 3.0;
  pose.pose.orientation.w = 1.0;

  const std::string base_frame = "gt_plain_pose/base_link";

  bool tf_available = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!tf_available && pub_node->now() < deadline) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 5);
    tf_available = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }

  EXPECT_TRUE(tf_available) <<
    "earth->base_link TF chain should be available after publishing a valid pose";
}

TEST(GroundTruthIntegrationTest, PlainPose_WrongFrameId_Rejected_NodeDoesNotCrash)
{
  auto node = getGroundTruthNode("gt_plain_pose_wrong_frame", {"ground_truth.mocap_sub_topic:=''"});
  auto pub_node = rclcpp::Node::make_shared("gt_plain_pose_wrong_frame_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gt_plain_pose_wrong_frame/ground_truth/pose", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared("gt_plain_pose_wrong_frame_tf_listener");
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
  pose.header.frame_id = "map";  // wrong: ground_truth expects "earth"
  pose.pose.orientation.w = 1.0;

  const std::string base_frame = "gt_plain_pose_wrong_frame/base_link";

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      pose.header.stamp = pub_node->now();
      pose_pub->publish(pose);
      spinSome(exec, 5);
    }
  });

  EXPECT_FALSE(tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero)) <<
    "TF chain must stay unavailable when the pose frame_id does not match the earth frame";
}

TEST(GroundTruthIntegrationTest, PlainPose_DuplicatePose_Rejected_NodeDoesNotCrash)
{
  auto node = getGroundTruthNode("gt_plain_pose_dup", {"ground_truth.mocap_sub_topic:=''"});
  auto pub_node = rclcpp::Node::make_shared("gt_plain_pose_dup_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gt_plain_pose_dup/ground_truth/pose", rclcpp::SensorDataQoS());

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  // Give the StateEstimator's 1s deferred setup() timer time to fire (plugin subscriptions
  // aren't created until then) before publishing anything.
  spinSome(exec, 30);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  // Non-zero position: last_pose_ defaults to (0,0,0), so a (0,0,0) pose would already look
  // like a duplicate on the very first message.
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.position.z = 3.0;
  pose.pose.orientation.w = 1.0;
  pose.header.stamp = pub_node->now();

  EXPECT_NO_THROW({
    pose_pub->publish(pose);
    spinSome(exec, 5);
    // Publish the identical pose a second time — should be silently rejected
    pose_pub->publish(pose);
    spinSome(exec, 5);
  });
}

// twist_smooth_filter_cte defaults to 1.0 (no filtering), so with identity orientation the
// computed twist should exactly match (pose_b - pose_a) / dt in earth-frame axes.
TEST(GroundTruthIntegrationTest, PlainPose_TwistComputedFromPoseDifferentiation)
{
  auto node = getGroundTruthNode("gt_plain_pose_twist", {"ground_truth.mocap_sub_topic:=''"});
  auto pub_node = rclcpp::Node::make_shared("gt_plain_pose_twist_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/gt_plain_pose_twist/ground_truth/pose", rclcpp::SensorDataQoS());

  auto sub_node = rclcpp::Node::make_shared("gt_plain_pose_twist_sub");
  geometry_msgs::msg::TwistStamped::SharedPtr last_twist;
  auto twist_sub = sub_node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/gt_plain_pose_twist/self_localization/twist", rclcpp::SensorDataQoS(),
    [&last_twist](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      last_twist = msg;
    });

  auto tf_node = rclcpp::Node::make_shared("gt_plain_pose_twist_tf_listener");
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

  const double dt = 0.5;
  const std::string base_frame = "gt_plain_pose_twist/base_link";

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.orientation.w = 1.0;
  // Non-zero starting position: last_pose_ defaults to (0,0,0), so a (0,0,0) first pose would
  // collide with the duplicate-pose check and never reach the twist computation at all.
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 5.0;
  pose.pose.position.z = 5.0;
  // Keep the stamp fixed across retries so that whichever attempt is actually accepted by the
  // plugin, its internal last_time_ is known precisely (needed to compute pose B's dt below).
  const rclcpp::Time a_stamp = pub_node->now();
  pose.header.stamp = a_stamp;

  // A freshly-created subscription can lose the discovery race against the very first publish
  // (dropped under best-effort QoS), so retry until the plugin has provably processed it —
  // observable externally via the earth->map TF it establishes on the first accepted pose.
  bool a_processed = false;
  auto a_deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!a_processed && pub_node->now() < a_deadline) {
    pose_pub->publish(pose);
    spinSome(exec, 5);
    a_processed = tf_buffer->canTransform(base_frame, "earth", tf2::TimePointZero);
  }
  ASSERT_TRUE(a_processed) << "first pose should be processed (earth->map established)";

  // The first pose only initializes the differentiator and publishes a zero twist; reset the
  // capture so we only observe the twist computed from the second (moved) pose below.
  last_twist = nullptr;

  pose.pose.position.x = 6.0;
  pose.header.stamp = a_stamp + rclcpp::Duration::from_seconds(dt);

  bool twist_received = false;
  auto deadline = pub_node->now() + rclcpp::Duration(2, 0);
  while (!twist_received && pub_node->now() < deadline) {
    pose_pub->publish(pose);
    spinSome(exec, 5);
    twist_received = (last_twist != nullptr);
  }

  ASSERT_TRUE(twist_received) << "self_localization/twist should receive a message";
  EXPECT_NEAR(last_twist->twist.linear.x, 1.0 / dt, 0.05);
  EXPECT_NEAR(last_twist->twist.linear.y, 0.0, 0.05);
  EXPECT_NEAR(last_twist->twist.linear.z, 0.0, 0.05);
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
