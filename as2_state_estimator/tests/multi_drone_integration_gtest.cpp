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
* @file multi_drone_integration_gtest.cpp
*
* Multi-drone (drone0, drone1, drone2, ...) tests for the state estimator core.
*
* WHAT THIS FILE COVERS, AND WHY IT IS SHAPED THIS WAY
* ----------------------------------------------------
* In a fleet, every drone runs its own state_estimator in its own ROS namespace. The
* correctness question is entirely about *isolation*: drone1's frames, topics and TF tree
* must be its own, and nothing drone1 publishes may perturb drone0.
*
* These tests instantiate ONE StateEstimator at a time, in namespaces drone0/drone1/drone2,
* and assert that everything it exposes is correctly scoped to that namespace. They do NOT
* run two StateEstimator instances concurrently, and that is deliberate rather than a
* shortcut — StateEstimator keeps its identity in process-wide statics:
*
*   as2_state_estimator.hpp:88   static StateEstimator::SharedPtr instance_;
*   as2_state_estimator.hpp:143  static std::string earth_frame_id_;
*   as2_state_estimator.hpp:144  static std::string base_frame_id_;
*   as2_state_estimator.hpp:145  static std::string odom_frame_id_;
*   as2_state_estimator.hpp:146  static std::string map_frame_id_;
*   as2_state_estimator.hpp:147  static RobotState robot_state_;
*
* readParameters() writes the four frame ids during setup(), and every plugin reads them
* back through PluginWrapperInterface::getXFrame(). One copy exists per PROCESS, not per
* node. Two instances in one process therefore share a single set of frame names and a
* single robot state, and PluginWrapper::create() binds each plugin to whichever node last
* assigned instance_. This is fine in production, where each drone is a separate
* `as2_state_estimator_node` process, and it is what MultipleInstancesShareProcessWideState
* below pins so the constraint is visible rather than folklore.
*
* Consequence: concurrent-instance behaviour cannot be covered from a gtest, because the
* fixture and the subject would be sharing the same statics. That needs a launch_testing
* test driving two real node processes. See the note on that test.
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
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// The drone namespaces under test. Deliberately the real fleet naming (drone0, drone1,
// drone2) rather than test-shaped names: the frame strings these produce are exactly what
// a three-drone launch produces, so a regression in namespacing shows up as a diff a
// reader recognises.
static const char * kDroneNamespaces[] = {"drone0", "drone1", "drone2"};

static std::shared_ptr<as2_state_estimator::StateEstimator> getStateEstimatorNode(
  const std::string & ns,
  const std::string & plugin_name = "ground_truth",
  const std::vector<std::string> & extra_param_overrides = {})
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_cfg = package_path + "/config/state_estimator_default.yaml";
  const std::string plugin_cfg =
    package_path + "/plugins/" + plugin_name + "/config/plugin_default.yaml";

  std::vector<std::string> args = {
    "--ros-args",
    "-r", "__ns:=/" + ns,
    "-p", "namespace:=" + ns,
    "-p", "plugin_name:=" + plugin_name,
    "--params-file", state_estimator_cfg,
    "--params-file", plugin_cfg,
  };
  for (const auto & override_arg : extra_param_overrides) {
    args.push_back("-p");
    args.push_back(override_arg);
  }

  auto opts = rclcpp::NodeOptions().arguments(args);
  auto node = std::make_shared<as2_state_estimator::StateEstimator>(opts);
  // PluginWrapper::create() reaches for StateEstimator::getInstance(), so a directly
  // constructed node has to register itself as the singleton for the deferred setup() to
  // find it. See the file header: this is the same global that makes concurrent instances
  // impossible in-process.
  as2_state_estimator::StateEstimator::instance_ = node;
  return node;
}

// spin_some(duration) returns almost immediately when nothing is ready, so each iteration
// needs a real sleep to let StateEstimator's 1 s deferred setup() timer mature.
static void spinSome(rclcpp::executors::MultiThreadedExecutor & exec, int iterations = 10)
{
  for (int i = 0; i < iterations; ++i) {
    exec.spin_some(50ms);
    std::this_thread::sleep_for(50ms);
  }
}

// Spin until setup() has run, detected by the base frame having been rewritten to this
// drone's namespaced value. Polling instead of a fixed spinSome(30) keeps the suite fast
// (setup lands at ~1 s, not 3 s) while still failing honestly on timeout.
//
// Safe against the frame statics leaking between tests precisely BECAUSE the expected
// value is namespace-specific: every test uses a distinct namespace, so a stale value left
// by a previous test can never satisfy this predicate.
static bool waitForSetup(
  rclcpp::executors::MultiThreadedExecutor & exec,
  const std::string & expected_base_frame,
  int max_iterations = 40)
{
  for (int i = 0; i < max_iterations; ++i) {
    if (as2_state_estimator::StateEstimator::getBaseFrame() == expected_base_frame) {
      return true;
    }
    exec.spin_some(50ms);
    std::this_thread::sleep_for(50ms);
  }
  return as2_state_estimator::StateEstimator::getBaseFrame() == expected_base_frame;
}

// Publish a pose on <ns>/ground_truth/pose until earth -> <ns>/map shows up in TF.
//
// x defaults to 1.0, not 0.0, on purpose: ground_truth's last_pose_ starts at the origin
// and it drops poses identical to the previous one, so a (0,0,0) first pose is silently
// skipped and earth->map is never set.
static bool establishEarthToMap(
  rclcpp::executors::MultiThreadedExecutor & exec,
  const rclcpp::Node::SharedPtr & pub_node,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pose_pub,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const std::string & map_frame,
  double x = 1.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = x;

  for (int i = 0; i < 30; ++i) {
    pose.header.stamp = pub_node->now();
    pose_pub->publish(pose);
    spinSome(exec, 2);
    if (tf_buffer->canTransform("earth", map_frame, tf2::TimePointZero)) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Frame namespacing
// ---------------------------------------------------------------------------

// The core multi-drone contract: map/odom/base_link are per-drone, earth is shared.
//
// earth NOT being namespaced is the load-bearing half. It is the common frame the whole
// fleet is expressed in, so if it ever became "drone0/earth" the drones would each sit in
// their own disconnected TF tree and no inter-drone transform would resolve — while every
// single-drone test in this package kept passing.
TEST(MultiDroneTest, FramesAreNamespacedPerDrone)
{
  for (const auto * ns : kDroneNamespaces) {
    const std::string drone(ns);
    auto node = getStateEstimatorNode(drone + "_frames");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    ASSERT_TRUE(waitForSetup(exec, drone + "_frames/base_link"))
      << "setup() did not run for " << drone;

    EXPECT_EQ(as2_state_estimator::StateEstimator::getMapFrame(), drone + "_frames/map");
    EXPECT_EQ(as2_state_estimator::StateEstimator::getOdomFrame(), drone + "_frames/odom");
    EXPECT_EQ(as2_state_estimator::StateEstimator::getBaseFrame(), drone + "_frames/base_link");
    EXPECT_EQ(as2_state_estimator::StateEstimator::getEarthFrame(), "earth")
      << "earth must stay global — namespacing it would split the fleet into "
      << "disconnected TF trees";
  }
}

// base_frame: "" resolves the base frame to the namespace itself ("drone0" rather than
// "drone0/base_link"), which is what a fleet sharing one URDF per drone needs.
// as2_core's tf2_namespace_test covers generateTfName() in isolation; this covers the
// state estimator actually wiring it through to the frame the plugins publish into.
TEST(MultiDroneTest, EmptyBaseFrameResolvesToDroneNamespace)
{
  for (const auto * ns : kDroneNamespaces) {
    const std::string drone = std::string(ns) + "_emptybase";
    auto node = getStateEstimatorNode(drone, "ground_truth", {"base_frame:=''"});

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    ASSERT_TRUE(waitForSetup(exec, drone)) << "setup() did not run for " << drone;

    EXPECT_EQ(as2_state_estimator::StateEstimator::getBaseFrame(), drone);
    // The other frames must be unaffected by the empty base_frame.
    EXPECT_EQ(as2_state_estimator::StateEstimator::getMapFrame(), drone + "/map");
    EXPECT_EQ(as2_state_estimator::StateEstimator::getOdomFrame(), drone + "/odom");
  }
}

// Every supported plugin must namespace identically — the frame ids come from the shared
// core, so a plugin that cached or rebuilt them itself would show up here.
class MultiDronePluginTest : public ::testing::TestWithParam<std::string> {};

TEST_P(MultiDronePluginTest, FramesNamespacedRegardlessOfPlugin)
{
  const std::string plugin = GetParam();
  const std::string drone = "drone0_" + plugin;
  auto node = getStateEstimatorNode(drone, plugin);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  ASSERT_TRUE(waitForSetup(exec, drone + "/base_link"))
    << "setup() did not run for plugin " << plugin;

  EXPECT_EQ(as2_state_estimator::StateEstimator::getMapFrame(), drone + "/map");
  EXPECT_EQ(as2_state_estimator::StateEstimator::getOdomFrame(), drone + "/odom");
  EXPECT_EQ(as2_state_estimator::StateEstimator::getEarthFrame(), "earth");
}

INSTANTIATE_TEST_SUITE_P(
  AllPlugins, MultiDronePluginTest,
  ::testing::Values("ground_truth", "raw_odometry", "simple_ekf"));

// ---------------------------------------------------------------------------
// Topic namespacing
// ---------------------------------------------------------------------------

// The published state topics must live under the drone's namespace. If these were absolute,
// every drone in the fleet would publish its pose onto one shared topic and the last writer
// would win — the single most damaging way multi-drone can fail.
TEST(MultiDroneTest, StateTopicsAreNamespacedPerDrone)
{
  for (const auto * ns : kDroneNamespaces) {
    const std::string drone = std::string(ns) + "_topics";
    auto node = getStateEstimatorNode(drone);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    ASSERT_TRUE(waitForSetup(exec, drone + "/base_link"));

    auto observer = rclcpp::Node::make_shared(drone + "_observer");
    exec.add_node(observer);
    spinSome(exec, 4);

    const auto topics = observer->get_topic_names_and_types();
    EXPECT_TRUE(topics.count("/" + drone + "/self_localization/pose"))
      << "self_localization/pose must be namespaced under " << drone;
    EXPECT_TRUE(topics.count("/" + drone + "/self_localization/twist"))
      << "self_localization/twist must be namespaced under " << drone;

    // And must NOT exist at the root, which is what an absolute topic name would produce.
    EXPECT_FALSE(topics.count("/self_localization/pose"))
      << "a root-level self_localization/pose means the topic escaped its namespace and "
      "every drone in the fleet would collide on it";
  }
}

// ---------------------------------------------------------------------------
// TF isolation
// ---------------------------------------------------------------------------

// Each drone's TF chain hangs off the shared earth frame: earth -> <ns>/map -> <ns>/odom
// -> <ns>/base_link. Assert the whole chain resolves, and that a *sibling* drone's frames
// do not appear as a side effect.
TEST(MultiDroneTest, TfChainIsCompleteAndScopedToTheDrone)
{
  const std::string drone = "drone1_tfchain";
  const std::string other = "drone2_tfchain";

  auto node = getStateEstimatorNode(drone);
  auto pub_node = rclcpp::Node::make_shared(drone + "_pub");
  auto pose_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + drone + "/ground_truth/pose", rclcpp::SensorDataQoS());

  auto tf_node = rclcpp::Node::make_shared(drone + "_tf");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  // spin_thread=false: tf_node is spun by our executor, and double-spinning a node crashes.
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(tf_node);
  ASSERT_TRUE(waitForSetup(exec, drone + "/base_link"));

  ASSERT_TRUE(establishEarthToMap(exec, pub_node, pose_pub, tf_buffer, drone + "/map"))
    << "earth -> " << drone << "/map was never published";
  spinSome(exec, 10);

  EXPECT_TRUE(tf_buffer->canTransform("earth", drone + "/map", tf2::TimePointZero));
  EXPECT_TRUE(tf_buffer->canTransform(drone + "/map", drone + "/odom", tf2::TimePointZero));
  EXPECT_TRUE(tf_buffer->canTransform(drone + "/odom", drone + "/base_link", tf2::TimePointZero));
  // The full chain, which is what a consumer (a behavior, a controller) actually asks for.
  EXPECT_TRUE(tf_buffer->canTransform("earth", drone + "/base_link", tf2::TimePointZero));

  // No frames belonging to a drone that was never instantiated.
  EXPECT_FALSE(tf_buffer->canTransform("earth", other + "/map", tf2::TimePointZero))
    << other << " frames must not appear — only " << drone << " is running";
  EXPECT_FALSE(tf_buffer->canTransform("earth", other + "/base_link", tf2::TimePointZero));
}

// ---------------------------------------------------------------------------
// Cross-drone isolation
// ---------------------------------------------------------------------------

// A measurement addressed to drone2 must not move drone1. This is the fleet-level
// cross-talk check, and it is meaningful in-process because only ONE estimator exists —
// the other namespace is just a topic nobody is subscribed to.
TEST(MultiDroneTest, MeasurementForAnotherDroneIsIgnored)
{
  const std::string drone = "drone1_xtalk";
  const std::string other = "drone2_xtalk";

  // twist_sub_topic:='' makes ground_truth differentiate the pose for twist. Without it no
  // twist is ever produced, and self_localization/pose is only published from publishTwist(),
  // so the observable this test reads would never appear at all. This is the shipped
  // default, but it is set explicitly here so the test does not silently depend on it.
  auto node = getStateEstimatorNode(drone, "ground_truth", {"ground_truth.twist_sub_topic:=''"});
  auto pub_node = rclcpp::Node::make_shared(drone + "_xtalk_pub");
  auto own_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + drone + "/ground_truth/pose", rclcpp::SensorDataQoS());
  auto other_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/" + other + "/ground_truth/pose", rclcpp::SensorDataQoS());

  geometry_msgs::msg::PoseStamped::SharedPtr last_pose;
  auto sub_node = rclcpp::Node::make_shared(drone + "_xtalk_sub");
  auto pose_sub = sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/" + drone + "/self_localization/pose", rclcpp::SensorDataQoS(),
    [&last_pose](geometry_msgs::msg::PoseStamped::SharedPtr msg) {last_pose = msg;});

  auto tf_node = rclcpp::Node::make_shared(drone + "_xtalk_tf");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, false);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(pub_node);
  exec.add_node(sub_node);
  exec.add_node(tf_node);
  ASSERT_TRUE(waitForSetup(exec, drone + "/base_link"));

  // Anchor drone1 through its OWN topic. A short ramp of DISTINCT poses, because
  // ground_truth drops a pose identical to the previous one and twist differentiation
  // needs two samples to produce anything.
  ASSERT_TRUE(establishEarthToMap(exec, pub_node, own_pub, tf_buffer, drone + "/map", 0.5));
  geometry_msgs::msg::PoseStamped own;
  own.header.frame_id = "earth";
  own.pose.orientation.w = 1.0;
  for (int i = 1; i <= 10; ++i) {
    own.header.stamp = pub_node->now();
    own.pose.position.x = 0.5 + 0.05 * i;   // ends at 1.0
    own_pub->publish(own);
    spinSome(exec, 1);
  }
  // Let every in-flight message land before sampling. Without this, the baseline is taken
  // mid-ramp and the last ramp message arriving afterwards looks exactly like cross-talk.
  spinSome(exec, 10);
  ASSERT_NE(last_pose, nullptr) << "drone1 never published its own state";
  const double x_before = last_pose->pose.position.x;
  EXPECT_NEAR(x_before, 1.0, 1e-3) << "drone1 should have taken its own measurement";

  // Now hammer drone2's topic with a wildly different pose.
  geometry_msgs::msg::PoseStamped foreign;
  foreign.header.frame_id = "earth";
  foreign.pose.orientation.w = 1.0;
  foreign.pose.position.x = 99.0;
  foreign.pose.position.y = -55.0;
  for (int i = 0; i < 20; ++i) {
    foreign.header.stamp = pub_node->now();
    other_pub->publish(foreign);
    spinSome(exec, 1);
  }

  // drone1 must be exactly where it was.
  EXPECT_NEAR(last_pose->pose.position.x, x_before, 1e-6)
    << "a pose published on " << other << "'s topic moved " << drone;
  EXPECT_NEAR(last_pose->pose.position.y, 0.0, 1e-6)
    << "a pose published on " << other << "'s topic moved " << drone;
  EXPECT_FALSE(tf_buffer->canTransform("earth", other + "/map", tf2::TimePointZero))
    << other << " frames must not be created by " << drone << "'s estimator";
}

// ---------------------------------------------------------------------------
// The single-process constraint
// ---------------------------------------------------------------------------

// Pins the limitation described in the file header so it stays visible: a second instance
// in the same process silently takes over the process-wide frame ids. This is NOT asserting
// desirable behaviour — it is asserting the boundary of what one process supports, so that
// anyone tempted to compose two estimators into a single component container (or to write
// a concurrent-instance test here) finds this test instead of a heisenbug.
//
// If StateEstimator is ever refactored to per-instance state, this test SHOULD fail and
// should then be replaced with a genuine concurrent-instance test.
TEST(MultiDroneTest, MultipleInstancesShareProcessWideState)
{
  auto first = getStateEstimatorNode("drone0_shared");
  rclcpp::executors::MultiThreadedExecutor exec_first;
  exec_first.add_node(first);
  ASSERT_TRUE(waitForSetup(exec_first, "drone0_shared/base_link"));
  EXPECT_EQ(as2_state_estimator::StateEstimator::getMapFrame(), "drone0_shared/map");

  // Bring up a second drone while the first is still alive.
  auto second = getStateEstimatorNode("drone1_shared");
  rclcpp::executors::MultiThreadedExecutor exec_second;
  exec_second.add_node(second);
  ASSERT_TRUE(waitForSetup(exec_second, "drone1_shared/base_link"));

  // The frame ids are static: the second instance has overwritten them for BOTH nodes.
  EXPECT_EQ(as2_state_estimator::StateEstimator::getMapFrame(), "drone1_shared/map")
    << "if this now reports drone0_shared/map, frame state has become per-instance — "
    "replace this test with a real concurrent-instance test";
  EXPECT_EQ(as2_state_estimator::StateEstimator::getInstance(), second)
    << "instance_ tracks the most recently constructed node, so the first node's plugins "
    "would publish through the second node";
}

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
