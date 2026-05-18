// Copyright 2026 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names
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
 * @file integration_test.cpp
 *
 * @brief Single-process integration test for
 * GeneratePolynomialTrajectoryBehavior. The behavior, a mock support node and
 * an action client share a MultiThreadedExecutor. Parametrized over every
 * plugin discovered via pluginlib so any future plugin is exercised without
 * touching the test.
 *
 * @authors Rafael Perez-Segui
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>  // NOLINT(build/include_order)

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "as2_msgs/msg/behavior_status.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/controller_info.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"
#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp"

namespace
{

using Action = as2_msgs::action::GeneratePolynomialTrajectory;
using Goal = Action::Goal;
using Feedback = Action::Feedback;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
using BehaviorStatus = as2_msgs::msg::BehaviorStatus;
using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase;

constexpr const char * kTestNamespace = "test_drone";
constexpr const char * kBehaviorActionName = "TrajectoryGeneratorBehavior";
constexpr double kHoverHeight = 2.0;
constexpr double kMaxSpeed = 1.0;

/**
 * @brief Mock support node providing the minimum environment for the behavior
 * to activate: static TF (earth → odom → base_link), a periodic
 * TwistStamped on self_localization/twist, a ControllerInfo publisher and a
 * controller/set_control_mode service that always succeeds. The motion
 * reference handlers (trajectory + hover) used by the behavior issue a
 * synchronous setMode call when the controller's reported mode does not match
 * the desired one; without these mocks the behavior would block forever.
 */
class MockSupportNode : public as2::Node
{
public:
  MockSupportNode()
  : as2::Node("integration_test_mock", makeOptions())
  {
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publishStaticTransforms();

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "self_localization/twist", rclcpp::SensorDataQoS());

    controller_info_pub_ = this->create_publisher<as2_msgs::msg::ControllerInfo>(
      "controller/info", rclcpp::QoS(1));

    set_control_mode_srv_ =
      this->create_service<as2_msgs::srv::SetControlMode>(
      "controller/set_control_mode",
      [this](
        const std::shared_ptr<as2_msgs::srv::SetControlMode::Request> req,
        std::shared_ptr<as2_msgs::srv::SetControlMode::Response> resp) {
        resp->success = true;
        // Echo the requested mode back via controller/info so any concurrent
        // checkMode() observes the matching state.
        as2_msgs::msg::ControllerInfo info;
        info.input_control_mode = req->control_mode;
        info.output_control_mode = req->control_mode;
        controller_info_pub_->publish(info);
      });

    twist_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {publishZeroTwist();});
  }

private:
  static rclcpp::NodeOptions makeOptions()
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {
        "--ros-args",
        "-r", std::string("__ns:=/") + kTestNamespace,
        "-p", std::string("namespace:=") + kTestNamespace,
      });
    return options;
  }

  void publishStaticTransforms()
  {
    const std::string ns = kTestNamespace;
    const auto stamp = this->now();

    geometry_msgs::msg::TransformStamped earth_to_map;
    earth_to_map.header.stamp = stamp;
    earth_to_map.header.frame_id = "earth";
    earth_to_map.child_frame_id = ns + "/map";
    earth_to_map.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(earth_to_map);

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = ns + "/map";
    map_to_odom.child_frame_id = ns + "/odom";
    map_to_odom.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(map_to_odom);

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = stamp;
    odom_to_base.header.frame_id = ns + "/odom";
    odom_to_base.child_frame_id = ns + "/base_link";
    odom_to_base.transform.translation.z = kHoverHeight;
    odom_to_base.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(odom_to_base);
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = std::string(kTestNamespace) + "/base_link";
    twist_pub_->publish(msg);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr
    controller_info_pub_;
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr
    set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr twist_timer_;
};

/// Returns plugin names declared by pluginlib (strips the trailing "::Plugin").
std::vector<std::string> getAvailablePlugins()
{
  std::vector<std::string> names;
  try {
    pluginlib::ClassLoader<PluginBase> loader(
      "as2_behaviors_trajectory_generation",
      "generate_polynomial_trajectory_behavior_plugin_base::"
      "GeneratePolynomialTrajectoryBase");
    for (const auto & declared : loader.getDeclaredClasses()) {
      const std::string suffix = "::Plugin";
      if (declared.size() > suffix.size() &&
        declared.compare(
          declared.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        names.push_back(declared.substr(0, declared.size() - suffix.size()));
      } else {
        names.push_back(declared);
      }
    }
  } catch (const std::exception & ex) {
    std::fprintf(
      stderr, "getAvailablePlugins: pluginlib query failed: %s\n", ex.what());
  }
  return names;
}

/// Build a 3-waypoint goal in the namespaced odom frame at hover height.
Goal makeBaseGoal()
{
  Goal goal;
  goal.max_speed = kMaxSpeed;
  goal.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  goal.yaw.angle = 0.0f;

  const std::string frame = std::string(kTestNamespace) + "/odom";
  const std::vector<std::pair<std::string, double>> wps = {
    {"wp_a", 1.0}, {"wp_b", 2.0}, {"wp_c", 3.0}
  };
  for (const auto & [id, x] : wps) {
    as2_msgs::msg::PoseStampedWithID wp;
    wp.id = id;
    wp.pose.header.frame_id = frame;
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.z = kHoverHeight;
    wp.pose.pose.orientation.w = 1.0;
    goal.path.push_back(wp);
  }
  return goal;
}

/// Sleep helper that keeps spinning the executor while waiting.
template<typename Predicate>
bool waitFor(
  Predicate pred, std::chrono::milliseconds timeout,
  std::chrono::milliseconds poll = std::chrono::milliseconds(20))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {return true;}
    std::this_thread::sleep_for(poll);
  }
  return pred();
}

class IntegrationTest : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    const std::string package_share = ament_index_cpp::get_package_share_directory(
      "as2_behaviors_trajectory_generation");
    const std::string config_file = package_share +
      "/generate_polynomial_trajectory_behavior/config/config_default.yaml";

    rclcpp::NodeOptions options;
    options.arguments(
      {
        "--ros-args",
        "-r", std::string("__ns:=/") + kTestNamespace,
        "-p", std::string("namespace:=") + kTestNamespace,
        "-p", std::string("plugin_name:=") + GetParam(),
        "--params-file", config_file,
      });
    behavior_ = std::make_shared<GeneratePolynomialTrajectoryBehavior>(options);
    mock_ = std::make_shared<MockSupportNode>();

    rclcpp::NodeOptions client_options;
    client_options.arguments(
      {
        "--ros-args",
        "-r", std::string("__ns:=/") + kTestNamespace,
      });
    client_node_ = std::make_shared<rclcpp::Node>(
      "integration_test_client", client_options);

    action_client_ = rclcpp_action::create_client<Action>(
      client_node_, kBehaviorActionName);
    pause_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      std::string(kBehaviorActionName) + "/_behavior/pause");
    resume_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      std::string(kBehaviorActionName) + "/_behavior/resume");
    modify_client_ = client_node_->create_client<Action::Impl::SendGoalService>(
      std::string(kBehaviorActionName) + "/_behavior/modify");

    behavior_status_sub_ = client_node_->create_subscription<BehaviorStatus>(
      std::string(kBehaviorActionName) + "/_behavior/behavior_status", 10,
      [this](BehaviorStatus::SharedPtr msg) {
        last_behavior_status_.store(msg->status);
      });

    executor_.add_node(behavior_);
    executor_.add_node(mock_);
    executor_.add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});

    ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(5)))
      << "action server not available for plugin " << GetParam();
    ASSERT_TRUE(pause_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(resume_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(modify_client_->wait_for_service(std::chrono::seconds(5)));

    // The behavior rejects activations until at least one twist message has
    // arrived AND the static TF buffer is populated. The mock publishes twist
    // at 50 Hz; give it a brief window before exercising the action.
    ASSERT_TRUE(
      waitFor(
        [this]() {return sendProbeAndCheckAcceptance();},
        std::chrono::seconds(5)))
      << "behavior never reached a state where it accepts goals (plugin "
      << GetParam() << ")";
  }

  /// Try sending a one-waypoint probe goal; if accepted, immediately deactivate
  /// so we leave the behavior idle for the actual test. Returns true once the
  /// probe is accepted (i.e. the behavior is ready).
  bool sendProbeAndCheckAcceptance()
  {
    Goal probe;
    probe.max_speed = kMaxSpeed;
    probe.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
    as2_msgs::msg::PoseStampedWithID wp;
    wp.id = "probe";
    wp.pose.header.frame_id = std::string(kTestNamespace) + "/odom";
    wp.pose.pose.position.x = 0.5;
    wp.pose.pose.position.z = kHoverHeight;
    wp.pose.pose.orientation.w = 1.0;
    probe.path.push_back(wp);

    auto future = action_client_->async_send_goal(probe);
    if (future.wait_for(std::chrono::milliseconds(500)) !=
      std::future_status::ready)
    {
      return false;
    }
    auto handle = future.get();
    if (!handle) {return false;}

    // Cancel the probe goal so the behavior returns to IDLE.
    auto cancel_future = action_client_->async_cancel_goal(handle);
    cancel_future.wait_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lk(feedback_mutex_);
      feedbacks_.clear();
    }
    last_behavior_status_.store(BehaviorStatus::IDLE);
    return true;
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {spin_thread_.join();}
    behavior_status_sub_.reset();
    pause_client_.reset();
    resume_client_.reset();
    action_client_.reset();
    client_node_.reset();
    mock_.reset();
    behavior_.reset();
  }

  /// Send the goal through the action client and start collecting feedbacks.
  GoalHandle::SharedPtr sendGoal(const Goal & goal)
  {
    rclcpp_action::Client<Action>::SendGoalOptions opts;
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> fb) {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        feedbacks_.push_back(*fb);
      };
    auto future = action_client_->async_send_goal(goal, opts);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  /// Wait until at least one feedback satisfies @p pred.
  template<typename Pred>
  bool waitForFeedback(Pred pred, std::chrono::milliseconds timeout)
  {
    return waitFor(
      [&]() {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        for (const auto & fb : feedbacks_) {
          if (pred(fb)) {return true;}
        }
        return false;
      }, timeout);
  }

  std::vector<Feedback> snapshotFeedbacks()
  {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    return feedbacks_;
  }

  void clearFeedbacks()
  {
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    feedbacks_.clear();
  }

  bool callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return false;
    }
    return fut.get()->success;
  }

  /// Call the BehaviorServer modify service synchronously.
  bool callModify(const Goal & goal)
  {
    auto req = std::make_shared<Action::Impl::SendGoalService::Request>();
    req->goal = goal;
    auto fut = modify_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return false;
    }
    return fut.get()->accepted;
  }

  bool waitForBehaviorStatus(uint8_t expected, std::chrono::milliseconds timeout)
  {
    return waitFor(
      [&]() {return last_behavior_status_.load() == expected;}, timeout);
  }

  // Members.
  std::shared_ptr<GeneratePolynomialTrajectoryBehavior> behavior_;
  std::shared_ptr<MockSupportNode> mock_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Client<Action::Impl::SendGoalService>::SharedPtr modify_client_;
  rclcpp::Subscription<BehaviorStatus>::SharedPtr behavior_status_sub_;
  std::atomic<uint8_t> last_behavior_status_{BehaviorStatus::IDLE};

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;

  std::mutex feedback_mutex_;
  std::vector<Feedback> feedbacks_;
};

TEST_P(IntegrationTest, BasicGoalCompletes)
{
  auto goal_handle = sendGoal(makeBaseGoal());
  ASSERT_NE(goal_handle, nullptr);

  ASSERT_TRUE(
    waitForFeedback(
      [](const Feedback & fb) {return fb.next_waypoint_id == "wp_a";},
      std::chrono::seconds(3)));

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(
    result_future.wait_for(std::chrono::seconds(20)),
    std::future_status::ready);
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->trajectory_generator_success);

  const auto fbs = snapshotFeedbacks();
  ASSERT_FALSE(fbs.empty());
  // The terminal feedback may still report 1 (the last waypoint about to be
  // reached at SUCCESS time). The invariant is that the queue is non-growing
  // and ends near zero pending entries.
  EXPECT_LE(fbs.back().remaining_waypoints, 1u);
  // The queue should have been observed at least at "wp_b" along the way.
  bool saw_wp_b = false;
  for (const auto & fb : fbs) {
    if (fb.next_waypoint_id == "wp_b") {saw_wp_b = true; break;}
  }
  EXPECT_TRUE(saw_wp_b);
}

TEST_P(IntegrationTest, ModifyMidFlightPreservesProgress)
{
  if (!behavior_) {GTEST_SKIP();}

  // The behavior delegates each modify to plugin_->updateWaypoints().
  // Plugins that override the contract stitch smoothly (preserve t0);
  // plugins that rely on the base default regenerate from the live
  // pose+twist as boundary conditions. In both cases the merge logic in
  // the wrapper drops already-passed waypoints; the modify request must
  // therefore include only ids that are still pending — sending an
  // already-passed id (e.g. "wp_a") would make the merge append it at
  // the tail and the trajectory would visit it again.

  auto goal_handle = sendGoal(makeBaseGoal());
  ASSERT_NE(goal_handle, nullptr);

  // Wait until the queue advances past wp_a so we can observe progress.
  ASSERT_TRUE(
    waitForFeedback(
      [](const Feedback & fb) {return fb.next_waypoint_id == "wp_b";},
      std::chrono::seconds(5)));

  // Capture the index marker BEFORE the modify request so we can assert on
  // post-modify feedback only.
  const std::size_t modify_marker = snapshotFeedbacks().size();

  // Send a modify with the still-pending ids only, tweaked in X. After
  // the modify the trajectory must continue forward; feedback after the
  // modify should never report "wp_a" (the cleared waypoint).
  Goal modify = makeBaseGoal();
  modify.path.erase(
    std::remove_if(
      modify.path.begin(), modify.path.end(),
      [](const as2_msgs::msg::PoseStampedWithID & wp) {
        return wp.id == "wp_a";
      }),
    modify.path.end());
  for (auto & wp : modify.path) {
    wp.pose.pose.position.x += 0.2;
  }
  ASSERT_TRUE(callModify(modify));

  // Run a bit longer to collect post-modify feedback samples.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto fbs = snapshotFeedbacks();
  ASSERT_GT(fbs.size(), modify_marker);
  for (std::size_t i = modify_marker; i < fbs.size(); ++i) {
    EXPECT_NE(fbs[i].next_waypoint_id, "wp_a")
      << "next_waypoint_id reverted to wp_a after dynamic modify";
  }

  // Let the goal finish.
  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(
    result_future.wait_for(std::chrono::seconds(20)),
    std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_P(IntegrationTest, PauseAndResumeCompletes)
{
  auto goal_handle = sendGoal(makeBaseGoal());
  ASSERT_NE(goal_handle, nullptr);

  // Wait until execution has progressed to wp_b before pausing.
  ASSERT_TRUE(
    waitForFeedback(
      [](const Feedback & fb) {return fb.next_waypoint_id == "wp_b";},
      std::chrono::seconds(5)));

  ASSERT_TRUE(callTrigger(pause_client_));
  ASSERT_TRUE(
    waitForBehaviorStatus(
      BehaviorStatus::PAUSED, std::chrono::seconds(2)));

  const std::size_t pause_marker = snapshotFeedbacks().size();

  ASSERT_TRUE(callTrigger(resume_client_));
  ASSERT_TRUE(
    waitForBehaviorStatus(
      BehaviorStatus::RUNNING, std::chrono::seconds(2)));

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(
    result_future.wait_for(std::chrono::seconds(20)),
    std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  // Feedback after resume must never report a waypoint earlier than wp_b.
  const auto fbs = snapshotFeedbacks();
  for (std::size_t i = pause_marker; i < fbs.size(); ++i) {
    EXPECT_NE(fbs[i].next_waypoint_id, "wp_a")
      << "next_waypoint_id rewinded to wp_a after resume";
  }
}

/// Build a single-waypoint goal placed `dist_m` ahead of the vehicle (which
/// hovers at (0, 0, kHoverHeight) in odom). With `dist_m < kDegenerateDistanceM`
/// the host must short-circuit the plugin and complete with SUCCESS.
Goal makeDegenerateGoal(double dist_m)
{
  Goal goal;
  goal.max_speed = kMaxSpeed;
  goal.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  goal.yaw.angle = 0.0f;

  as2_msgs::msg::PoseStampedWithID wp;
  wp.id = "wp_close";
  wp.pose.header.frame_id = std::string(kTestNamespace) + "/odom";
  wp.pose.pose.position.x = dist_m;
  wp.pose.pose.position.z = kHoverHeight;
  wp.pose.pose.orientation.w = 1.0;
  goal.path.push_back(wp);
  return goal;
}

TEST_P(IntegrationTest, GoToDegenerateTargetReturnsSuccess)
{
  // Single waypoint within kDegenerateDistanceM: the host must skip the
  // plugin and report SUCCESS on the first tick.
  auto goal_handle = sendGoal(makeDegenerateGoal(0.01));
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(
    result_future.wait_for(std::chrono::seconds(5)),
    std::future_status::ready)
    << "behavior did not finish for plugin " << GetParam();
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped.result->trajectory_generator_success);
}

INSTANTIATE_TEST_SUITE_P(
  AllPlugins, IntegrationTest,
  ::testing::ValuesIn(getAvailablePlugins()),
  [](const ::testing::TestParamInfo<std::string> & info) {
    std::string sanitized = info.param;
    for (char & c : sanitized) {
      if (!std::isalnum(static_cast<unsigned char>(c))) {c = '_';}
    }
    return sanitized;
  });

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
