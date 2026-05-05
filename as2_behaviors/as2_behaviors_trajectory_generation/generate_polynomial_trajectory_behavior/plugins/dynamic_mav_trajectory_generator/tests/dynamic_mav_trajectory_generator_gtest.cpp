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
 * @file dynamic_mav_trajectory_generator_gtest.cpp
 *
 * @brief Plugin-level tests for dynamic_mav_trajectory_generator.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase;

namespace
{
as2_msgs::msg::PoseStampedWithID makeWaypoint(
  const std::string & id, double x,
  double y, double z)
{
  as2_msgs::msg::PoseStampedWithID wp;
  wp.id = id;
  wp.pose.header.frame_id = "earth";
  wp.pose.pose.position.x = x;
  wp.pose.pose.position.y = y;
  wp.pose.pose.position.z = z;
  wp.pose.pose.orientation.w = 1.0;
  return wp;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "earth";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

struct PluginFixture
{
  std::shared_ptr<as2::Node> node;
  std::shared_ptr<pluginlib::ClassLoader<PluginBase>> loader;
  std::shared_ptr<PluginBase> plugin;
};

PluginFixture
makePluginFixture(
  const std::string & node_name,
  const std::vector<rclcpp::Parameter> & parameter_overrides)
{
  PluginFixture f;
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides);
  f.node = std::make_shared<as2::Node>(node_name, options);
  f.loader = std::make_shared<pluginlib::ClassLoader<PluginBase>>(
    "as2_behaviors_trajectory_generation",
    "generate_polynomial_trajectory_behavior_plugin_base::"
    "GeneratePolynomialTrajectoryBase");
  f.plugin = f.loader->createSharedInstance(
    "dynamic_mav_trajectory_generator::Plugin");
  f.plugin->initialize(f.node.get(), "dynamic_mav_trajectory_generator");
  return f;
}

class TestParameterPlugin
  : public generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase
{
public:
  using GeneratePolynomialTrajectoryBase::getParameter;

  bool generateTrajectory(
    const std::vector<as2_msgs::msg::PoseStampedWithID> &,
    double,
    double /*t_trajectory_now*/) override
  {
    return true;
  }
  bool evaluate(
    double, as2_msgs::msg::TrajectoryPoint &, bool) override
  {
    return true;
  }
  bool isFinished(double) const override {return false;}
  double getDuration() const override {return 0.0;}
  bool isTrajectoryGenerated() override {return false;}
  void reset() override {}
  std::string getNextWaypointId() override {return {};}
};
}  // namespace

TEST(DynamicMavTrajectoryGenerator, load_generate_update_get_next) {
  auto fx = makePluginFixture("dynamic_mav_traj_gen_test", {});

  // Mission waypoints only — the plugin reads the start state from
  // vehicle_pose_/vehicle_twist_ via setVehicleState.
  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
    makeWaypoint("waypoint_000", 1.0, 0.0, 0.0),
    makeWaypoint("waypoint_001", 2.0, 0.0, 0.0),
  };

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 0.0), zero_twist);

  ASSERT_TRUE(
    fx.plugin->generateTrajectory(
      waypoints, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.0));
  ASSERT_TRUE(fx.plugin->isTrajectoryGenerated());
  ASSERT_FALSE(fx.plugin->isFinished(0.0));
  ASSERT_TRUE(fx.plugin->isFinished(1e6));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_000");

  as2_msgs::msg::TrajectoryPoint point;
  EXPECT_TRUE(fx.plugin->evaluate(0.0, point, false));

  // Smooth update: modify pose of an existing waypoint and add a new one.
  // The dynamic backend stitches without resetting its internal time
  // origin, so subsequent isFinished/evaluate calls on the same
  // t_trajectory axis must keep working.
  std::vector<as2_msgs::msg::PoseStampedWithID> updated = {
    makeWaypoint("waypoint_000", 1.5, 0.0, 0.0),
    makeWaypoint("waypoint_001", 2.5, 0.0, 0.0),
    makeWaypoint("waypoint_002", 3.0, 0.0, 0.0),
  };
  EXPECT_TRUE(
    fx.plugin->updateWaypoints(
      updated, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.5));
  EXPECT_FALSE(fx.plugin->isFinished(0.5));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_000");
}

TEST(GetParameter, with_default_and_override) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("test_plugin.test_param_double", 1.5),
  });
  auto node =
    std::make_shared<as2::Node>("get_param_default_with_override", options);
  TestParameterPlugin plugin;
  plugin.initialize(node.get(), "test_plugin");

  double value = 0.25;
  plugin.getParameter("test_param_double", value, true);
  EXPECT_DOUBLE_EQ(value, 1.5);
  EXPECT_TRUE(node->has_parameter("test_plugin.test_param_double"));
}

TEST(GetParameter, with_default_no_override_keeps_default) {
  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<as2::Node>("get_param_default_no_override", options);
  TestParameterPlugin plugin;
  plugin.initialize(node.get(), "test_plugin");

  double value = 0.25;
  plugin.getParameter("test_param_double", value, true);
  EXPECT_DOUBLE_EQ(value, 0.25);
  EXPECT_TRUE(node->has_parameter("test_plugin.test_param_double"));
}

TEST(GetParameter, required_with_override) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("test_plugin.required_string", std::string("provided")),
  });
  auto node =
    std::make_shared<as2::Node>("get_param_required_with_override", options);
  TestParameterPlugin plugin;
  plugin.initialize(node.get(), "test_plugin");

  std::string value;
  plugin.getParameter("required_string", value, false);
  EXPECT_EQ(value, "provided");
  EXPECT_TRUE(node->has_parameter("test_plugin.required_string"));
}

TEST(GetParameter, required_no_override_logs_error) {
  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<as2::Node>("get_param_required_no_override", options);
  TestParameterPlugin plugin;
  plugin.initialize(node.get(), "test_plugin");

  std::string value = "untouched";
  plugin.getParameter("missing_required", value, false);
  EXPECT_EQ(value, "untouched");
}

TEST(GetParameter, unsupported_type_falls_back_to_generic) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter(
      "test_plugin.test_param_int64",
      static_cast<int64_t>(42)),
  });
  auto node =
    std::make_shared<as2::Node>("get_param_unsupported_type", options);
  TestParameterPlugin plugin;
  plugin.initialize(node.get(), "test_plugin");

  int64_t value = 0;
  plugin.getParameter("test_param_int64", value, true);
  EXPECT_EQ(value, static_cast<int64_t>(42));
  EXPECT_TRUE(node->has_parameter("test_plugin.test_param_int64"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
