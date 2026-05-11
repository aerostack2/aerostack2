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
 * @file mav_trajectory_generator_gtest.cpp
 *
 * @brief Plugin-level tests for mav_trajectory_generator.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase;

namespace
{
as2_msgs::msg::PoseStampedWithID makeWaypoint(
  const std::string & id, double x, double y, double z)
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
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "earth";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;
  return p;
}

struct PluginFixture
{
  std::shared_ptr<as2::Node> node;
  std::shared_ptr<pluginlib::ClassLoader<PluginBase>> loader;
  std::shared_ptr<PluginBase> plugin;
};

PluginFixture makePluginFixture(
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
    "mav_trajectory_generator::Plugin");
  f.plugin->initialize(f.node.get(), "mav_trajectory_generator");
  return f;
}
}  // namespace

TEST(MavTrajectoryGenerator, load_generate_evaluate_reset) {
  auto fx = makePluginFixture("mav_traj_gen_test", {});

  // Mission waypoints only — the plugin auto-prepends the current
  // vehicle pose as the trajectory start.
  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
    makeWaypoint("waypoint_000", 2.0, 0.0, 1.0),
    makeWaypoint("waypoint_001", 4.0, 0.0, 1.0),
  };

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 1.0), zero_twist);

  ASSERT_TRUE(
    fx.plugin->generateTrajectory(
      waypoints, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.0));
  ASSERT_TRUE(fx.plugin->isTrajectoryGenerated());
  ASSERT_FALSE(fx.plugin->isFinished(0.0));
  ASSERT_TRUE(fx.plugin->isFinished(1e6));

  // At t_trajectory = 0, the next pending waypoint is the first mission
  // waypoint after the implicit start.
  as2_msgs::msg::TrajectoryPoint p_start;
  EXPECT_TRUE(fx.plugin->evaluate(0.0, p_start, false));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_000");

  double t_end = 0.0;
  for (double t = 0.0; t < 1e3; t += 0.05) {
    if (fx.plugin->isFinished(t)) {
      t_end = t;
      break;
    }
  }
  ASSERT_GT(t_end, 0.0);
  const double t_mid = 0.5 * t_end;
  as2_msgs::msg::TrajectoryPoint p_mid;
  EXPECT_TRUE(fx.plugin->evaluate(t_mid, p_mid, false));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_001");

  fx.plugin->reset();
  EXPECT_FALSE(fx.plugin->isTrajectoryGenerated());
  EXPECT_TRUE(fx.plugin->getNextWaypointId().empty());
}

TEST(MavTrajectoryGenerator, rejects_degenerate_inputs) {
  auto fx = makePluginFixture("mav_traj_gen_degenerate", {});

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 1.0), zero_twist);

  std::vector<as2_msgs::msg::PoseStampedWithID> empty;
  EXPECT_FALSE(
    fx.plugin->generateTrajectory(empty, 1.0, /*t_trajectory_now=*/ 0.0));
  EXPECT_FALSE(fx.plugin->isTrajectoryGenerated());

  std::vector<as2_msgs::msg::PoseStampedWithID> one = {
    makeWaypoint("end", 1.0, 0.0, 1.0),
  };
  EXPECT_FALSE(
    fx.plugin->generateTrajectory(
      one, /*max_speed=*/ 0.0, /*t_trajectory_now=*/ 0.0));
  EXPECT_FALSE(fx.plugin->isTrajectoryGenerated());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
