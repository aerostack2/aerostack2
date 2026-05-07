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
 * @file gcopter_trajectory_generator_gtest.cpp
 *
 * @brief Plugin-level tests for gcopter_trajectory_generator.
 *
 * @authors Rafael Perez-Segui
 */

#include <gtest/gtest.h>

#include <cstdio>
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
// Tolerances for the single-waypoint vertical climb test below. The xy
// tolerance at the start is relaxed because gcopter_lib applies
// `OptimizationConfig::vertical_perturbation` (~5 cm in x by default) to the
// initial state when every waypoint is vertically aligned, sidestepping the
// L-BFGS line-search divergence on purely 1D problems.
constexpr double kStartXyTol = 0.10;  // [m] absorbs vertical_perturbation
constexpr double kGoalXyTol = 0.20;   // [m] xy drift at goal
constexpr double kGoalZTol = 0.20;    // [m] z error at goal

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
    "gcopter_trajectory_generator::Plugin");
  f.plugin->initialize(f.node.get(), "gcopter_trajectory_generator");
  return f;
}
}  // namespace

TEST(GcopterTrajectoryGenerator, load_generate_evaluate_reset) {
  auto fx = makePluginFixture("gcopter_traj_gen_test", {});

  // Mission waypoints only — the plugin auto-prepends the current
  // vehicle pose (read from vehicle_pose_) as the trajectory start.
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
  // Fresh trajectory: t_trajectory_now = 0 must not be finished, and the
  // duration must be strictly positive (we walk it to find the end below).
  ASSERT_FALSE(fx.plugin->isFinished(0.0));
  ASSERT_TRUE(fx.plugin->isFinished(1e6));

  // At t_trajectory = 0, the next pending waypoint is the first mission
  // waypoint after the implicit start.
  as2_msgs::msg::TrajectoryPoint p_start;
  EXPECT_TRUE(fx.plugin->evaluate(0.0, p_start, false));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_000");

  // Walk t_trajectory until isFinished flips, then sample at the midpoint
  // to advance past the first mission segment.
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

TEST(GcopterTrajectoryGenerator, update_waypoints_re_anchors_offset) {
  // Regression test for the gcopter t_min == 0 issue: after the host
  // advances the trajectory time and triggers an updateWaypoints() that
  // falls back to regeneration, the plugin must re-anchor its internal
  // offset against the new t_trajectory_now so subsequent evaluations
  // stay within the valid window.
  auto fx = makePluginFixture("gcopter_traj_gen_reanchor", {});

  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
    makeWaypoint("waypoint_000", 2.0, 0.0, 1.0),
    makeWaypoint("waypoint_001", 4.0, 0.0, 1.0),
  };

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 1.0), zero_twist);

  ASSERT_TRUE(
    fx.plugin->generateTrajectory(waypoints, 1.0, /*t_trajectory_now=*/ 0.0));

  // Simulate the host advancing trajectory_time_ to 5.0 s while still
  // executing the previous trajectory, then triggering an update.
  constexpr double kTNow = 5.0;
  std::vector<as2_msgs::msg::PoseStampedWithID> updated = {
    makeWaypoint("waypoint_010", 6.0, 0.0, 1.0),
    makeWaypoint("waypoint_011", 8.0, 0.0, 1.0),
  };
  ASSERT_TRUE(
    fx.plugin->updateWaypoints(updated, 1.0, kTNow));

  // After the regen, isFinished(t_trajectory) must be false at kTNow and
  // become true some time later (not immediately, which is the bug).
  EXPECT_FALSE(fx.plugin->isFinished(kTNow));
  EXPECT_TRUE(fx.plugin->isFinished(kTNow + 1e3));

  as2_msgs::msg::TrajectoryPoint p;
  EXPECT_TRUE(fx.plugin->evaluate(kTNow, p, false));
}

TEST(GcopterTrajectoryGenerator, rejects_degenerate_inputs) {
  auto fx = makePluginFixture("gcopter_traj_gen_degenerate", {});

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 1.0), zero_twist);

  // Empty mission waypoints: nothing to plan toward.
  std::vector<as2_msgs::msg::PoseStampedWithID> empty;
  EXPECT_FALSE(
    fx.plugin->generateTrajectory(empty, 1.0, /*t_trajectory_now=*/ 0.0));
  EXPECT_FALSE(fx.plugin->isTrajectoryGenerated());

  // Non-positive max_speed is invalid regardless of waypoint count.
  std::vector<as2_msgs::msg::PoseStampedWithID> one = {
    makeWaypoint("end", 1.0, 0.0, 1.0),
  };
  EXPECT_FALSE(
    fx.plugin->generateTrajectory(
      one, /*max_speed=*/ 0.0, /*t_trajectory_now=*/ 0.0));
  EXPECT_FALSE(fx.plugin->isTrajectoryGenerated());
}

TEST(GcopterTrajectoryGenerator, single_waypoint_vertical_climb) {
  // Single mission waypoint shares xy with the vehicle pose: the auto-prepended
  // implicit start makes this a purely vertical climb from (0,0,0) to (0,0,2).
  // gcopter_lib's vertical_perturbation workaround in trajectory_generator.cpp
  // displaces the initial state by ~5 cm in x so the L-BFGS solver does not
  // diverge on the 1D problem; the xy tolerances here account for it.
  auto fx = makePluginFixture("gcopter_traj_gen_vertical_one_point", {});

  geometry_msgs::msg::TwistStamped zero_twist;
  fx.plugin->setVehicleState(makePose(0.0, 0.0, 0.0), zero_twist);

  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
    makeWaypoint("waypoint_000", 0.0, 0.0, 2.0),
  };

  ASSERT_TRUE(
    fx.plugin->generateTrajectory(
      waypoints, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.0));
  ASSERT_TRUE(fx.plugin->isTrajectoryGenerated());
  ASSERT_FALSE(fx.plugin->isFinished(0.0));
  ASSERT_TRUE(fx.plugin->isFinished(1e6));
  EXPECT_EQ(fx.plugin->getNextWaypointId(), "waypoint_000");

  as2_msgs::msg::TrajectoryPoint p_start;
  ASSERT_TRUE(fx.plugin->evaluate(0.0, p_start, false));
  EXPECT_NEAR(p_start.position.x, 0.0, kStartXyTol);
  EXPECT_NEAR(p_start.position.y, 0.0, kStartXyTol);
  EXPECT_NEAR(p_start.position.z, 0.0, kGoalZTol);

  double t_end = 0.0;
  for (double t = 0.0; t < 1e3; t += 0.05) {
    if (fx.plugin->isFinished(t)) {
      t_end = t;
      break;
    }
  }
  ASSERT_GT(t_end, 0.0);

  as2_msgs::msg::TrajectoryPoint p_end;
  ASSERT_TRUE(fx.plugin->evaluate(t_end, p_end, false));
  EXPECT_NEAR(p_end.position.x, 0.0, kGoalXyTol);
  EXPECT_NEAR(p_end.position.y, 0.0, kGoalXyTol);
  EXPECT_NEAR(p_end.position.z, 2.0, kGoalZTol);
}

TEST(GcopterTrajectoryGenerator, single_waypoint_vertical_climb_with_xy_drift) {
  // Regression for the runtime takeoff failure: the prepended vehicle pose is
  // not bit-exactly aligned with the user waypoint in xy (mm-to-cm drift
  // from localisation), so the legacy 1 mm `vertical_alignment_threshold`
  // skipped the perturbation patch even though the L-BFGS problem was still
  // numerically near-1D. With the new default (10 cm), drift up to 10 cm
  // stays inside the trigger band and the patch absorbs it.
  auto fx = makePluginFixture(
    "gcopter_traj_gen_vertical_one_point_drift", {});

  geometry_msgs::msg::TwistStamped zero_twist;
  // Drift representative of localisation noise: 1 cm in x, 5 mm in y.
  fx.plugin->setVehicleState(makePose(0.01, 0.005, 0.0), zero_twist);

  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
    makeWaypoint("waypoint_000", 0.0, 0.0, 1.0),
  };

  ASSERT_TRUE(
    fx.plugin->generateTrajectory(
      waypoints, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.0));
  ASSERT_TRUE(fx.plugin->isTrajectoryGenerated());
  ASSERT_FALSE(fx.plugin->isFinished(0.0));
  ASSERT_TRUE(fx.plugin->isFinished(1e6));

  double t_end = 0.0;
  for (double t = 0.0; t < 1e3; t += 0.05) {
    if (fx.plugin->isFinished(t)) {
      t_end = t;
      break;
    }
  }
  ASSERT_GT(t_end, 0.0);

  as2_msgs::msg::TrajectoryPoint p_end;
  ASSERT_TRUE(fx.plugin->evaluate(t_end, p_end, false));
  EXPECT_NEAR(p_end.position.x, 0.0, kGoalXyTol);
  EXPECT_NEAR(p_end.position.y, 0.0, kGoalXyTol);
  EXPECT_NEAR(p_end.position.z, 1.0, kGoalZTol);
}

// Exploratory sweep used to characterise the well-conditioned regime of the
// bare L-BFGS solver (patch disabled). Kept as a permanent diagnostic tool
// for future tuning of `vertical_alignment_threshold`. Disabled by default;
// run with
//   --gtest_also_run_disabled_tests --gtest_filter='*ThresholdSweep*'
// The patch is forced off (vertical_perturbation = 0) so the test reports
// the well-conditioned regime of the bare L-BFGS solver as a function of
// the xy drift between vehicle pose and the requested vertical waypoint.
//
// Two configurations are swept side by side:
//   - "default":  the YAML defaults of the plugin (loose corridor 0.1 m).
//   - "user":     the config the user reported the bug with (corridor 2.0 m,
//                 waypoint_margin 0.2, waypoint_anchor_radius 0.5).
TEST(GcopterTrajectoryGenerator, DISABLED_ThresholdSweep_DriftToConvergence) {
  const std::vector<double> drifts = {
    0.0, 1.0e-4, 5.0e-4, 1.0e-3, 5.0e-3,
    1.0e-2, 2.0e-2, 5.0e-2, 1.0e-1, 2.0e-1, 5.0e-1,
  };
  constexpr int kRunsPerDrift = 5;

  // Goals to exercise: the user's takeoff (z = 1 m) and a longer climb (z = 2 m).
  const std::vector<double> goal_zs = {1.0, 2.0};

  struct ConfigCase
  {
    const char * label;
    std::vector<rclcpp::Parameter> extras;
  };

  // Two configs: the YAML defaults of the plugin and the full set of
  // overrides the user reported the bug with (drone params, limits, optim
  // weights, anchoring knobs, all from the runtime log).
  const std::vector<ConfigCase> configs = {
    {"default", {}},
    {"user_full",
      {
        rclcpp::Parameter("gcopter_trajectory_generator.drone.mass", 1.14),
        rclcpp::Parameter("gcopter_trajectory_generator.drone.gravity", 9.81),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.drone.horizontal_drag", 0.7),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.drone.vertical_drag", 0.8),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.drone.parasitic_drag", 0.01),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.drone.speed_smooth_factor", 0.0001),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.limits.max_velocity", 4.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.limits.max_body_rate", 2.1),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.limits.max_tilt_angle", 1.05),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.limits.min_thrust", 2.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.limits.max_thrust", 12.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.time_weight", 20.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.position_weight", 10000.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.velocity_weight", 10000.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.body_rate_weight",
          10000.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.tilt_weight", 10000.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.thrust_weight", 100000.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.smoothing_eps", 0.01),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.integral_resolution", 16),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.rel_cost_tol", 1.0e-5),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.optimization.corridor_margin", 2.0),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.waypoints.waypoint_margin", 0.2),
        rclcpp::Parameter(
          "gcopter_trajectory_generator.waypoints.waypoint_anchor_radius", 0.5),
      }},
  };

  std::printf("\n[ThresholdSweep] cfg       z_goal  drift_x   passes / runs\n");
  for (const auto & cfg : configs) {
    for (double zg : goal_zs) {
      for (double d : drifts) {
        int passes = 0;
        for (int i = 0; i < kRunsPerDrift; ++i) {
          const double jitter = 1.0e-9 * static_cast<double>(i - kRunsPerDrift / 2);
          const double drift_x = d + jitter;

          std::vector<rclcpp::Parameter> overrides = {
            // Patch disabled: the sweep characterises the well-conditioned
            // regime of the bare L-BFGS solver as a function of xy drift
            // between vehicle pose and the requested vertical waypoint.
            // Toggle these to 0.05 / 1e-3 to repeat with the patch active.
            rclcpp::Parameter(
              "gcopter_trajectory_generator.optimization.vertical_perturbation",
              0.0),
            rclcpp::Parameter(
              "gcopter_trajectory_generator.optimization.vertical_alignment_threshold",
              0.0),
          };
          for (const auto & p : cfg.extras) {
            overrides.push_back(p);
          }
          auto fx = makePluginFixture(
            std::string("gcopter_threshold_sweep_") + cfg.label + "_" +
            std::to_string(static_cast<int>(zg * 100)) + "_" +
            std::to_string(i) + "_" +
            std::to_string(static_cast<int>(d * 1.0e6)),
            overrides);

          geometry_msgs::msg::TwistStamped zero_twist;
          fx.plugin->setVehicleState(makePose(drift_x, 0.0, 0.0), zero_twist);

          std::vector<as2_msgs::msg::PoseStampedWithID> waypoints = {
            makeWaypoint("waypoint_000", 0.0, 0.0, zg),
          };
          const bool ok = fx.plugin->generateTrajectory(
            waypoints, /*max_speed=*/ 1.0, /*t_trajectory_now=*/ 0.0);
          if (ok) {
            ++passes;
          }
        }
        std::printf(
          "[ThresholdSweep] %-8s  %5.2f   %8.4f m   %d / %d\n",
          cfg.label, zg, d, passes, kRunsPerDrift);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
