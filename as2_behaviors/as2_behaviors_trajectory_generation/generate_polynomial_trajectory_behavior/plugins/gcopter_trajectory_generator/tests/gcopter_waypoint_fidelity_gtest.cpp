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
 * @file waypoint_fidelity_gtest.cpp
 *
 * @brief Standalone fidelity tests for the gcopter trajectory plugin.
 *
 * Drives gcopter_lib through the same anchor expansion the plugin uses,
 * without instantiating the pluginlib runtime. Validates that:
 *
 *   - With anchoring enabled, the optimised trajectory passes within a
 *     reasonable tolerance of every user waypoint (including intermediates
 *     in zigzag-like paths) — the issue that the wide AABB corridor lets
 *     the solver cut corners unless anchors pin the trajectory near each
 *     waypoint.
 *
 *   - The L-BFGS optimisation converges when consecutive waypoints share
 *     xy and only differ in z (purely vertical segment), thanks to the
 *     epsilon padding of degenerate AABB axes in the corridor builder.
 *
 * @authors Rafael Perez-Segui
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "gcopter_lib/trajectory_generator.hpp"
#include "gcopter_lib/types.hpp"
#include "gcopter_trajectory_generator/gcopter_trajectory_generator.hpp"

namespace
{

constexpr double kFinalWaypointTolerance = 0.20;         // [m]
constexpr double kIntermediateWaypointTolerance = 0.40;  // [m]

constexpr double kCorridorMargin = 0.5;        // [m] loose corridor
constexpr double kWaypointMargin = 0.05;       // [m] tight pinch margin
constexpr double kWaypointAnchorRadius = 0.5;  // [m] offset for anchors

gcopter_lib::GeneratorConfig defaultConfig()
{
  gcopter_lib::GeneratorConfig cfg;
  cfg.params.mass = 1.0;
  cfg.params.gravity = 9.81;
  cfg.params.horizontal_drag = 0.1;
  cfg.params.vertical_drag = 0.1;
  cfg.params.parasitic_drag = 0.01;
  cfg.params.speed_smooth_factor = 0.01;

  cfg.limits.max_velocity = 5.0;
  cfg.limits.max_body_rate = 6.0;
  cfg.limits.max_tilt_angle = 0.785;
  cfg.limits.min_thrust = 0.1;
  cfg.limits.max_thrust = 30.0;

  cfg.optimization.time_weight = 512.0;
  cfg.optimization.position_weight = 1.0e4;
  cfg.optimization.velocity_weight = 1.0e4;
  cfg.optimization.body_rate_weight = 1.0e4;
  cfg.optimization.tilt_weight = 1.0e4;
  cfg.optimization.thrust_weight = 1.0e4;
  cfg.optimization.smoothing_eps = 1.0e-2;
  cfg.optimization.integral_resolution = 16;
  cfg.optimization.rel_cost_tol = 1.0e-3;
  cfg.optimization.corridor_margin = kCorridorMargin;
  return cfg;
}

double minDistanceToTarget(
  const gcopter_lib::TrajectoryGenerator & gen,
  const Eigen::Vector3d & target)
{
  const double t0 = gen.minTime();
  const double t1 = gen.maxTime();
  const double dt = 0.01;
  double best = std::numeric_limits<double>::infinity();
  for (double t = t0; t <= t1; t += dt) {
    const Eigen::Vector3d p = gen.position(t);
    best = std::min(best, (p - target).norm());
  }
  const Eigen::Vector3d p_end = gen.position(t1);
  best = std::min(best, (p_end - target).norm());
  return best;
}

void runWithAnchors(
  const std::vector<Eigen::Vector3d> & raw_path,
  const std::string & label,
  const std::vector<Eigen::Vector3d> & expected_waypoints)
{
  using Plugin = gcopter_trajectory_generator::Plugin;
  const Plugin::ExpandedPath expanded = Plugin::expandPathWithAnchors(
    raw_path, kCorridorMargin, kWaypointMargin, kWaypointAnchorRadius);
  ASSERT_EQ(expanded.positions.size() - 1, expanded.margins.size()) << label;
  ASSERT_EQ(expanded.original_indices.size(), raw_path.size()) << label;

  std::vector<gcopter_lib::Waypoint> wps;
  wps.reserve(expanded.positions.size());
  for (const auto & p : expanded.positions) {
    gcopter_lib::Waypoint wp;
    wp.position = p;
    wps.emplace_back(std::move(wp));
  }

  gcopter_lib::TrajectoryGenerator gen(defaultConfig());
  ASSERT_TRUE(
    gen.generate(wps, defaultConfig().limits.max_velocity, expanded.margins))
    << label << ": generate() returned false";

  for (std::size_t i = 0; i < expected_waypoints.size(); ++i) {
    const double d = minDistanceToTarget(gen, expected_waypoints[i]);
    const bool is_endpoint =
      (i == 0) || (i + 1 == expected_waypoints.size());
    const double tol = is_endpoint ?
      kFinalWaypointTolerance :
      kIntermediateWaypointTolerance;
    EXPECT_LE(d, tol)
      << label << ": waypoint #" << i
      << " (" << expected_waypoints[i].transpose() << ") was missed by "
      << d << " m (tol " << tol << " m)";
  }
}

}  // namespace

TEST(GcopterFidelity, DiagonalClimb)
{
  // Two-waypoint diagonal climb. GCOPTER's L-BFGS solver fails on a fully
  // 1-D 2-waypoint path (line-search divergence on the trivial univariate
  // problem); the test moves a small horizontal distance as well so the
  // optimisation is genuinely 3-D and converges.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 2.0},
  };
  runWithAnchors(path, "DiagonalClimb", path);
}

TEST(GcopterFidelity, HorizontalZigzag)
{
  // Five waypoints zig-zagging in y at constant z. Without anchors the AABB
  // overlap between consecutive segments is wide (almost the full y range)
  // and the solver cuts corners. The anchored expansion must keep the
  // trajectory near each user waypoint. Legs and amplitude leave the
  // anchor pinch corridors numerically well-behaved.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 1.0},
    {5.0, 1.0, 1.0},
    {10.0, -1.0, 1.0},
    {15.0, 1.0, 1.0},
    {20.0, 0.0, 1.0},
  };
  runWithAnchors(path, "HorizontalZigzag", path);
}

TEST(GcopterFidelity, ColinearVerticalSegment)
{
  // Three waypoints where the first leg is purely vertical (xy unchanged)
  // and the second leg moves sideways. Without the AABB epsilon padding
  // applied in corridor_builder.cpp, the solver returned a "negative
  // line-search step" error for the vertical leg and generate() failed.
  // With the fix it must converge.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 2.0},
    {2.0, 0.0, 2.0},
  };
  runWithAnchors(path, "ColinearVerticalSegment", path);
}

TEST(GcopterFidelity, MultipleColinearVerticalWaypoints)
{
  // Four waypoints sharing xy across two consecutive vertical segments,
  // followed by a horizontal leg. Without the AABB epsilon padding the
  // solver could fail at the first vertical segment; with the fix it
  // should converge.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, 2.0},
    {2.0, 0.0, 2.0},
  };
  runWithAnchors(path, "MultipleColinearVerticalWaypoints", path);
}
