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
 * @brief Standalone fidelity tests for trajectory_generator_jerk_limited.
 *
 * Exercises the upstream library directly (no ROS 2, no pluginlib) on the
 * geometries that fail in the user-facing setup: a vertical climb in a
 * straight line and a horizontal zigzag. For each waypoint provided, checks
 * the minimum distance from the sampled trajectory to that waypoint and
 * fails if the trajectory does not pass within a small tolerance.
 *
 * The tolerance is intentionally separated for the LAST waypoint (which the
 * generator is now required to reach tightly via final_acceptance_radius)
 * and for intermediate waypoints (which are subject to the L1 corner-cut
 * controlled by acceptance_radius).
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

#include "trajectory_generator_jerk_limited/trajectory_generator.hpp"
#include "trajectory_generator_jerk_limited/types.hpp"

namespace tg = trajectory_generator_jerk_limited;

namespace
{

// Tolerances for waypoint adherence. The last waypoint must be reached
// tightly thanks to final_acceptance_radius; intermediate waypoints are
// allowed a wider margin since acceptance_radius drives the L1 corner-cut.
constexpr double kFinalWaypointTolerance = 0.10;         // [m]
constexpr double kIntermediateWaypointTolerance = 1.10;  // [m]

tg::GeneratorConfig defaultConfig()
{
  tg::GeneratorConfig cfg;
  cfg.params.max_speed = 5.0;
  cfg.params.max_acceleration = 4.0;
  cfg.params.max_jerk = 20.0;
  cfg.params.acceptance_radius = 1.0;          // L1 corner-cut on intermediates.
  cfg.params.final_acceptance_radius = 0.05;   // tight stop at the goal.
  cfg.params.trajectory_gain = 1.0;
  cfg.simulator.step_dt = 0.01;
  cfg.simulator.max_simulation_time = 60.0;
  cfg.simulator.settle_velocity = 0.05;
  cfg.simulator.polynomial_order = 16;
  return cfg;
}

// Returns the minimum Euclidean distance from any sample of the trajectory
// to the given target position over the [0, duration] horizon.
double minDistanceToTarget(
  const tg::TrajectoryGenerator & gen, const Eigen::Vector3d & target)
{
  const double t0 = gen.minTime();
  const double t1 = gen.maxTime();
  const double dt = 0.01;
  double best = std::numeric_limits<double>::infinity();
  for (double t = t0; t <= t1; t += dt) {
    const Eigen::Vector3d p = gen.position(t);
    best = std::min(best, (p - target).norm());
  }
  // Always sample the exact endpoint as well so the final waypoint is
  // captured even when (t1 - t0) is not an integer multiple of dt.
  const Eigen::Vector3d p_end = gen.position(t1);
  best = std::min(best, (p_end - target).norm());
  return best;
}

void assertFidelity(
  const tg::TrajectoryGenerator & gen,
  const std::vector<Eigen::Vector3d> & user_waypoints,
  const std::string & label)
{
  ASSERT_FALSE(user_waypoints.empty()) << label;
  for (std::size_t i = 0; i < user_waypoints.size(); ++i) {
    const double d = minDistanceToTarget(gen, user_waypoints[i]);
    const bool is_last = (i + 1 == user_waypoints.size());
    const double tol =
      is_last ? kFinalWaypointTolerance : kIntermediateWaypointTolerance;
    EXPECT_LE(d, tol)
      << label << ": waypoint #" << i
      << " (" << user_waypoints[i].transpose() << ") was missed by "
      << d << " m (tol " << tol << " m)";
  }
}

}  // namespace

TEST(JerkLimitedFidelity, StraightLineVerticalClimb)
{
  // The drone takes off near (0,0,0) and must reach (0,0,2). With the legacy
  // single-knob behaviour and acceptance_radius = 1.0, the integrator could
  // declare itself settled barely off the ground; the new
  // final_acceptance_radius forces it to actually reach the goal.
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d goal(0.0, 0.0, 2.0);
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(start));
  wps.emplace_back(tg::EndWaypoint(goal));

  tg::TrajectoryGenerator gen(defaultConfig());
  ASSERT_TRUE(gen.generate(wps, defaultConfig().params.max_speed));

  assertFidelity(gen, {goal}, "StraightLineVerticalClimb");
}

TEST(JerkLimitedFidelity, HorizontalZigzag)
{
  // Five waypoints zig-zagging in y at constant z. With acceptance_radius=1.0
  // the L1 corner-cut rounds intermediate corners, so the path geometry must
  // give the cut some room (legs >= 5 m, amplitude <= 1 m) — tighter zigzags
  // are outside the well-behaved regime of the upstream L1 corner-cut. The
  // last waypoint must still be reached within kFinalWaypointTolerance even
  // though the L1 path through the previous corner ends ~1 m off the
  // straight line.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 1.0},
    {5.0, 1.0, 1.0},
    {10.0, -1.0, 1.0},
    {15.0, 1.0, 1.0},
    {20.0, 0.0, 1.0},
  };
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(path.front()));
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    wps.emplace_back(tg::Waypoint(path[i]));
  }
  wps.emplace_back(tg::EndWaypoint(path.back()));

  tg::TrajectoryGenerator gen(defaultConfig());
  ASSERT_TRUE(gen.generate(wps, defaultConfig().params.max_speed));

  assertFidelity(gen, path, "HorizontalZigzag");
}

TEST(JerkLimitedFidelity, SquareFourCorner)
{
  // Closed 10 m square at constant altitude — resembles the user's
  // mission.py PATH issued as a sequence of go_to_point calls. Verifies the
  // generator handles 90 deg turns and returns to the start.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 1.0},
    {10.0, 0.0, 1.0},
    {10.0, 10.0, 1.0},
    {0.0, 10.0, 1.0},
    {0.0, 0.0, 1.0},
  };
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(path.front()));
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    wps.emplace_back(tg::Waypoint(path[i]));
  }
  wps.emplace_back(tg::EndWaypoint(path.back()));

  tg::TrajectoryGenerator gen(defaultConfig());
  ASSERT_TRUE(gen.generate(wps, defaultConfig().params.max_speed));

  assertFidelity(gen, path, "SquareFourCorner");
}

TEST(JerkLimitedFidelity, ColinearVerticalSegment)
{
  // Two consecutive waypoints share xy (purely vertical motion) before
  // moving sideways. The integrator must terminate at the actual goal and
  // not mistake "near the intermediate vertical waypoint" for completion.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 2.0},
    {2.0, 0.0, 2.0},
  };
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(path.front()));
  wps.emplace_back(tg::Waypoint(path[1]));
  wps.emplace_back(tg::EndWaypoint(path.back()));

  tg::TrajectoryGenerator gen(defaultConfig());
  ASSERT_TRUE(gen.generate(wps, defaultConfig().params.max_speed));

  assertFidelity(gen, path, "ColinearVerticalSegment");
}

TEST(JerkLimitedFidelity, TightTrackingZigzag)
{
  // Regression guard for the `simulator.advance_radius_min` parameter.
  // The default 0.1 m floor is what makes intermediate waypoints get missed
  // by ~10 cm even with `acceptance_radius` set to 1 cm. By lowering the
  // floor to 5 mm and raising `trajectory_gain` to keep the corner-speed
  // cap near `max_speed`, the trajectory must pass close to every
  // intermediate waypoint.
  tg::GeneratorConfig cfg = defaultConfig();
  cfg.params.acceptance_radius = 0.01;
  cfg.params.trajectory_gain = 10.0;
  cfg.simulator.advance_radius_min = 0.005;

  // Mild zigzag: 5 m legs, 0.5 m amplitude. The corner-speed cap with this
  // config is sqrt(max_a * gain * R * tan(alpha/2)) >> max_speed, so the
  // bottleneck on corner velocity is `max_speed` itself.
  const std::vector<Eigen::Vector3d> path = {
    {0.0, 0.0, 1.0},
    {5.0, 0.5, 1.0},
    {10.0, -0.5, 1.0},
    {15.0, 0.5, 1.0},
    {20.0, 0.0, 1.0},
  };
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(path.front()));
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    wps.emplace_back(tg::Waypoint(path[i]));
  }
  wps.emplace_back(tg::EndWaypoint(path.back()));

  tg::TrajectoryGenerator gen(cfg);
  ASSERT_TRUE(gen.generate(wps, cfg.params.max_speed));

  // Tighter than the legacy intermediate tolerance: with the floor lowered,
  // every intermediate must be within 0.05 m of the trajectory.
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = minDistanceToTarget(gen, path[i]);
    const bool is_last = (i + 1 == path.size());
    const double tol = is_last ? 0.05 : 0.05;
    EXPECT_LE(d, tol)
      << "TightTrackingZigzag: waypoint #" << i
      << " (" << path[i].transpose() << ") was missed by "
      << d << " m (tol " << tol << " m)";
  }
}
