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
 * @file gcopter_trajectory_generator.hpp
 *
 * @brief Plugin wrapper around gcopter_lib::TrajectoryGenerator.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef GCOPTER_TRAJECTORY_GENERATOR__GCOPTER_TRAJECTORY_GENERATOR_HPP_
#define GCOPTER_TRAJECTORY_GENERATOR__GCOPTER_TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

#include "gcopter_lib/trajectory_generator.hpp"
#include "gcopter_lib/types.hpp"

namespace gcopter_trajectory_generator
{

/**
 * @brief Plugin wrapper for gcopter_lib::TrajectoryGenerator backend.
 *
 * The optimizer is offline and produces a piecewise polynomial spline
 * that can be evaluated continuously. The plugin does not override
 * updateWaypoints(): each modify request goes through the base class
 * default, which calls reset() + generateTrajectory() with the live
 * vehicle pose and twist as boundary conditions, so the resulting
 * trajectory is C1-continuous with the drone state at the time of the
 * re-plan.
 */
class Plugin : public generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase
{
public:
  Plugin();
  ~Plugin() override = default;

  bool generateTrajectory(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed,
    double t_trajectory_now) override;

  bool evaluate(
    double t_trajectory,
    as2_msgs::msg::TrajectoryPoint & out,
    bool is_horizon_sample) override;

  bool isFinished(double t_trajectory) const override;
  double getDuration() const override;
  bool isTrajectoryGenerated() override;
  void reset() override;

  std::string getNextWaypointId() override;

  /**
   * @brief Path with waypoint anchors and per-segment corridor margins.
   *
   * Used internally to insert anchor points around each user waypoint to
   * tighten the AABB Safe Flight Corridor near them, while keeping a loose
   * margin between anchors. `original_indices` stores the position of each
   * input waypoint inside @ref positions so the host can map mission ids
   * back to backend breakpoints.
   */
  struct ExpandedPath
  {
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> margins;
    std::vector<std::size_t> original_indices;
  };

  /**
   * @brief Insert anchor points around intermediate waypoints.
   *
   * For N >= 3 input waypoints, places one anchor at distance min(@p
   * anchor_radius, 0.45 * leg_length) before and after each intermediate
   * waypoint along the entry/exit directions. The two pinch segments that
   * touch each waypoint receive @p tight_margin while the rest of the
   * corridor uses @p loose_margin. For coincident consecutive waypoints
   * the anchor is skipped (no useful direction). For N < 3 or when
   * anchoring is disabled, returns the original waypoints with a uniform
   * @p loose_margin per segment.
   *
   * Exposed publicly to keep it unit-testable without instantiating the
   * full ROS plugin.
   *
   * @param waypoints     Ordered waypoint positions [m].
   * @param loose_margin  Default AABB half-margin per face [m].
   * @param tight_margin  Half-margin for the pinch segments [m]. <=0
   *                      disables anchoring.
   * @param anchor_radius Offset of each anchor along the path [m]. <=0
   *                      disables anchoring.
   * @return Expanded path with per-segment margins and a mapping back to
   *         the original waypoint indices.
   */
  static ExpandedPath expandPathWithAnchors(
    const std::vector<Eigen::Vector3d> & waypoints,
    double loose_margin,
    double tight_margin,
    double anchor_radius);

protected:
  void ownInitialize() override;

private:
  void readConfigParameters();

  static gcopter_lib::Waypoint toGcopterWaypoint(
    const as2_msgs::msg::PoseStampedWithID & wp);

  gcopter_lib::GeneratorConfig generator_config_;
  std::unique_ptr<gcopter_lib::TrajectoryGenerator> trajectory_generator_;

  // Anchor expansion knobs read from the YAML configuration. Both must be
  // strictly positive for anchoring to engage; otherwise the plugin falls
  // back to a uniform corridor margin.
  double waypoint_margin_{0.1};
  double waypoint_anchor_radius_{0.5};

  // Mission progress: ordered (waypoint_id, t_arrival) for each waypoint
  // beyond the implicit start. Walked by getNextWaypointId() against
  // last_eval_backend_time_, which tracks the latest non-horizon
  // evaluation time in the backend's own time axis (anchored at
  // minTime()=0 for gcopter, so t_arrival = bps[i] directly).
  std::vector<std::pair<std::string, double>> waypoint_arrival_times_;
  double last_eval_backend_time_{0.0};

  // Maps host trajectory time to backend time:
  //   t_backend = t_trajectory + internal_offset_.
  // Anchored on every generateTrajectory() so the host's t_trajectory_now
  // maps to the backend's minTime() == 0.
  double internal_offset_{0.0};
};

}  // namespace gcopter_trajectory_generator

#endif  // GCOPTER_TRAJECTORY_GENERATOR__GCOPTER_TRAJECTORY_GENERATOR_HPP_
