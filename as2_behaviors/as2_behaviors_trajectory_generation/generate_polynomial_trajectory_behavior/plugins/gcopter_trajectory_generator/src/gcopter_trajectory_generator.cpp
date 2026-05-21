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
 * @file gcopter_trajectory_generator.cpp
 *
 * @brief Plugin wrapper around gcopter_lib::TrajectoryGenerator.
 *
 * @authors Rafael Perez-Segui
 */

#include "gcopter_trajectory_generator/gcopter_trajectory_generator.hpp"

#include <algorithm>
#include <cstddef>

#include <pluginlib/class_list_macros.hpp>

namespace gcopter_trajectory_generator
{

Plugin::Plugin() = default;

void Plugin::ownInitialize()
{
  readConfigParameters();
}

void Plugin::readConfigParameters()
{
  // DroneParameters has no struct-level defaults, so seed reasonable values
  // before reading. They survive the read when the user does not override.
  auto & params = generator_config_.params;
  params.mass = 1.5;
  params.gravity = 9.81;
  params.horizontal_drag = 0.1;
  params.vertical_drag = 0.1;
  params.parasitic_drag = 0.01;
  params.speed_smooth_factor = 0.01;
  getParameter<double>("drone.mass", params.mass, true);
  getParameter<double>("drone.gravity", params.gravity, true);
  getParameter<double>("drone.horizontal_drag", params.horizontal_drag, true);
  getParameter<double>("drone.vertical_drag", params.vertical_drag, true);
  getParameter<double>("drone.parasitic_drag", params.parasitic_drag, true);
  getParameter<double>(
    "drone.speed_smooth_factor", params.speed_smooth_factor, true);

  auto & limits = generator_config_.limits;
  limits.max_velocity = 5.0;
  limits.max_body_rate = 6.0;
  limits.max_tilt_angle = 0.785;
  limits.min_thrust = 0.1;
  limits.max_thrust = 30.0;
  getParameter<double>("limits.max_velocity", limits.max_velocity, true);
  getParameter<double>("limits.max_body_rate", limits.max_body_rate, true);
  getParameter<double>("limits.max_tilt_angle", limits.max_tilt_angle, true);
  getParameter<double>("limits.min_thrust", limits.min_thrust, true);
  getParameter<double>("limits.max_thrust", limits.max_thrust, true);

  auto & opt = generator_config_.optimization;
  // OptimizationConfig already has reasonable struct-level defaults; only
  // expose the knobs that callers typically tune.
  getParameter<double>("optimization.time_weight", opt.time_weight, true);
  getParameter<double>(
    "optimization.position_weight", opt.position_weight, true);
  getParameter<double>(
    "optimization.velocity_weight", opt.velocity_weight, true);
  getParameter<double>(
    "optimization.body_rate_weight", opt.body_rate_weight, true);
  getParameter<double>("optimization.tilt_weight", opt.tilt_weight, true);
  getParameter<double>("optimization.thrust_weight", opt.thrust_weight, true);
  getParameter<double>("optimization.smoothing_eps", opt.smoothing_eps, true);
  getParameter<int>(
    "optimization.integral_resolution", opt.integral_resolution, true);
  getParameter<double>("optimization.rel_cost_tol", opt.rel_cost_tol, true);
  getParameter<double>(
    "optimization.corridor_margin", opt.corridor_margin, true);
  getParameter<double>(
    "optimization.vertical_perturbation", opt.vertical_perturbation, true);
  getParameter<double>(
    "optimization.vertical_alignment_threshold",
    opt.vertical_alignment_threshold, true);
  // length_per_piece intentionally omitted: keep INF default so each segment
  // maps to one waypoint pair (assumed by the arrival-time bookkeeping below).

  // Anchor-expansion knobs. Inserting anchor points around each intermediate
  // waypoint and using a tighter margin on the pinch segments forces the
  // optimised trajectory to pass close to every user waypoint, which
  // otherwise are only used to grow the AABB Safe Flight Corridor.
  // Setting either to <= 0 disables anchoring and falls back to a uniform
  // corridor margin (`optimization.corridor_margin`).
  getParameter<double>(
    "waypoints.waypoint_margin", waypoint_margin_, true);
  getParameter<double>(
    "waypoints.waypoint_anchor_radius", waypoint_anchor_radius_, true);
}

gcopter_lib::Waypoint Plugin::toGcopterWaypoint(
  const as2_msgs::msg::PoseStampedWithID & wp)
{
  gcopter_lib::Waypoint out;
  out.position = Eigen::Vector3d(
    wp.pose.pose.position.x,
    wp.pose.pose.position.y,
    wp.pose.pose.position.z);
  // velocity / acceleration left as nullopt → zero at endpoints, free at
  // intermediates (gcopter_lib::Waypoint contract).
  return out;
}

Plugin::ExpandedPath Plugin::expandPathWithAnchors(
  const std::vector<Eigen::Vector3d> & waypoints,
  double loose_margin,
  double tight_margin,
  double anchor_radius)
{
  ExpandedPath out;
  const std::size_t n_wps = waypoints.size();

  // Bail out early when anchoring cannot or should not run: not enough
  // waypoints to have intermediates, or one of the knobs is disabled.
  if (n_wps < 3 || tight_margin <= 0.0 || anchor_radius <= 0.0) {
    out.positions = waypoints;
    out.original_indices.resize(n_wps);
    for (std::size_t i = 0; i < n_wps; ++i) {
      out.original_indices[i] = i;
    }
    if (n_wps >= 2) {
      out.margins.assign(n_wps - 1, loose_margin);
    }
    return out;
  }

  out.positions.reserve(3 * (n_wps - 2) + 2);
  out.original_indices.reserve(n_wps);

  out.positions.emplace_back(waypoints.front());
  out.original_indices.emplace_back(0);

  for (std::size_t i = 1; i + 1 < n_wps; ++i) {
    const Eigen::Vector3d & prev = waypoints[i - 1];
    const Eigen::Vector3d & curr = waypoints[i];
    const Eigen::Vector3d & next = waypoints[i + 1];

    const Eigen::Vector3d in_vec = curr - prev;
    const Eigen::Vector3d out_vec = next - curr;
    const double in_norm = in_vec.norm();
    const double out_norm = out_vec.norm();
    if (in_norm <= 0.0 || out_norm <= 0.0) {
      // Degenerate: two consecutive waypoints coincide. Skip anchoring
      // around this waypoint and keep the waypoint itself in the path.
      out.positions.emplace_back(curr);
      out.original_indices.emplace_back(out.positions.size() - 1);
      continue;
    }
    // Skip anchoring when the entry and exit directions are (nearly)
    // collinear: there is no corner to cut, the waypoint sits on the
    // straight line and the AABB Safe Flight Corridor of the surrounding
    // segments already passes through its vicinity. Anchoring collinear
    // waypoints would otherwise generate stacks of thin pinch corridors
    // (e.g. on a chain of purely vertical waypoints) that the L-BFGS
    // solver tends to reject as ill-conditioned.
    constexpr double kCollinearSinThreshold = 0.05;  // ~3 deg
    const double sin_corner =
      in_vec.cross(out_vec).norm() / (in_norm * out_norm);
    if (sin_corner < kCollinearSinThreshold) {
      out.positions.emplace_back(curr);
      out.original_indices.emplace_back(out.positions.size() - 1);
      continue;
    }
    // Clamp the radius so consecutive anchors never cross each other.
    const double r_in = std::min(anchor_radius, 0.45 * in_norm);
    const double r_out = std::min(anchor_radius, 0.45 * out_norm);

    out.positions.emplace_back(curr - in_vec * (r_in / in_norm));
    out.positions.emplace_back(curr);
    out.original_indices.emplace_back(out.positions.size() - 1);
    out.positions.emplace_back(curr + out_vec * (r_out / out_norm));
  }

  out.positions.emplace_back(waypoints.back());
  out.original_indices.emplace_back(out.positions.size() - 1);

  // Per-segment margins. With at least one anchor inserted, the layout is:
  //   [loose] (wp_0 -> first anchor)
  //   for each intermediate wp i: [tight, tight] then a [loose] in between.
  //   [loose] (last anchor -> wp_{N-1})
  // Use the actual emitted positions to derive segment margins so the
  // skipped-anchor case (coincident waypoints) is handled correctly.
  const std::size_t n_segments = out.positions.size() - 1;
  out.margins.reserve(n_segments);

  // For each emitted segment, mark it tight if either endpoint is an
  // intermediate user waypoint position. The tight margin should apply to
  // exactly the two pinch segments (anchor->wp and wp->anchor) around each
  // intermediate waypoint.
  std::vector<bool> is_intermediate(out.positions.size(), false);
  for (std::size_t k = 1; k + 1 < out.original_indices.size(); ++k) {
    is_intermediate[out.original_indices[k]] = true;
  }
  for (std::size_t s = 0; s < n_segments; ++s) {
    const bool tight = is_intermediate[s] || is_intermediate[s + 1];
    out.margins.emplace_back(tight ? tight_margin : loose_margin);
  }
  return out;
}

bool Plugin::generateTrajectory(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
  double max_speed,
  double t_trajectory_now)
{
  waypoint_arrival_times_.clear();
  last_eval_backend_time_ = 0.0;

  if (waypoints.empty() || max_speed <= 0.0) {
    trajectory_generator_.reset();
    return false;
  }

  // Recreate backend per goal to avoid stale trajectory state.
  trajectory_generator_ =
    std::make_unique<gcopter_lib::TrajectoryGenerator>(generator_config_);

  // Build the raw position list: the live vehicle pose is prepended as the
  // trajectory start so the new plan is C1-continuous with the drone state.
  // The end velocity at the last waypoint defaults to zero (rest at goal).
  std::vector<Eigen::Vector3d> raw_positions;
  raw_positions.reserve(waypoints.size() + 1);
  raw_positions.emplace_back(
    vehicle_pose_.pose.position.x,
    vehicle_pose_.pose.position.y,
    vehicle_pose_.pose.position.z);
  for (const auto & wp : waypoints) {
    raw_positions.emplace_back(
      wp.pose.pose.position.x,
      wp.pose.pose.position.y,
      wp.pose.pose.position.z);
  }

  // Insert anchor points around each intermediate waypoint and assign per
  // segment corridor margins (tight on pinch segments). The host's wide
  // AABB Safe Flight Corridor leaves the trajectory free to cut corners
  // through intermediate waypoints; anchoring forces it to pass close to
  // each one. A single uniform margin is used as a fallback when anchoring
  // is disabled or the path has fewer than three points.
  const ExpandedPath expanded = expandPathWithAnchors(
    raw_positions,
    generator_config_.optimization.corridor_margin,
    waypoint_margin_,
    waypoint_anchor_radius_);

  std::vector<gcopter_lib::Waypoint> wps;
  wps.reserve(expanded.positions.size());
  // Pin the velocity boundary condition only on the very first waypoint
  // (the synthetic start), matching the live vehicle twist. All other
  // waypoints — including the goal — keep the default rest constraint.
  for (std::size_t i = 0; i < expanded.positions.size(); ++i) {
    gcopter_lib::Waypoint wp;
    wp.position = expanded.positions[i];
    if (i == 0) {
      wp.velocity = Eigen::Vector3d(
        vehicle_twist_.twist.linear.x,
        vehicle_twist_.twist.linear.y,
        vehicle_twist_.twist.linear.z);
    }
    wps.emplace_back(std::move(wp));
  }

  const bool ok = trajectory_generator_->generate(
    wps, max_speed, expanded.margins);
  if (!ok) {
    const auto & p0 = wps.front().position;
    const auto & v0 = wps.front().velocity;
    const Eigen::Vector3d v0_val = v0.has_value() ? *v0 : Eigen::Vector3d::Zero();
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "gcopter: backend generate() failed — n_wps=%zu, max_speed=%f, "
      "start_pos=(%f, %f, %f), start_vel=(%f, %f, %f)%s",
      wps.size(), max_speed, p0.x(), p0.y(), p0.z(),
      v0_val.x(), v0_val.y(), v0_val.z(),
      v0.has_value() ? "" : " [vel unset]");
    return false;
  }

  // Map mission waypoint ids to backend spline breakpoint times. The
  // expansion may have inserted anchor points, so the indices held by
  // expanded.original_indices identify the breakpoints that correspond
  // to actual user waypoints. expanded.original_indices[0] is the
  // synthetic vehicle start; entries [1..N] map to waypoints[0..N-1].
  const auto bps = trajectory_generator_->spline().breakpoints();
  if (!bps.empty() && expanded.original_indices.size() == waypoints.size() + 1) {
    const double t0 = bps.front();
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      const std::size_t bp_idx = expanded.original_indices[i + 1];
      if (bp_idx < bps.size()) {
        waypoint_arrival_times_.emplace_back(
          waypoints[i].id, bps[bp_idx] - t0);
      }
    }
  }

  // Anchor host axis to backend axis: t_trajectory_now -> minTime() == 0.
  internal_offset_ = trajectory_generator_->minTime() - t_trajectory_now;
  return true;
}

bool Plugin::evaluate(
  double t_trajectory,
  as2_msgs::msg::TrajectoryPoint & out,
  bool is_horizon_sample)
{
  if (!trajectory_generator_ || !trajectory_generator_->isValid()) {
    return false;
  }

  const double t_backend = std::clamp(
    t_trajectory + internal_offset_,
    trajectory_generator_->minTime(), trajectory_generator_->maxTime());

  const auto sample = trajectory_generator_->evaluate(t_backend);
  out.position.x = sample.position.x();
  out.position.y = sample.position.y();
  out.position.z = sample.position.z();
  out.twist.x = sample.velocity.x();
  out.twist.y = sample.velocity.y();
  out.twist.z = sample.velocity.z();
  out.acceleration.x = sample.acceleration.x();
  out.acceleration.y = sample.acceleration.y();
  out.acceleration.z = sample.acceleration.z();

  if (!is_horizon_sample) {
    last_eval_backend_time_ = t_backend;
  }
  return true;
}

bool Plugin::isFinished(double t_trajectory) const
{
  if (!trajectory_generator_ || !trajectory_generator_->isValid()) {
    return true;
  }
  return (t_trajectory + internal_offset_) >= trajectory_generator_->maxTime();
}

double Plugin::getDuration() const
{
  if (!trajectory_generator_ || !trajectory_generator_->isValid()) {
    return 0.0;
  }
  return trajectory_generator_->maxTime() - trajectory_generator_->minTime();
}

bool Plugin::isTrajectoryGenerated()
{
  return trajectory_generator_ && trajectory_generator_->isValid();
}

void Plugin::reset()
{
  trajectory_generator_.reset();
  waypoint_arrival_times_.clear();
  last_eval_backend_time_ = 0.0;
  internal_offset_ = 0.0;
}

std::string Plugin::getNextWaypointId()
{
  for (const auto & [id, t_arrival] : waypoint_arrival_times_) {
    if (t_arrival > last_eval_backend_time_) {
      return id;
    }
  }
  return {};
}

}  // namespace gcopter_trajectory_generator

PLUGINLIB_EXPORT_CLASS(
  gcopter_trajectory_generator::Plugin,
  generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase)
