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
 * @file mav_trajectory_generator.cpp
 *
 * @brief Plugin wrapper around mav_trajectory_generation_cpp::TrajectoryGenerator.
 *
 * @authors Rafael Perez-Segui
 */

#include "mav_trajectory_generator/mav_trajectory_generator.hpp"

#include <algorithm>
#include <cctype>

#include <pluginlib/class_list_macros.hpp>

namespace mav_trajectory_generator
{

namespace
{
mav_trajectory_generation_cpp::Solver parseSolver(const std::string & raw)
{
  std::string s = raw;
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
  if (s == "nonlinear") {
    return mav_trajectory_generation_cpp::Solver::Nonlinear;
  }
  return mav_trajectory_generation_cpp::Solver::Linear;
}
}  // namespace

Plugin::Plugin() = default;

void Plugin::ownInitialize()
{
  readConfigParameters();
}

void Plugin::readConfigParameters()
{
  auto & opt = generator_config_.optimization;

  std::string solver_str = "linear";
  getParameter<std::string>("optimization.solver", solver_str, true);
  opt.solver = parseSolver(solver_str);

  getParameter<int>(
    "optimization.derivative_to_optimize", opt.derivative_to_optimize, true);
  getParameter<double>("optimization.a_max", opt.a_max, true);
  getParameter<int>(
    "optimization.nl_max_iterations", opt.nl_max_iterations, true);
  getParameter<double>("optimization.nl_f_rel", opt.nl_f_rel, true);
  getParameter<double>("optimization.nl_x_rel", opt.nl_x_rel, true);
  getParameter<double>(
    "optimization.nl_time_penalty", opt.nl_time_penalty, true);
  getParameter<double>(
    "optimization.nl_initial_stepsize_rel", opt.nl_initial_stepsize_rel, true);
  getParameter<double>(
    "optimization.nl_inequality_constraint_tolerance",
    opt.nl_inequality_constraint_tolerance, true);
}

mav_trajectory_generation_cpp::Waypoint Plugin::toMavWaypoint(
  const as2_msgs::msg::PoseStampedWithID & wp)
{
  mav_trajectory_generation_cpp::Waypoint out;
  out.position = Eigen::Vector3d(
    wp.pose.pose.position.x,
    wp.pose.pose.position.y,
    wp.pose.pose.position.z);
  // Endpoints default to zero velocity / acceleration when fields are unset
  // (mav_trajectory_generation_cpp::Waypoint contract).
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
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "mav_trajectory: generateTrajectory failed — waypoints.empty()=%d, "
      "max_speed=%f", static_cast<int>(waypoints.empty()), max_speed);
    trajectory_generator_.reset();
    return false;
  }

  // Recreate backend per goal to avoid stale trajectory state.
  trajectory_generator_ =
    std::make_unique<mav_trajectory_generation_cpp::TrajectoryGenerator>(
    generator_config_);

  // Prepend the current vehicle state as the trajectory start. Position
  // and velocity become hard boundary conditions for the optimizer (the
  // optional velocity field is honored when set on the first waypoint).
  // This guarantees C1 continuity at the start when re-planning while
  // the drone is already in motion.
  std::vector<mav_trajectory_generation_cpp::Waypoint> wps;
  wps.reserve(waypoints.size() + 1);
  mav_trajectory_generation_cpp::Waypoint start;
  start.position = Eigen::Vector3d(
    vehicle_pose_.pose.position.x,
    vehicle_pose_.pose.position.y,
    vehicle_pose_.pose.position.z);
  start.velocity = Eigen::Vector3d(
    vehicle_twist_.twist.linear.x,
    vehicle_twist_.twist.linear.y,
    vehicle_twist_.twist.linear.z);
  wps.emplace_back(std::move(start));
  for (const auto & wp : waypoints) {
    wps.emplace_back(toMavWaypoint(wp));
  }

  if (!trajectory_generator_->generate(wps, max_speed)) {
    const auto & p0 = wps.front().position;
    const auto & v0 = wps.front().velocity;
    const Eigen::Vector3d v0_val = v0.has_value() ? *v0 : Eigen::Vector3d::Zero();
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "mav_trajectory: backend generate() failed — n_wps=%zu, max_speed=%f, "
      "start_pos=(%f, %f, %f), start_vel=(%f, %f, %f)%s",
      wps.size(), max_speed, p0.x(), p0.y(), p0.z(),
      v0_val.x(), v0_val.y(), v0_val.z(),
      v0.has_value() ? "" : " [vel unset]");
    return false;
  }

  // Map mission waypoint ids to spline breakpoint times. wps[0] is the
  // synthetic start, so bps[i+1] corresponds to waypoints[i].
  const auto bps = trajectory_generator_->spline().breakpoints();
  if (!bps.empty()) {
    const double t0 = bps.front();
    for (std::size_t i = 1; i < bps.size() && (i - 1) < waypoints.size();
      ++i)
    {
      waypoint_arrival_times_.emplace_back(
        waypoints[i - 1].id, bps[i] - t0);
    }
  }
  // Anchor host axis to backend axis: t_trajectory_now -> minTime().
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

}  // namespace mav_trajectory_generator

PLUGINLIB_EXPORT_CLASS(
  mav_trajectory_generator::Plugin,
  generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase)
