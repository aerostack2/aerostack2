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
 * @file dynamic_mav_trajectory_generator.cpp
 *
 * @brief Plugin wrapper around dynamic_trajectory_generator.
 *
 * @authors Rafael Perez-Segui
 */

#include "dynamic_mav_trajectory_generator/dynamic_mav_trajectory_generator.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace dynamic_mav_trajectory_generator
{

namespace
{
bool isAuxiliaryWaypointId(const std::string & id)
{
  // Internal helper ids are excluded from mission progress reporting.
  return id.empty() || id == "stitching_point" || id == "current";
}
}  // namespace

Plugin::Plugin()
: trajectory_generator_(
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>()) {}

void Plugin::ownInitialize()
{
  // Route DYNAMIC_LOG / COUNT_TIME messages emitted by the upstream
  // dynamic_trajectory_generator library to the rclcpp DEBUG-severity logger
  // named "dynamic_traj_generator". Visibility is then controlled at runtime
  // with: --ros-args --log-level dynamic_traj_generator:=debug
  dynamic_traj_generator::setLogSink(
    [](const std::string & msg) {
      RCLCPP_INFO(
        rclcpp::get_logger("dynamic_traj_generator"), "%s", msg.c_str());
    });

  // Low-speed segment factors
  getParameter<double>("ls_velocity_factor", ls_velocity_factor_, true);
  getParameter<double>("ls_acceleration_factor", ls_acceleration_factor_, true);
  RCLCPP_INFO(
    getNodePtr()->get_logger(),
    "Low-speed factors: v_factor=%.2f, a_factor=%.2f",
    ls_velocity_factor_, ls_acceleration_factor_);
}

std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
Plugin::makeTrajectoryGenerator() const
{
  auto generator =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  generator->setLowSpeedVelocityFactor(ls_velocity_factor_);
  generator->setLowSpeedAccelerationFactor(ls_acceleration_factor_);
  return generator;
}

dynamic_traj_generator::DynamicWaypoint::Deque Plugin::toDynamicDeque(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints)
{
  dynamic_traj_generator::DynamicWaypoint::Deque out;
  for (const auto & wp : waypoints) {
    // Preserve waypoint names so behavior feedback can map progress by id.
    dynamic_traj_generator::DynamicWaypoint dyn_wp;
    dyn_wp.setName(wp.id);
    dyn_wp.resetWaypoint(
      Eigen::Vector3d(
        wp.pose.pose.position.x,
        wp.pose.pose.position.y,
        wp.pose.pose.position.z));
    out.emplace_back(std::move(dyn_wp));
  }
  return out;
}

void Plugin::pushVehiclePositionToGenerator(
  const geometry_msgs::msg::PoseStamped & pose)
{
  trajectory_generator_->updateVehiclePosition(
    Eigen::Vector3d(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
}

bool Plugin::generateTrajectory(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
  double max_speed,
  double t_trajectory_now)
{
  // Recreate backend per goal to avoid stale trajectory state.
  trajectory_generator_ = makeTrajectoryGenerator();
  trajectory_generator_->setSpeed(max_speed);

  // Seed generator vehicle state from the live pose published by the
  // host. The backend prepends an internal "current" waypoint with zero
  // velocity / acceleration constraints, so vehicle_twist_ is not
  // applied here.
  pushVehiclePositionToGenerator(vehicle_pose_);

  auto deque = toDynamicDeque(waypoints);
  dynamic_traj_generator::DynamicWaypoint::Vector vector;
  vector.reserve(deque.size());
  for (auto & wp : deque) {
    vector.emplace_back(std::move(wp));
  }
  trajectory_generator_->setWaypoints(vector);

  // Anchor the host axis to the backend axis: when t_trajectory ==
  // t_trajectory_now, t_backend == backend_min_time.
  internal_offset_ =
    trajectory_generator_->getMinTime() - t_trajectory_now;
  return true;
}

bool Plugin::evaluate(
  double t_trajectory,
  as2_msgs::msg::TrajectoryPoint & out,
  bool is_horizon_sample)
{
  if (!is_horizon_sample) {
    // Inject live vehicle position only for control setpoint evaluation.
    // Horizon predictions and debug sampling must not mutate generator
    // state, so for_plotting=true skips the internal time advance.
    pushVehiclePositionToGenerator(vehicle_pose_);
  }

  const double t_backend = t_trajectory + internal_offset_;

  dynamic_traj_generator::References refs;
  const bool ok = trajectory_generator_->evaluateTrajectory(
    t_backend, refs, /*only_pos=*/ false, /*for_plotting=*/ is_horizon_sample);
  if (!ok) {
    return false;
  }

  out.position.x = refs.position.x();
  out.position.y = refs.position.y();
  out.position.z = refs.position.z();
  out.twist.x = refs.velocity.x();
  out.twist.y = refs.velocity.y();
  out.twist.z = refs.velocity.z();
  out.acceleration.x = refs.acceleration.x();
  out.acceleration.y = refs.acceleration.y();
  out.acceleration.z = refs.acceleration.z();
  return true;
}

bool Plugin::isFinished(double t_trajectory) const
{
  if (!trajectory_generator_) {
    return true;
  }
  return (t_trajectory + internal_offset_) >=
         trajectory_generator_->getMaxTime();
}

double Plugin::getDuration() const
{
  if (!trajectory_generator_) {
    return 0.0;
  }
  return trajectory_generator_->getMaxTime() -
         trajectory_generator_->getMinTime();
}

bool Plugin::isTrajectoryGenerated()
{
  return trajectory_generator_ && trajectory_generator_->getMaxTime() > 0.0;
}

void Plugin::reset()
{
  trajectory_generator_ = makeTrajectoryGenerator();
  internal_offset_ = 0.0;
}

bool Plugin::updateWaypoints(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
  double /*max_speed*/,
  double /*t_trajectory_now*/)
{
  // Cruise speed is fixed at the previous generateTrajectory(); the
  // backend does not expose a per-update setter so max_speed is
  // intentionally ignored here. The backend stitches in place over its
  // continuous time axis, so internal_offset_ is preserved unchanged.
  if (!trajectory_generator_ || waypoints.empty()) {
    return false;
  }

  dynamic_traj_generator::DynamicWaypoint::Vector remaining;
  remaining.reserve(waypoints.size());
  for (const auto & wp : waypoints) {
    if (isAuxiliaryWaypointId(wp.id)) {
      continue;
    }
    dynamic_traj_generator::DynamicWaypoint dyn_wp;
    dyn_wp.setName(wp.id);
    dyn_wp.resetWaypoint(
      Eigen::Vector3d(
        wp.pose.pose.position.x,
        wp.pose.pose.position.y,
        wp.pose.pose.position.z));
    remaining.emplace_back(std::move(dyn_wp));
  }

  if (remaining.empty()) {
    return false;
  }

  trajectory_generator_->setWaypoints(remaining);
  return true;
}

std::string Plugin::getNextWaypointId()
{
  if (!trajectory_generator_) {
    return {};
  }

  for (const auto & waypoint :
    trajectory_generator_->getNextTrajectoryWaypoints())
  {
    if (!isAuxiliaryWaypointId(waypoint.getName())) {
      return waypoint.getName();
    }
  }
  return {};
}

bool Plugin::consumeRegeneratedFlag()
{
  if (!trajectory_generator_) {
    return false;
  }
  return trajectory_generator_->getWasTrajectoryRegenerated();
}

}  // namespace dynamic_mav_trajectory_generator

PLUGINLIB_EXPORT_CLASS(
  dynamic_mav_trajectory_generator::Plugin,
  generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase)
