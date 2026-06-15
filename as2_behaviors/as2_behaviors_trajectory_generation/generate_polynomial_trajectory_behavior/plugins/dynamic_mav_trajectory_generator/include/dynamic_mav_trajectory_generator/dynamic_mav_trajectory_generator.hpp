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
 * @file dynamic_mav_trajectory_generator.hpp
 *
 * @brief Plugin wrapper around the dynamic_trajectory_generator library.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef DYNAMIC_MAV_TRAJECTORY_GENERATOR__DYNAMIC_MAV_TRAJECTORY_GENERATOR_HPP_
#define DYNAMIC_MAV_TRAJECTORY_GENERATOR__DYNAMIC_MAV_TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

namespace dynamic_mav_trajectory_generator
{

/**
 * @brief Plugin wrapper for DynamicTrajectory backend.
 */
class Plugin : public generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase
{
public:
  /**
   * @brief Construct plugin instance.
   */
  Plugin();

  /**
   * @brief Virtual destructor.
   */
  ~Plugin() override = default;

  /**
   * @brief Generate trajectory from waypoint list.
   *
   * The starting position is read from vehicle_pose_ (refreshed by the
   * host) and pushed into the backend before scheduling the new
   * trajectory. The DynamicTrajectory backend forces zero velocity at
   * the starting waypoint when generating from scratch, so the live
   * vehicle_twist_ is intentionally ignored on this path; smooth
   * velocity matching is only available through updateWaypoints() after
   * a previous trajectory has been generated.
   *
   * Anchors the internal time-axis offset so that the host's
   * t_trajectory_now maps to the backend's getMinTime().
   *
   * @param waypoints Mission waypoints (no synthetic "current" entry).
   * @param max_speed Maximum speed in m/s.
   * @param t_trajectory_now Current host trajectory time (seconds).
   * @return true when generation succeeds.
   */
  bool generateTrajectory(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed,
    double t_trajectory_now) override;

  /**
   * @brief Evaluate references at host trajectory time @p t_trajectory.
   *
   * Reads the live vehicle pose from the protected base member
   * (vehicle_pose_) so the dynamic backend can close the loop on the
   * actual vehicle state. The injection is gated on @p is_horizon_sample
   * being false: for horizon predictions and debug sampling the call is
   * non-mutating (for_plotting=true).
   *
   * @param t_trajectory Evaluation time in the host's trajectory axis.
   * @param out Output trajectory point.
   * @param is_horizon_sample True for horizon predictions and debug
   *                          sampling; false for the live control setpoint.
   * @return true when evaluation succeeds.
   */
  bool evaluate(
    double t_trajectory,
    as2_msgs::msg::TrajectoryPoint & out,
    bool is_horizon_sample) override;

  /**
   * @brief Whether the trajectory is exhausted at the given t_trajectory.
   *
   * @param t_trajectory Current host trajectory time in seconds.
   * @return true when the trajectory has finished.
   */
  bool isFinished(double t_trajectory) const override;
  double getDuration() const override;

  /**
   * @brief Check if a trajectory is currently available.
   *
   * @return true when trajectory was generated.
   */
  bool isTrajectoryGenerated() override;

  /**
   * @brief Reset trajectory generator state.
   */
  void reset() override;

  /**
   * @brief Update the active trajectory using the backend stitching logic.
   *
   * Overrides the base default (regenerate) to leverage
   * DynamicTrajectory's automatic stitching, which evaluates the
   * existing trajectory at the current time and uses position +
   * velocity + acceleration of that sample as boundary conditions for
   * the new trajectory. The backend keeps its internal time origin so
   * the plugin's internal_offset_ is preserved unchanged and the host's
   * t_trajectory keeps mapping to the same backend trajectory.
   *
   * @param waypoints Updated pending waypoints (mission only).
   * @param max_speed Maximum cruise speed in m/s.
   * @param t_trajectory_now Current host trajectory time (unused on the
   *                  stitching path, but kept for API symmetry).
   * @return true when the backend accepts the update request.
   */
  bool updateWaypoints(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed,
    double t_trajectory_now) override;

  /**
   * @brief Consume and return the backend's "trajectory regenerated"
   * flag.
   *
   * Delegates to DynamicTrajectory::getWasTrajectoryRegenerated(), which
   * is set to true by swapTrajectory() (the deferred event triggered by
   * an asynchronous updateWaypoints()) and consumed (reset to false) on
   * read.
   *
   * @return true at most once per asynchronous regeneration event.
   */
  bool consumeRegeneratedFlag() override;

  /**
   * @brief Get the id of the next pending mission waypoint.
   *
   * Auxiliary backend waypoints (e.g. stitching points) are filtered out.
   *
   * @return Empty string when no pending waypoint, else its id.
   */
  std::string getNextWaypointId() override;

protected:
  /**
   * @brief Optional plugin-specific initialization hook.
   */
  void ownInitialize() override;

private:
  /**
   * @brief Convert AS2 waypoints to dynamic generator deque.
   *
   * @param waypoints Input waypoint list.
   * @return Converted dynamic waypoint deque.
   */
  static dynamic_traj_generator::DynamicWaypoint::Deque toDynamicDeque(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints);

  /**
   * @brief Push vehicle position into generator state.
   *
   * @param pose Current vehicle pose.
   */
  void
  pushVehiclePositionToGenerator(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Build a fresh DynamicTrajectory
   *
   * @return A new generator with the low-speed factors set.
   */
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
  makeTrajectoryGenerator() const;

  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
  trajectory_generator_;

  // Low-speed segment factors
  double ls_velocity_factor_{1.0};
  double ls_acceleration_factor_{1.0};

  // Maps host trajectory time to backend time:
  //   t_backend = t_trajectory + internal_offset_.
  // Anchored on generateTrajectory(); preserved across stitched
  // updateWaypoints() so the backend's continuous time axis stays
  // aligned with the host's t_trajectory.
  double internal_offset_{0.0};
};

}  // namespace dynamic_mav_trajectory_generator

#endif  // DYNAMIC_MAV_TRAJECTORY_GENERATOR__DYNAMIC_MAV_TRAJECTORY_GENERATOR_HPP_
