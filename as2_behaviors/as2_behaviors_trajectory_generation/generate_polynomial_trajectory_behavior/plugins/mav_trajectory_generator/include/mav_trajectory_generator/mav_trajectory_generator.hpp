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
 * @file mav_trajectory_generator.hpp
 *
 * @brief Plugin wrapper around mav_trajectory_generation_cpp::TrajectoryGenerator.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef MAV_TRAJECTORY_GENERATOR__MAV_TRAJECTORY_GENERATOR_HPP_
#define MAV_TRAJECTORY_GENERATOR__MAV_TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

#include "mav_trajectory_generation_cpp/trajectory_generator.hpp"
#include "mav_trajectory_generation_cpp/types.hpp"

namespace mav_trajectory_generator
{

/**
 * @brief Plugin wrapper for the static (offline) ETH-ASL
 * mav_trajectory_generation backend.
 *
 * The plugin does not override updateWaypoints(): each modify request
 * goes through the base class default, which calls reset() +
 * generateTrajectory() with the live vehicle pose and twist as
 * boundary conditions, so the resulting trajectory is C1-continuous
 * with the drone state at the time of the re-plan.
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

protected:
  void ownInitialize() override;

private:
  void readConfigParameters();

  static mav_trajectory_generation_cpp::Waypoint toMavWaypoint(
    const as2_msgs::msg::PoseStampedWithID & wp);

  mav_trajectory_generation_cpp::GeneratorConfig generator_config_;
  std::unique_ptr<mav_trajectory_generation_cpp::TrajectoryGenerator>
  trajectory_generator_;

  // Mission progress: ordered (waypoint_id, t_arrival) for each waypoint
  // beyond the implicit start. Walked by getNextWaypointId() against
  // last_eval_backend_time_, which tracks the latest non-horizon
  // evaluation time in the backend's own time axis.
  std::vector<std::pair<std::string, double>> waypoint_arrival_times_;
  double last_eval_backend_time_{0.0};

  // Maps host trajectory time to backend time:
  //   t_backend = t_trajectory + internal_offset_.
  // Anchored on every generateTrajectory().
  double internal_offset_{0.0};
};

}  // namespace mav_trajectory_generator

#endif  // MAV_TRAJECTORY_GENERATOR__MAV_TRAJECTORY_GENERATOR_HPP_
