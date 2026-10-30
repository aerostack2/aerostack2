// Copyright 2026 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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
 * @file DriftOdometry.hpp
 *
 * Gazebo system that publishes odometry which drifts over time, like a VIO
 * or LiDAR odometry source, to test state estimation in simulation.
 *
 * It reads the pose and velocity of the model's canonical link every physics
 * step, runs them through DriftOdometryModel and publishes
 * gz::msgs::OdometryWithCovariance at a fixed rate.
 *
 * SDF parameters (all optional):
 *   <topic>                  default /model/<model>/odometry_drift
 *   <odom_frame_id>          default <model>/odom
 *   <base_frame_id>          default <model>/base_link
 *   <publish_rate>           [Hz] default 100
 *   <seed>                   0 picks a random seed
 *   <linear_velocity_noise>, <linear_bias_noise>, <yaw_rate_noise>,
 *   <yaw_rate_bias_noise>, <scale_error>, <twist_linear_noise>,
 *   <twist_angular_noise>    see DriftOdometryParams
 */

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFTODOMETRY_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFTODOMETRY_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "drift_odometry_model.hpp"

namespace as2_gazebo_assets
{

class DriftOdometryPublisher
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & event_manager) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::unique_ptr<DriftOdometryModel> drift_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  std::chrono::steady_clock::duration publish_period_{};
  std::chrono::steady_clock::duration last_publish_time_{};
};

}  // namespace as2_gazebo_assets

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFTODOMETRY_HPP_
