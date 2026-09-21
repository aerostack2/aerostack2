// Copyright 2024 Universidad Politécnica de Madrid
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
* @file filter.cpp
*
* The simple_ekf filter implementation
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#include "simple_ekf_core/filter.hpp"

#include <algorithm>
#include <array>
#include <utility>

#include "simple_ekf_core/measurement_utils.hpp"
#include "simple_ekf_core/transform_utils.hpp"

namespace simple_ekf_core
{

namespace
{

// Seconds and nanoseconds of a stamp, for printing it the way ROS prints one
int wholeSeconds(Nanoseconds stamp)
{
  return static_cast<int>(stamp / 1000000000);
}

int remainingNanoseconds(Nanoseconds stamp)
{
  return static_cast<int>(stamp % 1000000000);
}

}  // namespace

Filter::Filter(const Config & config, LogSink log)
: logger_(std::move(log)),
  config_(validated(config, logger_)),
  ekf_history_buffer_(ekf_wrapper_, config_.max_update_latency_ms, config_.unobserved_variance),
  preflight_source_(SourceConfig{"<pre-flight correction>"})
{
  setupWrapper();
}

Config Filter::validated(const Config & config, const Logger & logger)
{
  Config result = config;

  if (result.unobserved_variance <= 0.0) {
    logger.log(
      LogLevel::WARN,
      "unobserved_variance is %g, which marks a component unobserved rather than standing in "
      "for one. Using %g", result.unobserved_variance, 1.0e2);
    result.unobserved_variance = 1.0e2;
  } else if (result.unobserved_variance > 1.0e4) {
    logger.log(
      LogLevel::WARN,
      "unobserved_variance is %g. Above about %g the innovation covariance loses its "
      "significant digits against measurement variances of 1e-3 and the filter diverges",
      result.unobserved_variance, 1.0e4);
  }

  if (result.map_odom_alpha <= 0.0 || result.map_odom_alpha > 1.0) {
    logger.log(
      LogLevel::WARN, "map_odom_alpha is %g, but must be in (0, 1]. Using 0.1",
      result.map_odom_alpha);
    result.map_odom_alpha = 0.1;
  }

  // A non-positive variance would read as "not measured", and the correction would do nothing
  if (result.preflight_variance <= 0.0) {
    logger.log(
      LogLevel::WARN, "preflight_variance is %g, but must be positive. Using 1e-5",
      result.preflight_variance);
    result.preflight_variance = 1e-5;
  }

  return result;
}

void Filter::setupWrapper()
{
  ekf::Covariance initial_covariance;
  const std::array<std::pair<int, double>, 5> groups = {{
    {ekf::State::X, config_.initial_position_covariance},
    {ekf::State::VX, config_.initial_velocity_covariance},
    {ekf::State::ROLL, config_.initial_orientation_covariance},
    {ekf::State::ABX, config_.initial_bias_acc_covariance},
    {ekf::State::WBX, config_.initial_bias_gyro_covariance}}};
  for (const auto & [first_state, variance] : groups) {
    for (int state = first_state; state < first_state + 3; ++state) {
      initial_covariance.data[varianceIndex(state)] = variance;
    }
  }
  ekf_wrapper_.reset(ekf::State(), initial_covariance);

  ekf_wrapper_.set_gravity(ekf::Gravity({0.0, 0.0, config_.gravity}));

  ekf_wrapper_.set_noise_parameters(
    Eigen::Vector<double, 6>::Zero(),
    config_.accelerometer_noise_density, config_.gyroscope_noise_density,
    config_.accelerometer_random_walk, config_.gyroscope_random_walk);
}

SourceId Filter::addSource(const SourceConfig & config)
{
  sources_.emplace_back(config);
  return sources_.size() - 1;
}

bool Filter::setEarthToMapFromFirstPose(const Rigid & pose, SourceFrame frame)
{
  switch (frame) {
    case SourceFrame::EARTH:
      outputs_.earth_to_map = pose;
      return true;
    case SourceFrame::MAP:
      outputs_.earth_to_map = pose.inverse();
      resetStateToPose(pose);
      return true;
    default:
      return false;
  }
}

void Filter::resetStateToPose(const Rigid & pose_in_map)
{
  const std::array<double, 3> angles = toRollPitchYaw(pose_in_map.getRotation());

  ekf::State state;
  state.data[ekf::State::X] = pose_in_map.getOrigin().x();
  state.data[ekf::State::Y] = pose_in_map.getOrigin().y();
  state.data[ekf::State::Z] = pose_in_map.getOrigin().z();
  state.data[ekf::State::ROLL] = angles[0];
  state.data[ekf::State::PITCH] = angles[1];
  state.data[ekf::State::YAW] = angles[2];
  ekf_wrapper_.set_state(state);

  // Start the smoothing from the new pose instead of blending away from a stale one
  output_blend_initialized_ = false;

  if (config_.verbose) {
    logger_.log(
      LogLevel::INFO,
      "Reset EKF state to first received pose: "
      "[x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]",
      state.data[ekf::State::X], state.data[ekf::State::Y], state.data[ekf::State::Z],
      state.data[ekf::State::ROLL], state.data[ekf::State::PITCH], state.data[ekf::State::YAW]);
  }
}

void Filter::setOffboard(bool offboard)
{
  drone_offboard_ = offboard;
  drone_has_been_offboard_ = drone_has_been_offboard_ || offboard;
}

bool Filter::shouldThrottleUpdate(SourceId source_id, Nanoseconds stamp)
{
  SourceState & source = sources_.at(source_id);
  if (source.config.update_rate_hz <= 0.0) {
    return false;
  }

  if (source.last_fused_stamp &&
    toSeconds(stamp - *source.last_fused_stamp) < 1.0 / source.config.update_rate_hz)
  {
    return true;
  }

  source.last_fused_stamp = stamp;
  return false;
}

bool Filter::isRepeatedPosition(SourceId source_id, const Vector3 & position, Nanoseconds now)
{
  SourceState & source = sources_.at(source_id);
  if (!source.config.reject_repeated_positions) {
    return false;
  }

  if (source.last_position &&
    isSamePosition(position, *source.last_position, source.config.repeated_position_threshold))
  {
    if (source.repeated_position_warning.allow(now)) {
      logger_.log(
        LogLevel::WARN,
        "Dropping a measurement from source '%s': same position as the last one received",
        source.config.name.c_str());
    }
    return true;
  }

  source.last_position = position;
  return false;
}

void Filter::onImu(const ImuSample & imu)
{
  const ekf::Input input(
    {
      imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2],
      imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]});

  double dt = 0.0;
  if (last_imu_.stamp != 0) {
    dt = toSeconds(imu.stamp - last_imu_.stamp);
  } else if (config_.verbose) {
    logger_.log(LogLevel::WARN, "Received first IMU message, initializing EKF state");
  }

  ekf_history_buffer_.predictAndRecord(imu.stamp, input, dt);

  if (config_.debug_verbose) {
    logger_.log(
      LogLevel::INFO,
      "Processed IMU message [%.6f, %.6f, %.6f] for EKF prediction with dt = %.6f seconds",
      imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2], dt);
  }

  // Stored before updateOutputs(), which publishes this reading's angular velocity
  last_imu_ = imu;
  updateOutputs();
}

void Filter::onPose(SourceId source_id, const PoseSample & pose, Nanoseconds now)
{
  processPose(sources_.at(source_id), pose, now);
  updateOutputs();
}

void Filter::onTwist(SourceId source_id, const TwistSample & twist, Nanoseconds now)
{
  processTwist(sources_.at(source_id), twist, now);
  updateOutputs();
}

bool Filter::onTick(Nanoseconds now)
{
  if (!earth_to_map_set_) {
    return false;
  }

  stepOutputBlend();

  if (drone_offboard_ || drone_has_been_offboard_) {
    return false;
  }

  // An absolute assertion of where the drone is standing: it moves map->odom, and it is not
  // gated, since its point is to pull a drifted state back
  PoseSample standing;
  standing.stamp = now;
  standing.frame = SourceFrame::MAP;
  standing.pose = config_.preflight_pose;
  standing.covariance.fill(config_.preflight_variance);
  processPose(preflight_source_, standing, now);
  updateOutputs();

  if (config_.debug_verbose) {
    logger_.log(LogLevel::WARN, "Offboard is false, applied the pre-flight correction");
  }
  return true;
}

void Filter::updateOutputs()
{
  const ekf::State & state = ekf_wrapper_.get_state();

  outputs_.map_to_odom = eigenMatrix4dToRigid(ekf_wrapper_.get_map_to_odom());
  const StateTransforms transforms(state, outputs_.map_to_odom);
  outputs_.odom_to_base = transforms.odom_to_base;

  // What is published combines the smoothed map->odom with the raw odom->base: map->odom
  // carries every EKF correction, odom->base is the smooth dead-reckoned part
  const Rigid published_map_to_base = outputs_.published_map_to_odom * outputs_.odom_to_base;

  // For the twist to match the published pose, the raw map->odom velocity is swapped for
  // the smoothed one
  const auto velocity = state.get_velocity();
  const Vector3 velocity_in_map_raw(velocity[0], velocity[1], velocity[2]);
  const Eigen::Vector3d & map_to_odom_velocity = ekf_wrapper_.get_map_to_odom_velocity();
  const Vector3 velocity_in_map = velocity_in_map_raw -
    Vector3(map_to_odom_velocity.x(), map_to_odom_velocity.y(), map_to_odom_velocity.z()) +
    published_map_to_odom_velocity_;

  outputs_.twist_in_base = ekfStateToTwist(
    state, published_map_to_base, last_imu_.angular_velocity, velocity_in_map);
  outputs_.internal_twist_in_base = ekfStateToTwist(
    state, transforms.map_to_base, last_imu_.angular_velocity, velocity_in_map_raw);
}

void Filter::stepOutputBlend()
{
  const Rigid raw_map_to_odom = eigenMatrix4dToRigid(ekf_wrapper_.get_map_to_odom());
  const Eigen::Vector3d & velocity = ekf_wrapper_.get_map_to_odom_velocity();
  const Vector3 raw_velocity(velocity.x(), velocity.y(), velocity.z());

  // Seeded from the raw value, so the first steps don't blend up from identity
  if (!output_blend_initialized_) {
    outputs_.published_map_to_odom = raw_map_to_odom;
    published_map_to_odom_velocity_ = raw_velocity;
    output_blend_initialized_ = true;
    return;
  }

  outputs_.published_map_to_odom = blendTransforms(
    outputs_.published_map_to_odom, raw_map_to_odom, config_.map_odom_alpha);
  published_map_to_odom_velocity_ = blendVectors(
    published_map_to_odom_velocity_, raw_velocity, config_.map_odom_alpha);
}

void Filter::warnIfZeroVariance(
  SourceState & source, const std::array<double, 36> & covariance, Nanoseconds now)
{
  const std::array<bool, 6> zeroed = zeroVarianceComponents(covariance);
  if (std::any_of(zeroed.begin(), zeroed.end(), [](bool flag) {return flag;}) &&
    source.zero_variance_warning.allow(now))
  {
    logger_.log(
      LogLevel::WARN,
      "Source '%s' carries a variance of exactly zero, so those components are being "
      "ignored. A source that publishes no covariance needs use_message_covariance: false",
      source.config.name.c_str());
  }
}

void Filter::processPose(SourceState & source, const PoseSample & pose, Nanoseconds now)
{
  if (config_.debug_verbose) {
    logger_.log(
      LogLevel::WARN, "Processing pose measurement at time %d.%09d",
      wholeSeconds(pose.stamp), remainingNanoseconds(pose.stamp));
  }

  // The measurement is moved into the map frame with the newest state, even a late one:
  // only the correction itself is replayed at the measurement's stamp
  const ekf::State current_state = ekf_wrapper_.get_state();
  const StateTransforms transforms(current_state, outputs_.map_to_odom);

  // Non-positive variances become usable numbers before the rotation, and which ones is kept
  const std::array<bool, 6> unobserved = unobservedComponents(pose.covariance);
  warnIfZeroVariance(source, pose.covariance, now);
  PoseSample measurement = pose;
  resolveUnobservedVariances(measurement.covariance, config_.unobserved_variance);

  const PoseSample measurement_in_map =
    transformPoseToMapFrame(transforms, outputs_.earth_to_map, measurement);

  // Recorded as measured: the buffer neutralises and unwraps it again against the state the
  // correction lands on, which for a late measurement is not the current one
  const ekf::PoseMeasurement recorded_measurement =
    poseToRawEkfMeasurement(measurement_in_map.pose);
  const ekf::PoseMeasurementCovariance recorded_covariance =
    covarianceToEkfMeasurementCovariance(measurement_in_map.covariance);

  // Gated against the newest state, the same approximation as the frame transform above
  ekf::PoseMeasurement measurement_now = recorded_measurement;
  ekf::PoseMeasurementCovariance covariance_now = recorded_covariance;
  neutraliseUnobservedComponents(
    measurement_now, covariance_now, unobserved, current_state, config_.unobserved_variance);
  const ekf::PoseMeasurement unwrapped_now = unwrapPoseMeasurement(measurement_now, current_state);

  const ekf::Covariance & state_covariance = ekf_wrapper_.get_state_covariance();
  std::array<double, 6> innovations;
  std::array<double, 6> state_variances;
  std::array<double, 6> measurement_variances;
  for (std::size_t i = 0; i < kPoseStateIndices.size(); ++i) {
    const int state_index = kPoseStateIndices[i];
    innovations[i] = unwrapped_now.data[i] - current_state.data[state_index];
    state_variances[i] = state_covariance.data[varianceIndex(state_index)];
    measurement_variances[i] = covariance_now.data[i];
  }
  if (!acceptsInnovation(
      source, innovations, state_variances, measurement_variances, pose.stamp, now))
  {
    return;
  }

  if (config_.debug_verbose) {
    const auto & z = measurement_now.data;
    const auto & r = covariance_now.data;
    logger_.log(
      LogLevel::INFO,
      "Raw pose measurement (map frame, wrapped):"
      "[x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]",
      z[0], z[1], z[2], z[3], z[4], z[5]);
    logger_.log(
      LogLevel::INFO,
      "Pose measurement covariance:"
      "[c_x=%.6f, c_y=%.6f, c_z=%.6f, c_roll=%.6f, c_pitch=%.6f, c_yaw=%.6f]",
      r[0], r[1], r[2], r[3], r[4], r[5]);
  }

  const EkfOperationType type = source.config.is_odometry ?
    EkfOperationType::UPDATE_POSE_ODOM : EkfOperationType::UPDATE_POSE;
  const UpdateResult result = ekf_history_buffer_.updateAndRecord(
    pose.stamp, type, recorded_measurement, recorded_covariance, now, unobserved);

  if (!result.applied && source.stale_measurement_warning.allow(now)) {
    logger_.log(
      LogLevel::WARN,
      "Dropping a pose measurement from source '%s': %.3f s old, more than "
      "max_update_latency_ms (%.0f ms)",
      source.config.name.c_str(), toSeconds(now - pose.stamp), config_.max_update_latency_ms);
  }
}

void Filter::processTwist(SourceState & source, const TwistSample & twist, Nanoseconds now)
{
  const ekf::State current_state = ekf_wrapper_.get_state();
  const StateTransforms transforms(current_state, outputs_.map_to_odom);

  const std::array<bool, 6> unobserved = unobservedComponents(twist.covariance);
  warnIfZeroVariance(source, twist.covariance, now);
  TwistSample measurement = twist;
  resolveUnobservedVariances(measurement.covariance, config_.unobserved_variance);

  const TwistSample measurement_in_map =
    transformTwistToMapFrame(transforms, outputs_.earth_to_map, measurement);

  const ekf::VelocityMeasurement recorded_velocity =
    twistToEkfVelocityMeasurement(measurement_in_map);
  const ekf::VelocityMeasurementCovariance recorded_covariance =
    twistToEkfVelocityCovariance(measurement_in_map);

  ekf::VelocityMeasurement velocity_now = recorded_velocity;
  ekf::VelocityMeasurementCovariance covariance_now = recorded_covariance;
  neutraliseUnobservedVelocityComponents(
    velocity_now, covariance_now, unobserved, current_state, config_.unobserved_variance);

  const ekf::Covariance & state_covariance = ekf_wrapper_.get_state_covariance();
  std::array<double, 3> innovations;
  std::array<double, 3> state_variances;
  std::array<double, 3> measurement_variances;
  for (std::size_t i = 0; i < kVelocityStateIndices.size(); ++i) {
    const int state_index = kVelocityStateIndices[i];
    innovations[i] = velocity_now.data[i] - current_state.data[state_index];
    state_variances[i] = state_covariance.data[varianceIndex(state_index)];
    measurement_variances[i] = covariance_now.data[i];
  }
  if (!acceptsInnovation(
      source, innovations, state_variances, measurement_variances, twist.stamp, now))
  {
    return;
  }

  if (config_.debug_verbose) {
    logger_.log(
      LogLevel::INFO,
      "Velocity measurement (map frame): [vx=%.3f, vy=%.3f, vz=%.3f], "
      "covariance [%.5f, %.5f, %.5f]",
      velocity_now.data[0], velocity_now.data[1], velocity_now.data[2],
      covariance_now.data[0], covariance_now.data[1], covariance_now.data[2]);
  }

  const UpdateResult result = ekf_history_buffer_.updateAndRecord(
    twist.stamp, recorded_velocity, recorded_covariance, now, unobserved);

  if (!result.applied && source.stale_measurement_warning.allow(now)) {
    logger_.log(
      LogLevel::WARN,
      "Dropping a velocity measurement from source '%s': %.3f s old, more than "
      "max_update_latency_ms (%.0f ms)",
      source.config.name.c_str(), toSeconds(now - twist.stamp), config_.max_update_latency_ms);
  }
}

}  // namespace simple_ekf_core
