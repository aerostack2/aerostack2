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
* @file ekf_history_buffer.hpp
*
* Chronological buffer of EKF operations supporting out-of-sequence
* (delayed) pose/odom/mocap/velocity updates via rewind + replay
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__EKF_HISTORY_BUFFER_HPP_
#define SIMPLE_EKF_CORE__EKF_HISTORY_BUFFER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <utility>

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>

#include "simple_ekf_core/measurement_utils.hpp"
#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

enum class EkfOperationType
{
  PREDICT,
  UPDATE_POSE,
  UPDATE_POSE_ODOM,
  UPDATE_VELOCITY
};

/**
 * @brief One recorded EKF operation, with the state it was applied to
 *
 * Invariant: applying an entry's operation to its (state_before, covariance_before) gives
 * the next entry's, or the wrapper's current ones for the last entry.
 */
struct EkfTimelineEntry
{
  Nanoseconds stamp = 0;
  ekf::State state_before;
  ekf::Covariance covariance_before;
  EkfOperationType type = EkfOperationType::PREDICT;

  // PREDICT
  ekf::Input imu_input;
  double dt = 0.0;

  // UPDATE_POSE and UPDATE_POSE_ODOM. The angles as measured, each in [-pi, pi]: they are
  // unwrapped again on every replay, since a replay can change the state they unwrap against.
  ekf::PoseMeasurement raw_pose_measurement;
  ekf::PoseMeasurementCovariance pose_measurement_covariance;

  // UPDATE_VELOCITY, already in the map frame
  ekf::VelocityMeasurement velocity_measurement;
  ekf::VelocityMeasurementCovariance velocity_measurement_covariance;

  // Components the source does not measure, neutralised when the correction is applied
  std::array<bool, 6> unobserved{};
};

struct UpdateResult
{
  bool applied = false;
};

/**
 * @brief Chronological buffer of EKF operations, for fusing measurements that arrive late
 *
 * A late measurement is applied at its own timestamp: the EKF is rewound to the state it
 * had then, the correction is applied, and every operation recorded after it is replayed.
 * Operations older than the latency limit are forgotten.
 *
 * Operates on a wrapper it does not own, which must outlive it. Not thread-safe.
 */
class EkfHistoryBuffer
{
public:
  /**
   * @param max_update_latency_ms Age beyond which a measurement is dropped as stale
   * @param unobserved_variance Variance given to the components a source does not measure
   */
  EkfHistoryBuffer(
    ekf::EKFWrapper & wrapper, double max_update_latency_ms, double unobserved_variance)
  : wrapper_(wrapper),
    max_update_latency_(fromSeconds(max_update_latency_ms / 1000.0)),
    unobserved_variance_(unobserved_variance)
  {
  }

  /**
   * @brief Predict with an IMU reading and record it.
   *
   * @param stamp Time of the reading, which is also the "now" old entries are trimmed against
   */
  void predictAndRecord(Nanoseconds stamp, const ekf::Input & input, double dt)
  {
    EkfTimelineEntry entry;
    entry.stamp = stamp;
    entry.state_before = wrapper_.get_state();
    entry.covariance_before = wrapper_.get_state_covariance();
    entry.type = EkfOperationType::PREDICT;
    entry.imu_input = input;
    entry.dt = dt;

    wrapper_.predict(input, dt);

    buffer_.push_back(std::move(entry));
    trim(stamp);
  }

  /**
   * @brief Correct with a pose measurement taken at `stamp`, and record it.
   *
   * @param type UPDATE_POSE moves map->odom by the correction, UPDATE_POSE_ODOM does not
   * @param raw_measurement Pose in the map frame, angles not unwrapped
   * @param now Current time, against which the measurement's age is judged
   * @param unobserved Components the source does not measure
   * @return applied is false if the measurement was too old, and nothing changed
   */
  UpdateResult updateAndRecord(
    Nanoseconds stamp, EkfOperationType type,
    const ekf::PoseMeasurement & raw_measurement,
    const ekf::PoseMeasurementCovariance & measurement_cov,
    Nanoseconds now,
    const std::array<bool, 6> & unobserved = {})
  {
    EkfTimelineEntry entry;
    entry.type = type;
    entry.raw_pose_measurement = raw_measurement;
    entry.pose_measurement_covariance = measurement_cov;
    entry.unobserved = unobserved;
    return insertAndReplay(stamp, std::move(entry), now);
  }

  /**
   * @brief Correct with a velocity measurement taken at `stamp`, and record it.
   *
   * @param measurement Velocity in the map frame
   * @return applied is false if the measurement was too old, and nothing changed
   */
  UpdateResult updateAndRecord(
    Nanoseconds stamp,
    const ekf::VelocityMeasurement & measurement,
    const ekf::VelocityMeasurementCovariance & measurement_cov,
    Nanoseconds now,
    const std::array<bool, 6> & unobserved = {})
  {
    EkfTimelineEntry entry;
    entry.type = EkfOperationType::UPDATE_VELOCITY;
    entry.velocity_measurement = measurement;
    entry.velocity_measurement_covariance = measurement_cov;
    entry.unobserved = unobserved;
    return insertAndReplay(stamp, std::move(entry), now);
  }

  std::size_t size() const
  {
    return buffer_.size();
  }

  const EkfTimelineEntry & at(std::size_t i) const
  {
    return buffer_.at(i);
  }

private:
  ekf::EKFWrapper & wrapper_;
  Nanoseconds max_update_latency_;
  double unobserved_variance_;
  std::deque<EkfTimelineEntry> buffer_;

  /**
   * @brief Apply a recorded correction on top of the wrapper's current state.
   *
   * Pose corrections go through update_pose_odom, which never moves map->odom:
   * insertAndReplay moves it once, by the net effect of the whole replay.
   *
   * @param reference_state The state the correction lands on, which the unobserved
   *        components are taken from and the angles are unwrapped against
   */
  void applyUpdate(const EkfTimelineEntry & entry, const ekf::State & reference_state)
  {
    if (entry.type == EkfOperationType::UPDATE_VELOCITY) {
      ekf::VelocityMeasurement measurement = entry.velocity_measurement;
      ekf::VelocityMeasurementCovariance covariance = entry.velocity_measurement_covariance;
      neutraliseUnobservedVelocityComponents(
        measurement, covariance, entry.unobserved, reference_state, unobserved_variance_);
      wrapper_.update_velocity(measurement, covariance);
      return;
    }

    // Before the unwrap, so that a component standing in for the state unwraps to itself
    ekf::PoseMeasurement measurement = entry.raw_pose_measurement;
    ekf::PoseMeasurementCovariance covariance = entry.pose_measurement_covariance;
    neutraliseUnobservedComponents(
      measurement, covariance, entry.unobserved, reference_state, unobserved_variance_);

    wrapper_.update_pose_odom(unwrapPoseMeasurement(measurement, reference_state), covariance);
  }

  /**
   * @brief Insert a correction at its place in the timeline and replay what follows it.
   */
  UpdateResult insertAndReplay(Nanoseconds stamp, EkfTimelineEntry entry, Nanoseconds now)
  {
    if (now - stamp > max_update_latency_) {
      return {false};
    }

    // The first entry newer than the measurement, where the rewind goes to
    const auto newer = std::upper_bound(
      buffer_.begin(), buffer_.end(), stamp,
      [](Nanoseconds s, const EkfTimelineEntry & e) {return s < e.stamp;});
    const std::size_t index = static_cast<std::size_t>(std::distance(buffer_.begin(), newer));

    // Nothing newer is the common, non-delayed case: the rewind point is the current state
    const bool delayed = index < buffer_.size();
    const ekf::State rewind_state =
      delayed ? buffer_[index].state_before : wrapper_.get_state();
    const ekf::Covariance rewind_cov =
      delayed ? buffer_[index].covariance_before : wrapper_.get_state_covariance();

    const ekf::State state_now_before = wrapper_.get_state();

    wrapper_.reset(rewind_state, rewind_cov);
    applyUpdate(entry, rewind_state);

    const EkfOperationType type = entry.type;
    entry.stamp = stamp;
    entry.state_before = rewind_state;
    entry.covariance_before = rewind_cov;
    buffer_.insert(buffer_.begin() + index, std::move(entry));

    for (std::size_t i = index + 1; i < buffer_.size(); ++i) {
      buffer_[i].state_before = wrapper_.get_state();
      buffer_[i].covariance_before = wrapper_.get_state_covariance();

      if (buffer_[i].type == EkfOperationType::PREDICT) {
        wrapper_.predict(buffer_[i].imu_input, buffer_[i].dt);
      } else {
        applyUpdate(buffer_[i], buffer_[i].state_before);
      }
    }

    // Only an absolute pose moves map->odom, by the net change the whole replay made
    if (type == EkfOperationType::UPDATE_POSE) {
      const ekf::State state_now_after = wrapper_.get_state();
      wrapper_.set_map_to_odom(
        ekf::EKFWrapper::compute_map_to_odom(
          state_now_before, state_now_after, wrapper_.get_map_to_odom()));
      wrapper_.set_map_to_odom_velocity(
        ekf::EKFWrapper::compute_map_to_odom_velocity(
          state_now_before, state_now_after, wrapper_.get_map_to_odom_velocity()));
    }

    trim(now);
    return {true};
  }

  /**
   * @brief Forget what is older than the latency limit, always keeping one entry at or
   *        before it for a rewind to land on.
   */
  void trim(Nanoseconds now)
  {
    const Nanoseconds horizon = now - max_update_latency_;
    while (buffer_.size() > 1 && buffer_[1].stamp < horizon) {
      buffer_.pop_front();
    }
  }
};

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__EKF_HISTORY_BUFFER_HPP_
