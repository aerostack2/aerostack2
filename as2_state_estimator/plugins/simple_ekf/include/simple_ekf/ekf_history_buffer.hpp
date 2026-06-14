// Copyright 2025 Universidad Politécnica de Madrid
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
* (delayed) pose/odom/mocap updates via rewind + replay
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF__EKF_HISTORY_BUFFER_HPP_
#define SIMPLE_EKF__EKF_HISTORY_BUFFER_HPP_

#include <algorithm>
#include <cstddef>
#include <deque>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>

#include "simple_ekf/simple_ekf_utils.hpp"

namespace simple_ekf
{

/**
 * @brief Type of operation recorded in an EkfHistoryBuffer timeline entry
 */
enum class EkfOperationType
{
  PREDICT,
  UPDATE_POSE,
  UPDATE_POSE_ODOM
};

/**
 * @brief One recorded EKF operation (IMU predict or pose/odom update)
 *
 * Invariant: applying `type`'s operation (with the type-specific payload below)
 * to (state_before, covariance_before) produces exactly the (state_before,
 * covariance_before) of the next entry in the buffer, or the EKFWrapper's
 * current state/covariance for the last entry.
 */
struct EkfTimelineEntry
{
  rclcpp::Time stamp;
  ekf::State state_before;
  ekf::Covariance covariance_before;
  EkfOperationType type = EkfOperationType::PREDICT;

  // PREDICT only
  ekf::Input imu_input;
  double dt = 0.0;

  // UPDATE_POSE / UPDATE_POSE_ODOM only. Raw measurement: roll/pitch/yaw as
  // returned by tf2::Matrix3x3::getRPY() (wrapped to [-pi, pi]), NOT unwrapped
  // relative to any state. Re-unwrapped on every (re)application, since replay
  // may change state_before and thus the correct unwrap branch.
  ekf::PoseMeasurement raw_pose_measurement;
  ekf::PoseMeasurementCovariance pose_measurement_covariance;
};

/**
 * @brief Result of an EkfHistoryBuffer::updateAndRecord() call
 */
struct UpdateResult
{
  bool applied = false;
};

/**
 * @brief Chronological buffer of EKF operations supporting out-of-sequence
 * (delayed) pose/odom/mocap updates via rewind + replay.
 *
 * Holds a reference to the EKFWrapper it operates on. Not thread-safe —
 * intended for use from a single-threaded executor only.
 */
class EkfHistoryBuffer
{
public:
  /**
   * @brief Construct the buffer
   *
   * @param wrapper Reference to the EKFWrapper instance to operate on. Must
   *        outlive this object.
   * @param max_update_latency_ms Maximum age (milliseconds) of a pose/odom/mocap
   *        measurement before it is dropped as stale.
   */
  EkfHistoryBuffer(ekf::EKFWrapper & wrapper, double max_update_latency_ms)
  : wrapper_(wrapper),
    max_update_latency_(rclcpp::Duration::from_seconds(max_update_latency_ms / 1000.0))
  {
  }

  /**
   * @brief Record an IMU predict step.
   *
   * Snapshots the pre-predict state/covariance, calls wrapper.predict(input, dt),
   * appends a PREDICT entry, and trims the buffer front.
   *
   * @param stamp Timestamp of the IMU message (used for ordering and as the trim
   *        "now" reference)
   * @param input IMU input (ax, ay, az, wx, wy, wz)
   * @param dt Time delta passed to EKFWrapper::predict()
   */
  void predictAndRecord(const rclcpp::Time & stamp, const ekf::Input & input, double dt)
  {
    EkfTimelineEntry entry;
    entry.stamp = stamp;
    entry.state_before = wrapper_.get_state();
    entry.covariance_before = wrapper_.get_state_covariance();
    entry.type = EkfOperationType::PREDICT;
    entry.imu_input = input;
    entry.dt = dt;

    wrapper_.predict(input, dt);

    buffer_.push_back(entry);
    trim(stamp);
  }

  /**
   * @brief Record a pose/odom update, handling out-of-order arrival.
   *
   * If the measurement is older than `max_update_latency_ms`, it is dropped and
   * the EKF state/covariance are left untouched. Otherwise, the EKF is rewound
   * to the state immediately preceding `stamp`, the correction is applied there
   * (re-unwrapping the measured angles against the rewind-point state), and every
   * later operation in the buffer is replayed to bring the state back to "now".
   *
   * @param stamp Timestamp of the measurement (msg.header.stamp)
   * @param type UPDATE_POSE or UPDATE_POSE_ODOM
   * @param raw_measurement Raw pose measurement (already in map frame, roll/pitch/yaw
   *        in [-pi, pi], not unwrapped)
   * @param measurement_cov Pose measurement covariance
   * @param now Current time, used for the staleness check (now - stamp >
   *        max_update_latency) and as the trim horizon reference
   * @return UpdateResult with applied=false if the measurement was dropped as stale
   *        (state/covariance untouched in that case)
   */
  UpdateResult updateAndRecord(
    const rclcpp::Time & stamp, EkfOperationType type,
    const ekf::PoseMeasurement & raw_measurement,
    const ekf::PoseMeasurementCovariance & measurement_cov,
    const rclcpp::Time & now)
  {
    if ((now - stamp) > max_update_latency_) {
      return {false};
    }

    // idx = first entry with entry.stamp > stamp
    auto it = std::upper_bound(
      buffer_.begin(), buffer_.end(), stamp,
      [](const rclcpp::Time & s, const EkfTimelineEntry & e) {return s < e.stamp;});
    std::size_t idx = static_cast<std::size_t>(std::distance(buffer_.begin(), it));

    // Rewind snapshot: state/covariance immediately before `stamp`.
    ekf::State rewind_state;
    ekf::Covariance rewind_cov;
    if (idx < buffer_.size()) {
      rewind_state = buffer_[idx].state_before;
      rewind_cov = buffer_[idx].covariance_before;
    } else {
      // No buffered operation is newer than `stamp` — this is the common,
      // non-delayed case. Rewind point is simply the current state.
      rewind_state = wrapper_.get_state();
      rewind_cov = wrapper_.get_state_covariance();
    }

    const ekf::State state_now_before = wrapper_.get_state();

    ekf::PoseMeasurement unwrapped = unwrapPoseMeasurement(raw_measurement, rewind_state);
    wrapper_.reset(rewind_state, rewind_cov);
    // Always use the "raw" correction primitive here (no map_to_odom jump):
    // the net map_to_odom jump for this call is computed once, below, from the
    // overall "now" state transition — not from the rewind-point transition.
    wrapper_.update_pose_odom(unwrapped, measurement_cov);

    EkfTimelineEntry new_entry;
    new_entry.stamp = stamp;
    new_entry.state_before = rewind_state;
    new_entry.covariance_before = rewind_cov;
    new_entry.type = type;
    new_entry.raw_pose_measurement = raw_measurement;
    new_entry.pose_measurement_covariance = measurement_cov;
    buffer_.insert(buffer_.begin() + idx, new_entry);

    // Replay every operation that happened after the inserted entry, re-unwrapping
    // update measurements against their (possibly updated) state_before.
    for (std::size_t i = idx + 1; i < buffer_.size(); ++i) {
      buffer_[i].state_before = wrapper_.get_state();
      buffer_[i].covariance_before = wrapper_.get_state_covariance();

      if (buffer_[i].type == EkfOperationType::PREDICT) {
        wrapper_.predict(buffer_[i].imu_input, buffer_[i].dt);
      } else {
        ekf::PoseMeasurement unwrapped_i = unwrapPoseMeasurement(
          buffer_[i].raw_pose_measurement, buffer_[i].state_before);
        wrapper_.update_pose_odom(unwrapped_i, buffer_[i].pose_measurement_covariance);
      }
    }

    // map_to_odom jump only for the non-odom UPDATE_POSE type, matching the
    // existing non-buffered semantics where update_pose_odom never jumps map_to_odom.
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
   * @brief Number of entries currently in the buffer (for testing)
   */
  std::size_t size() const
  {
    return buffer_.size();
  }

  /**
   * @brief Read-only access to entry i (for testing)
   */
  const EkfTimelineEntry & at(std::size_t i) const
  {
    return buffer_.at(i);
  }

private:
  ekf::EKFWrapper & wrapper_;
  rclcpp::Duration max_update_latency_;
  std::deque<EkfTimelineEntry> buffer_;

  /**
   * @brief Trim the front of the buffer.
   *
   * Pops entries from the front while buffer_.size() > 1 and buffer_[1].stamp is
   * older than (now - max_update_latency_), always leaving at least one "floor"
   * entry at-or-before the latency horizon as a valid rewind reference.
   */
  void trim(const rclcpp::Time & now)
  {
    const rclcpp::Time horizon = now - max_update_latency_;
    while (buffer_.size() > 1 && buffer_[1].stamp < horizon) {
      buffer_.pop_front();
    }
  }
};

}  // namespace simple_ekf

#endif  // SIMPLE_EKF__EKF_HISTORY_BUFFER_HPP_
