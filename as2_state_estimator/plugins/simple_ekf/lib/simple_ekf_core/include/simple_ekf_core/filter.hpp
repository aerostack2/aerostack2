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
* @file filter.hpp
*
* The simple_ekf filter: an EKF, the frame tree its corrections move, and the policy
* deciding which measurements reach it
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__FILTER_HPP_
#define SIMPLE_EKF_CORE__FILTER_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>

#include "simple_ekf_core/ekf_history_buffer.hpp"
#include "simple_ekf_core/logging.hpp"
#include "simple_ekf_core/rigid.hpp"
#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

/**
 * @class Filter
 * @brief The state estimator's filter, with no middleware underneath it
 *
 * Everything that happens to a measurement between arriving and moving the frame tree
 * lives here: the rotation into the map frame, the components a source does not measure,
 * the innovation gate, the rewind and replay of late measurements, and the smoothing of
 * what comes out.
 *
 * Typical use: register each source once with addSource, then feed IMU readings to onImu,
 * measurements to onPose and onTwist, and call onTick at a fixed rate. Read the result
 * with outputs().
 *
 * The filter reads no clock. Every time it needs comes in as an argument, so the same
 * sequence of calls always produces the same outputs, whether they come from live data or
 * from a recording. Not thread-safe, and neither copyable nor movable.
 */
class Filter
{
public:
  /**
   * @param config Validated here: config() returns the values actually used
   * @param log Where log lines go. An empty sink discards them
   */
  explicit Filter(const Config & config, LogSink log = nullptr);

  /**
   * @brief Register a measurement source. The id is what onPose and onTwist take.
   */
  SourceId addSource(const SourceConfig & config);

  const Config & config() const {return config_;}
  const Outputs & outputs() const {return outputs_;}
  const ekf::State & state() const {return ekf_wrapper_.get_state();}
  const ekf::Covariance & stateCovariance() const {return ekf_wrapper_.get_state_covariance();}

  /**
   * @brief Whether estimation has started, see markEarthToMapSet.
   */
  bool isEarthToMapSet() const {return earth_to_map_set_;}

  /**
   * @brief Declare earth->map known, so that onTick starts doing its work.
   */
  void markEarthToMapSet() {earth_to_map_set_ = true;}

  void setEarthToMap(const Rigid & earth_to_map) {outputs_.earth_to_map = earth_to_map;}

  /**
   * @brief Set earth->map from the first pose a source reports.
   *
   * A pose in the earth frame is used as it is. A pose in the map frame is the vehicle's,
   * so earth->map is its inverse, and the state starts from it so that the first correction
   * is not a jump. Any other frame sets nothing.
   *
   * @return true if earth->map was set
   */
  bool setEarthToMapFromFirstPose(const Rigid & pose, SourceFrame frame);

  /**
   * @brief Whether the drone is flying, for the pre-flight correction onTick applies.
   *
   * Once true, the pre-flight correction never runs again, even after landing.
   */
  void setOffboard(bool offboard);

  /**
   * @brief Whether a measurement of `source` comes too soon after the last one fused to
   *        respect its `update_rate_hz`. If not, it counts as the last one fused.
   */
  bool shouldThrottleUpdate(SourceId source, Nanoseconds stamp);

  /**
   * @brief Whether a measurement of `source` repeats the position of its last one, when
   *        `reject_repeated_positions` is set. If not, its position becomes the last one.
   *
   * @param now Time the measurement was received, for rate-limiting the warning
   */
  bool isRepeatedPosition(SourceId source, const Vector3 & position, Nanoseconds now);

  void onImu(const ImuSample & imu);

  /**
   * @brief Correct with a pose measurement, in whichever frame it is expressed.
   *
   * @param now Time the measurement was received, which decides whether it is too old to
   *        be replayed
   */
  void onPose(SourceId source, const PoseSample & pose, Nanoseconds now);

  /**
   * @brief Correct the velocity states with a velocity measurement.
   *
   * The measurement is rotated into the map frame with the filter's own attitude.
   *
   * @param now Time the measurement was received
   */
  void onTwist(SourceId source, const TwistSample & twist, Nanoseconds now);

  /**
   * @brief Advance what moves with time rather than with data.
   *
   * Steps the output smoothing and, before the drone's first offboard activation, corrects
   * the state to the map origin, where the drone is known to be standing. Both scale with
   * how often this is called: the smoothing's time constant is in ticks, and each tick is
   * one more correction.
   *
   * @return true if the pre-flight correction was applied
   */
  bool onTick(Nanoseconds now);

private:
  /**
   * @brief A registered source, and what the filter remembers about it
   */
  struct SourceState
  {
    explicit SourceState(const SourceConfig & source_config)
    : config(source_config) {}

    SourceConfig config;
    std::optional<Nanoseconds> last_fused_stamp;
    std::optional<Vector3> last_position;
    std::optional<Nanoseconds> first_rejection_stamp;

    Throttle repeated_position_warning{fromSeconds(1.0)};
    Throttle zero_variance_warning{fromSeconds(5.0)};
    Throttle gate_rejection_warning{fromSeconds(1.0)};
    Throttle stale_measurement_warning{fromSeconds(1.0)};
  };

  // Declared in the order they are initialised: the logger reports what validating the
  // configuration changed, and the history buffer is built from the validated values.
  Logger logger_;
  Config config_;

  ekf::EKFWrapper ekf_wrapper_;
  EkfHistoryBuffer ekf_history_buffer_;

  std::vector<SourceState> sources_;
  SourceState preflight_source_;

  Outputs outputs_;
  bool earth_to_map_set_ = false;

  // The published map->odom is an exponential moving average of the raw one, which spreads
  // each EKF correction over several ticks instead of handing the controller a step. The
  // raw one stays in outputs_.map_to_odom, since measurements are moved into the map frame
  // with it.
  bool output_blend_initialized_ = false;
  Vector3 published_map_to_odom_velocity_{0, 0, 0};

  // Its angular velocity is what the published twist reports
  ImuSample last_imu_;

  bool drone_offboard_ = false;
  bool drone_has_been_offboard_ = false;

  static Config validated(const Config & config, const Logger & logger);
  void setupWrapper();
  void resetStateToPose(const Rigid & pose_in_map);
  void updateOutputs();
  void stepOutputBlend();

  void processPose(SourceState & source, const PoseSample & pose, Nanoseconds now);
  void processTwist(SourceState & source, const TwistSample & twist, Nanoseconds now);
  void warnIfZeroVariance(
    SourceState & source, const std::array<double, 36> & covariance, Nanoseconds now);

  /**
   * @brief The innovation gate, plus the per-source bookkeeping that keeps it from wedging
   *        the filter.
   *
   * Once a source has been rejected without interruption for `innovation_gate_timeout`,
   * its next measurement is accepted whatever its innovation: a source that disagrees for
   * that long is more likely to be right than the state it disagrees with.
   *
   * @return true if the measurement should be fused
   */
  template<std::size_t N>
  bool acceptsInnovation(
    SourceState & source,
    const std::array<double, N> & innovations,
    const std::array<double, N> & state_variances,
    const std::array<double, N> & measurement_variances,
    Nanoseconds stamp,
    Nanoseconds now)
  {
    const SourceConfig & config = source.config;
    if (isWithinInnovationGate(
        innovations, state_variances, measurement_variances, config.innovation_gate))
    {
      source.first_rejection_stamp.reset();
      return true;
    }

    if (!source.first_rejection_stamp) {
      source.first_rejection_stamp = stamp;
    }

    if (toSeconds(stamp - *source.first_rejection_stamp) >= config.innovation_gate_timeout) {
      logger_.log(
        LogLevel::WARN,
        "Source '%s' has disagreed with the filter for %.1f s. Accepting it and letting it "
        "correct the state", config.name.c_str(), config.innovation_gate_timeout);
      source.first_rejection_stamp.reset();
      return true;
    }

    if (source.gate_rejection_warning.allow(now)) {
      logger_.log(
        LogLevel::WARN,
        "Dropping a measurement from source '%s': further than %.1f standard deviations "
        "from the filter's prediction", config.name.c_str(), config.innovation_gate);
    }
    return false;
  }
};

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__FILTER_HPP_
