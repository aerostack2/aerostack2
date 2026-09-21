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
* @file types.hpp
*
* Plain types the simple_ekf filter is driven with: timestamps, the frame a
* measurement is expressed in, the measurements themselves and the filter's output
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__TYPES_HPP_
#define SIMPLE_EKF_CORE__TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "simple_ekf_core/rigid.hpp"

namespace simple_ekf_core
{

/**
 * @brief A point in time or a duration, in nanoseconds.
 *
 * Integer nanoseconds and not seconds in a double: a stamp counted from the epoch is
 * about 1.7e18 ns and a double holds 53 bits, so seconds would lose resolution the
 * moment a real clock is used.
 */
using Nanoseconds = std::int64_t;

/// Divided rather than multiplied by 1e-9, which is not exact: this is what
/// rclcpp::Duration::seconds() computes, to the last bit.
inline double toSeconds(Nanoseconds duration)
{
  return static_cast<double>(duration) / 1e9;
}

inline Nanoseconds fromSeconds(double duration)
{
  return static_cast<Nanoseconds>(duration * 1e9);
}

/**
 * @brief The frame a measurement is expressed in: one of the four of the tree the filter
 *        maintains.
 */
enum class SourceFrame
{
  EARTH,
  MAP,
  ODOM,
  BASE
};

/**
 * @brief Identifies a measurement source registered with Filter::addSource.
 */
using SourceId = std::size_t;

/**
 * @brief How the filter treats the measurements of one source
 */
struct SourceConfig
{
  /// Name used in log lines
  std::string name;

  /// Whether a correction is absorbed by odom->base (true) or moves map->odom (false)
  bool is_odometry = false;

  /// Maximum rate at which measurements are fused, in Hz. 0 fuses every one
  double update_rate_hz = 0.0;

  /// Drop a measurement whose position repeats the last one
  bool reject_repeated_positions = false;

  /// Positions closer than this, in metres, count as the same one
  double repeated_position_threshold = 1e-6;

  /// Innovation gate width in standard deviations. 0 disables it
  double innovation_gate = 0.0;

  /// Seconds of uninterrupted rejection after which the next measurement is fused anyway
  double innovation_gate_timeout = 1.0;

  /// Scale the variances a measurement carries (true) or replace them (false) with the
  /// values below. See getCovarianceWithConfig
  bool use_message_covariance = false;

  /// Position and orientation variances or multipliers (x, y, z and roll, pitch, yaw)
  std::array<double, 3> position_values{};
  std::array<double, 3> orientation_values{};

  /// Linear velocity variances or multipliers, for a velocity source
  std::array<double, 3> linear_values{};
};

/**
 * @brief One IMU reading, in the vehicle's frame: the filter's prediction input
 */
struct ImuSample
{
  Nanoseconds stamp = 0;
  std::array<double, 3> linear_acceleration{};
  std::array<double, 3> angular_velocity{};
};

/**
 * @brief One pose measurement
 *
 * The covariance keeps the 6x6 row-major layout of the sources this filter is fed from:
 * the rotation into the map frame reads the off-diagonal terms of both 3x3 blocks, so the
 * diagonal alone would not do. A non-positive variance marks a component the source does
 * not measure.
 */
struct PoseSample
{
  Nanoseconds stamp = 0;
  SourceFrame frame = SourceFrame::MAP;
  Rigid pose = Rigid::getIdentity();
  std::array<double, 36> covariance{};
};

/**
 * @brief One linear velocity measurement, with the same covariance layout as PoseSample
 */
struct TwistSample
{
  Nanoseconds stamp = 0;
  SourceFrame frame = SourceFrame::BASE;
  std::array<double, 3> linear{};
  std::array<double, 36> covariance{};
};

/**
 * @brief The vehicle's velocity, expressed in its own frame
 */
struct TwistInBase
{
  Vector3 linear{0, 0, 0};
  Vector3 angular{0, 0, 0};
};

/**
 * @brief Everything the filter produces, recomputed after every prediction and correction
 *
 * The tree to publish is `earth_to_map`, `published_map_to_odom` and `odom_to_base`, with
 * `twist_in_base`. The raw `map_to_odom` and `internal_twist_in_base` are the same before
 * the output smoothing, for debugging.
 */
struct Outputs
{
  Rigid earth_to_map = Rigid::getIdentity();
  Rigid map_to_odom = Rigid::getIdentity();
  Rigid published_map_to_odom = Rigid::getIdentity();
  Rigid odom_to_base = Rigid::getIdentity();
  TwistInBase twist_in_base;
  TwistInBase internal_twist_in_base;
};

/**
 * @brief Everything the filter is set up with
 *
 * The defaults are the values of the plugin's config/plugin_default.yaml.
 */
struct Config
{
  /// Initial variance of each group of states
  double initial_position_covariance = 0.0;
  double initial_velocity_covariance = 0.0;
  double initial_orientation_covariance = 0.0;
  double initial_bias_acc_covariance = 1e-8;
  double initial_bias_gyro_covariance = 1e-8;

  /// Gravity along z. The model subtracts it from the rotated specific force, so it is
  /// positive: an IMU at rest reads +9.81 on z
  double gravity = 9.81;

  /// IMU noise, as the datasheet states it
  double accelerometer_noise_density = 1e-3;
  double gyroscope_noise_density = 1e-4;
  double accelerometer_random_walk = 1e-4;
  double gyroscope_random_walk = 1e-5;

  /// Age beyond which a measurement is dropped instead of replayed, in milliseconds
  double max_update_latency_ms = 1000.0;

  /// Variance standing in for a component no source measures. A conditioning constant,
  /// not a tuning knob: a textbook 1e9 diverges here
  double unobserved_variance = 1.0e2;

  /// Weight of the newest raw map->odom in the published one, per tick. 1 disables smoothing
  double map_odom_alpha = 0.1;

  /// Pre-flight correction: until the drone first goes offboard, every tick corrects the
  /// state towards this pose, in the map frame, with this variance on every component. By
  /// default it asserts that the drone sits at the map origin
  Rigid preflight_pose = Rigid::getIdentity();
  double preflight_variance = 1e-5;

  bool verbose = false;
  bool debug_verbose = false;
};

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__TYPES_HPP_
