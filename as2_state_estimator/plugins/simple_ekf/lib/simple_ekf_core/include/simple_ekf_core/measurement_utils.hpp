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
* @file measurement_utils.hpp
*
* What a measurement goes through between arriving and being believed: its covariance,
* the components nobody measured, the angles' branch and the innovation gate
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_
#define SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_

#include <array>
#include <cmath>
#include <cstddef>

#include "ekf/ekf_datatype.hpp"

#include "simple_ekf_core/rigid.hpp"
#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

/// The state each component of a pose measurement measures, in measurement order
inline constexpr std::array<int, 6> kPoseStateIndices = {
  ekf::State::X, ekf::State::Y, ekf::State::Z,
  ekf::State::ROLL, ekf::State::PITCH, ekf::State::YAW};

/// The state each component of a velocity measurement measures, in measurement order
inline constexpr std::array<int, 3> kVelocityStateIndices = {
  ekf::State::VX, ekf::State::VY, ekf::State::VZ};

/// Flat index in ekf::Covariance of the variance of a state
constexpr int varianceIndex(int state_index)
{
  return state_index * (ekf::Covariance::cols + 1);
}

/// Flat index in a 6x6 row-major covariance of the variance of component `index`
constexpr std::size_t diagonalIndex(std::size_t index)
{
  return index * 6 + index;
}

/**
 * @brief The 6x6 covariance a source configured with fixed variances is believed with
 *
 * Diagonal, from `position_values` and `orientation_values`. All zeros when the source is
 * configured to use the covariance its measurements carry, since then there is nothing
 * fixed to give it.
 */
inline std::array<double, 36> generateCovarianceFromConfig(const SourceConfig & config)
{
  std::array<double, 36> covariance{};
  if (config.use_message_covariance) {
    return covariance;
  }

  for (std::size_t index = 0; index < 3; ++index) {
    covariance[diagonalIndex(index)] = config.position_values[index];
    covariance[diagonalIndex(index + 3)] = config.orientation_values[index];
  }
  return covariance;
}

/**
 * @brief The covariance a pose measurement is believed with, from what it carries
 *
 * With `use_message_covariance` the carried variances are scaled by the configured values;
 * without it they are replaced by them.
 *
 * A negative variance is the source saying it does not measure that component, so the
 * configured values never overwrite one: that would turn a component nobody measured into
 * a reading of whatever the message left in that field. A zero is not the same statement:
 * it is what a message that carries no covariance at all is full of, and supplying the
 * number it lacks is the whole point of `use_message_covariance: false`.
 */
inline std::array<double, 36> getCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const SourceConfig & config)
{
  if (!config.use_message_covariance) {
    std::array<double, 36> covariance = generateCovarianceFromConfig(config);
    for (std::size_t index = 0; index < 6; ++index) {
      if (input_covariance[diagonalIndex(index)] < 0.0) {
        covariance[diagonalIndex(index)] = input_covariance[diagonalIndex(index)];
      }
    }
    return covariance;
  }

  std::array<double, 36> covariance = input_covariance;
  for (std::size_t index = 0; index < 3; ++index) {
    covariance[diagonalIndex(index)] *= config.position_values[index];
    covariance[diagonalIndex(index + 3)] *= config.orientation_values[index];
  }
  return covariance;
}

/**
 * @brief The velocity counterpart of @ref getCovarianceWithConfig, applied to the linear
 *        block only: no correction reads the angular one.
 */
inline std::array<double, 36> getLinearCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const SourceConfig & config)
{
  std::array<double, 36> covariance = input_covariance;
  for (std::size_t index = 0; index < 3; ++index) {
    double & variance = covariance[diagonalIndex(index)];
    if (config.use_message_covariance) {
      variance *= config.linear_values[index];
    } else if (input_covariance[diagonalIndex(index)] >= 0.0) {
      variance = config.linear_values[index];
    }
  }
  return covariance;
}

/**
 * @brief A pose as the EKF measures it: position and roll, pitch, yaw
 *
 * The angles are each in [-pi, pi], not unwrapped against any state: see
 * @ref unwrapPoseMeasurement.
 */
inline ekf::PoseMeasurement poseToRawEkfMeasurement(const Rigid & pose)
{
  const std::array<double, 3> angles = toRollPitchYaw(pose.getRotation());
  return ekf::PoseMeasurement(
    {
      pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
      angles[0], angles[1], angles[2]});
}

/**
 * @brief Move each measured angle to the branch closest to the state's
 *
 * The EKF state tracks orientation continuously. Without this, a measurement crossing +-pi
 * would look like a jump of 2 pi to it.
 */
inline ekf::PoseMeasurement unwrapPoseMeasurement(
  const ekf::PoseMeasurement & raw,
  const ekf::State & reference_state)
{
  ekf::PoseMeasurement unwrapped = raw;
  for (const int angle : {ekf::PoseMeasurement::ROLL, ekf::PoseMeasurement::PITCH,
      ekf::PoseMeasurement::YAW})
  {
    const double state_angle = reference_state.data[kPoseStateIndices[angle]];
    double difference = raw.data[angle] - state_angle;
    difference -= 2.0 * M_PI * std::round(difference / (2.0 * M_PI));
    unwrapped.data[angle] = state_angle + difference;
  }
  return unwrapped;
}

/**
 * @brief The diagonal of a 6x6 pose covariance, which is all the EKF takes
 */
inline ekf::PoseMeasurementCovariance covarianceToEkfMeasurementCovariance(
  const std::array<double, 36> & covariance)
{
  ekf::PoseMeasurementCovariance measurement_cov;
  for (std::size_t index = 0; index < 6; ++index) {
    measurement_cov.data[index] = covariance[diagonalIndex(index)];
  }
  return measurement_cov;
}

/**
 * @brief Which components of a measurement carry no information: those with a
 *        non-positive variance
 */
inline std::array<bool, 6> unobservedComponents(const std::array<double, 36> & covariance)
{
  std::array<bool, 6> unobserved{};
  for (std::size_t index = 0; index < 6; ++index) {
    unobserved[index] = covariance[diagonalIndex(index)] <= 0.0;
  }
  return unobserved;
}

/**
 * @brief Which components carry a variance of exactly zero
 *
 * The only non-positive variance that is usually a mistake rather than a statement: a
 * source filling no covariance in leaves zeros, and reading them as unobserved drops the
 * measurement.
 */
inline std::array<bool, 6> zeroVarianceComponents(const std::array<double, 36> & covariance)
{
  std::array<bool, 6> zeroed{};
  for (std::size_t index = 0; index < 6; ++index) {
    zeroed[index] = covariance[diagonalIndex(index)] == 0.0;
  }
  return zeroed;
}

/**
 * @brief Replace every non-positive variance with a usable one
 *
 * Applied before anything else touches the measurement, so that the rest of the pipeline,
 * the rotation into the map frame included, only ever sees usable numbers.
 */
inline void resolveUnobservedVariances(
  std::array<double, 36> & covariance, double unobserved_variance)
{
  for (std::size_t index = 0; index < 6; ++index) {
    double & variance = covariance[diagonalIndex(index)];
    if (variance <= 0.0) {
      variance = unobserved_variance;
    }
  }
}

/**
 * @brief Give the unobserved components of a pose measurement nothing to say
 *
 * Each flagged component is overwritten with the state's own value and given the
 * unobserved variance, so its innovation is zero and so is the correction it produces,
 * whatever the gain turns out to be.
 *
 * The flags are read in the source's frame and applied after the rotation into the map one,
 * which is exact for the sets that survive a yaw-only rotation: all three, none, or the
 * horizontal pair.
 */
inline void neutraliseUnobservedComponents(
  ekf::PoseMeasurement & measurement,
  ekf::PoseMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state,
  double unobserved_variance)
{
  for (std::size_t index = 0; index < kPoseStateIndices.size(); ++index) {
    if (unobserved[index]) {
      measurement.data[index] = state.data[kPoseStateIndices[index]];
      covariance.data[index] = unobserved_variance;
    }
  }
}

/**
 * @brief The velocity counterpart of @ref neutraliseUnobservedComponents
 */
inline void neutraliseUnobservedVelocityComponents(
  ekf::VelocityMeasurement & measurement,
  ekf::VelocityMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state,
  double unobserved_variance)
{
  for (std::size_t index = 0; index < kVelocityStateIndices.size(); ++index) {
    if (unobserved[index]) {
      measurement.data[index] = state.data[kVelocityStateIndices[index]];
      covariance.data[index] = unobserved_variance;
    }
  }
}

inline ekf::VelocityMeasurement twistToEkfVelocityMeasurement(const TwistSample & twist)
{
  return ekf::VelocityMeasurement(twist.linear);
}

inline ekf::VelocityMeasurementCovariance twistToEkfVelocityCovariance(const TwistSample & twist)
{
  return ekf::VelocityMeasurementCovariance(
    {
      twist.covariance[diagonalIndex(0)],
      twist.covariance[diagonalIndex(1)],
      twist.covariance[diagonalIndex(2)]});
}

/**
 * @brief Whether a measurement is close enough to the prediction to be believed
 *
 * The measurement models select states directly, so a component's innovation is the
 * difference between measurement and prediction, and its variance is the sum of the two
 * variances. A component further than `gate` standard deviations away is not a
 * measurement of this vehicle: an obstacle under the rangefinder, a motion capture frame
 * that swapped bodies, a sensor gone wrong. A component the measurement does not observe
 * passes on its own, since the unobserved variance dwarfs any innovation it could produce.
 *
 * @param gate Number of standard deviations allowed. Non-positive disables the check
 * @return true when every component is within the gate
 */
template<std::size_t N>
bool isWithinInnovationGate(
  const std::array<double, N> & innovations,
  const std::array<double, N> & state_variances,
  const std::array<double, N> & measurement_variances,
  double gate)
{
  if (gate <= 0.0) {
    return true;
  }

  for (std::size_t index = 0; index < N; ++index) {
    const double innovation_variance = state_variances[index] + measurement_variances[index];
    if (innovation_variance > 0.0 &&
      innovations[index] * innovations[index] > gate * gate * innovation_variance)
    {
      return false;
    }
  }
  return true;
}

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__MEASUREMENT_UTILS_HPP_
