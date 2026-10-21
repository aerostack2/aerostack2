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
* @file ekf_datatype.hpp
*
* The vectors the EKF reads and writes, as fixed-size arrays with named indices
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef EKF__EKF_DATATYPE_HPP_
#define EKF__EKF_DATATYPE_HPP_

#include <array>
#include <cstddef>
#include <string>

namespace ekf
{

/**
 * @brief Format values as "[a, b, c, ...]", starting a new line every `values_per_line`.
 */
std::string format_values(
  const double * values, std::size_t count, std::size_t values_per_line);

/**
 * @brief Filter state: position, velocity and orientation in the map frame, and IMU biases
 */
struct State
{
  static constexpr std::size_t size = 15;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;
  static constexpr int VX = 3;
  static constexpr int VY = 4;
  static constexpr int VZ = 5;
  static constexpr int ROLL = 6;
  static constexpr int PITCH = 7;
  static constexpr int YAW = 8;
  static constexpr int ABX = 9;
  static constexpr int ABY = 10;
  static constexpr int ABZ = 11;
  static constexpr int WBX = 12;
  static constexpr int WBY = 13;
  static constexpr int WBZ = 14;

  std::array<double, size> data{};

  State() = default;
  explicit State(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::array<double, 3> get_position() const;
  std::array<double, 3> get_velocity() const;

  /// Roll, pitch and yaw, in radians
  std::array<double, 3> get_orientation() const;

  /// The orientation as a quaternion (qx, qy, qz, qw)
  std::array<double, 4> get_orientation_quaternion() const;

  std::array<double, 3> get_accelerometer_bias() const;
  std::array<double, 3> get_gyroscope_bias() const;

  std::string to_string() const;
};

/**
 * @brief State covariance P, a 15x15 matrix stored row-major
 *
 * The named indices are the flat positions of the diagonal: `Covariance::YAW` is the
 * variance of `State::YAW`.
 */
struct Covariance
{
  static constexpr int rows = 15;
  static constexpr int cols = 15;
  static constexpr std::size_t size = rows * cols;
  static constexpr int X = State::X * (cols + 1);
  static constexpr int Y = State::Y * (cols + 1);
  static constexpr int Z = State::Z * (cols + 1);
  static constexpr int VX = State::VX * (cols + 1);
  static constexpr int VY = State::VY * (cols + 1);
  static constexpr int VZ = State::VZ * (cols + 1);
  static constexpr int ROLL = State::ROLL * (cols + 1);
  static constexpr int PITCH = State::PITCH * (cols + 1);
  static constexpr int YAW = State::YAW * (cols + 1);
  static constexpr int ABX = State::ABX * (cols + 1);
  static constexpr int ABY = State::ABY * (cols + 1);
  static constexpr int ABZ = State::ABZ * (cols + 1);
  static constexpr int WBX = State::WBX * (cols + 1);
  static constexpr int WBY = State::WBY * (cols + 1);
  static constexpr int WBZ = State::WBZ * (cols + 1);

  std::array<double, size> data{};

  Covariance() = default;
  explicit Covariance(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
  std::string to_string_diagonal() const;
};

/**
 * @brief Gravity, as the model subtracts it from the rotated specific force
 *
 * An IMU at rest reads +9.81 on z, so gravity is +9.81 on z for the two to cancel.
 */
struct Gravity
{
  static constexpr std::size_t size = 3;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;

  std::array<double, size> data{0.0, 0.0, 9.81};

  Gravity() = default;
  explicit Gravity(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}
};

/**
 * @brief IMU reading, the prediction input: specific force and angular rate in the body frame
 */
struct Input
{
  static constexpr std::size_t size = 6;
  static constexpr int AX = 0;
  static constexpr int AY = 1;
  static constexpr int AZ = 2;
  static constexpr int WX = 3;
  static constexpr int WY = 4;
  static constexpr int WZ = 5;

  std::array<double, size> data{};

  Input() = default;
  explicit Input(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
};

/**
 * @brief Pose measurement: position and roll, pitch, yaw, in the map frame
 */
struct PoseMeasurement
{
  static constexpr std::size_t size = 6;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;
  static constexpr int ROLL = 3;
  static constexpr int PITCH = 4;
  static constexpr int YAW = 5;

  std::array<double, size> data{};

  PoseMeasurement() = default;
  explicit PoseMeasurement(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
};

/**
 * @brief Variances of a pose measurement, one per component
 */
struct PoseMeasurementCovariance
{
  static constexpr std::size_t size = 6;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;
  static constexpr int ROLL = 3;
  static constexpr int PITCH = 4;
  static constexpr int YAW = 5;

  std::array<double, size> data{};

  PoseMeasurementCovariance() = default;
  explicit PoseMeasurementCovariance(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
};

/**
 * @brief Velocity measurement, in the map frame
 */
struct VelocityMeasurement
{
  static constexpr std::size_t size = 3;
  static constexpr int VX = 0;
  static constexpr int VY = 1;
  static constexpr int VZ = 2;

  std::array<double, size> data{};

  VelocityMeasurement() = default;
  explicit VelocityMeasurement(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
};

/**
 * @brief Variances of a velocity measurement, one per component
 */
struct VelocityMeasurementCovariance
{
  static constexpr std::size_t size = 3;
  static constexpr int VX = 0;
  static constexpr int VY = 1;
  static constexpr int VZ = 2;

  std::array<double, size> data{};

  VelocityMeasurementCovariance() = default;
  explicit VelocityMeasurementCovariance(const std::array<double, size> & values)
  : data(values) {}
  void set(const std::array<double, size> & values) {data = values;}

  std::string to_string() const;
};

}  // namespace ekf

#endif  // EKF__EKF_DATATYPE_HPP_
