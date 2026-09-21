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
* @file ekf_datatype.cpp
*
* The vectors the EKF reads and writes
*
* @authors Rodrigo Da Silva Gómez
*/

#include "ekf/ekf_datatype.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <string>

namespace ekf
{

std::string format_values(
  const double * values, std::size_t count, std::size_t values_per_line)
{
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < count; ++i) {
    oss << values[i];
    if (i + 1 != count) {
      oss << ", ";
      if ((i + 1) % values_per_line == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}

std::array<double, 3> State::get_position() const
{
  return {data[X], data[Y], data[Z]};
}

std::array<double, 3> State::get_velocity() const
{
  return {data[VX], data[VY], data[VZ]};
}

std::array<double, 3> State::get_orientation() const
{
  return {data[ROLL], data[PITCH], data[YAW]};
}

std::array<double, 4> State::get_orientation_quaternion() const
{
  const double cy = std::cos(data[YAW] * 0.5);
  const double sy = std::sin(data[YAW] * 0.5);
  const double cr = std::cos(data[ROLL] * 0.5);
  const double sr = std::sin(data[ROLL] * 0.5);
  const double cp = std::cos(data[PITCH] * 0.5);
  const double sp = std::sin(data[PITCH] * 0.5);

  const double w = cr * cp * cy + sr * sp * sy;
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;

  return {x, y, z, w};
}

std::array<double, 3> State::get_accelerometer_bias() const
{
  return {data[ABX], data[ABY], data[ABZ]};
}

std::array<double, 3> State::get_gyroscope_bias() const
{
  return {data[WBX], data[WBY], data[WBZ]};
}

std::string State::to_string() const
{
  return format_values(data.data(), size, 3);
}

std::string Covariance::to_string() const
{
  return format_values(data.data(), size, cols);
}

std::string Covariance::to_string_diagonal() const
{
  std::array<double, rows> diagonal;
  for (int i = 0; i < rows; ++i) {
    diagonal[i] = data[i * (cols + 1)];
  }
  return format_values(diagonal.data(), diagonal.size(), 3);
}

std::string Input::to_string() const
{
  return format_values(data.data(), size, 6);
}

std::string PoseMeasurement::to_string() const
{
  return format_values(data.data(), size, 6);
}

std::string PoseMeasurementCovariance::to_string() const
{
  return format_values(data.data(), size, 6);
}

std::string VelocityMeasurement::to_string() const
{
  return format_values(data.data(), size, 3);
}

std::string VelocityMeasurementCovariance::to_string() const
{
  return format_values(data.data(), size, 3);
}

}  // namespace ekf
