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
* An EKF Wrapper implementation
*
* @authors Rodrigo Da Silva Gómez
*/

#include "ekf/ekf_datatype.hpp"

namespace ekf
{


State::State()
{
  data.fill(0.0);
}


State::State(const std::array<double, size> & values)
{
  set(values);
}


std::array<double, 3> State::get_position() const
{
  return {data[0], data[1], data[2]};
}


std::array<double, 3> State::get_velocity() const
{
  return {data[3], data[4], data[5]};
}


std::array<double, 3> State::get_orientation() const
{
  return {data[6], data[7], data[8]};
}


std::array<double, 4> State::get_orientation_quaternion() const
{
  double roll = data[6];
  double pitch = data[7];
  double yaw = data[8];

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  return {x, y, z, w};
}


std::array<double, 3> State::get_accelerometer_bias() const
{
  return {data[9], data[10], data[11]};
}


std::array<double, 3> State::get_gyroscope_bias() const
{
  return {data[12], data[13], data[14]};
}


void State::set(const std::array<double, size> & values)
{
  data = values;
}


std::string State::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Covariance::Covariance()
{
  data.fill(0.0);
}


Covariance::Covariance(const std::array<double, size> & values)
{
  set(values);
}


void Covariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string Covariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 15 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


std::string Covariance::to_string_diagonal() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < 15; ++i) {
    oss << data[i * 15 + i];

    if (i + 1 != 15) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Gravity::Gravity()
{
  data.fill(0.0);
  data[2] = 9.81; // Default gravity value in m/s^2
}


Gravity::Gravity(const std::array<double, size> & values)
{
  set(values);
}


void Gravity::set(const std::array<double, size> & values)
{
  data = values;
}


Input::Input()
{
  data.fill(0.0);
}


Input::Input(const std::array<double, size> & values)
{
  set(values);
}


void Input::set(const std::array<double, size> & values)
{
  data = values;
}


std::string Input::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


PoseMeasurement::PoseMeasurement()
{
  data.fill(0.0);
}


PoseMeasurement::PoseMeasurement(const std::array<double, size> & values)
{
  set(values);
}


void PoseMeasurement::set(const std::array<double, size> & values)
{
  data = values;
}


std::string PoseMeasurement::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


PoseMeasurementCovariance::PoseMeasurementCovariance()
{
  data.fill(0.0);
}


PoseMeasurementCovariance::PoseMeasurementCovariance(const std::array<double, size> & values)
{
  set(values);
}


void PoseMeasurementCovariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string PoseMeasurementCovariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


VelocityMeasurement::VelocityMeasurement()
{
  data.fill(0.0);
}


VelocityMeasurement::VelocityMeasurement(const std::array<double, size> & values)
{
  set(values);
}


void VelocityMeasurement::set(const std::array<double, size> & values)
{
  data = values;
}


std::string VelocityMeasurement::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


VelocityMeasurementCovariance::VelocityMeasurementCovariance()
{
  data.fill(0.0);
}


VelocityMeasurementCovariance::VelocityMeasurementCovariance(
  const std::array<double,
  size> & values)
{
  set(values);
}


void VelocityMeasurementCovariance::set(const std::array<double, size> & values)
{
  data = values;
}


std::string VelocityMeasurementCovariance::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 3 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


Odometry::Odometry()
{
  data.fill(0.0);
}

Odometry::Odometry(const std::array<double, size> & values)
{
  set(values);
}

void Odometry::set(const std::array<double, size> & values)
{
  data = values;
}

std::string Odometry::to_string() const
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < data.size(); ++i) {
    oss << data[i];

    if (i + 1 != data.size()) {
      oss << ", ";
      if ((i + 1) % 6 == 0) {
        oss << "\n ";
      }
    }
  }
  oss << "]";
  return oss.str();
}


} // namespace ekf
