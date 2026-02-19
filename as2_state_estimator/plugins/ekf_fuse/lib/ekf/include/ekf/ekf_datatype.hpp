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
* An EKF Wrapper implementation
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef EKF__EKF_DATATYPE_H
#define EKF__EKF_DATATYPE_H

#include <array>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace ekf
{

/**
 * @brief State X
 */
struct State
{
  static const std::size_t size = 15;
  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;
  static const int VX = 3;
  static const int VY = 4;
  static const int VZ = 5;
  static const int ROLL = 6;
  static const int PITCH = 7;
  static const int YAW = 8;
  static const int ABX = 9;
  static const int ABY = 10;
  static const int ABZ = 11;
  static const int WBX = 12;
  static const int WBY = 13;
  static const int WBZ = 14;

  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  State();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the state
   */
  State(const std::array<double, size> & values);

  /**
   * @brief Sets the state to the provided values
   * @param values Values to set the state
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief Get position (x, y, z)
   * @return A 3D vector representing the position
   */
  std::array<double, 3> get_position() const;

  /**
   * @brief Get velocity (vx, vy, vz)
   * @return A 3D vector representing the Velocity
   */
  std::array<double, 3> get_velocity() const;

  /**
   * @brief Get orientation (roll, pitch, yaw)
   * @return A 3D vector representing the orientation in radians
   */
  std::array<double, 3> get_orientation() const;

  /**
   * @brief Get orientation as a quaternion (qx, qy, qz, qw)
   * @return A 4D vector representing the orientation as a quaternion
   */
  std::array<double, 4> get_orientation_quaternion() const;

  /**
   * @brief Get accelerometer bias (abx, aby, abz)
   * @return A 3D vector representing the accelerometer bias
   */
  std::array<double, 3> get_accelerometer_bias() const;

  /**
   * @brief Get gyroscope bias (wbx, wby, wbz)
   * @return A 3D vector representing the gyroscope bias
   */
  std::array<double, 3> get_gyroscope_bias() const;

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the state
   */
  std::string to_string() const;

};


/**
 * @brief State covariance P
 */
struct Covariance
{
  static const std::size_t size = 225; // 15x15 covariance matrix
  static const int rows = 15;
  static const int cols = 15;
  static const int X = 0;
  static const int Y = 16;
  static const int Z = 32;
  static const int VX = 48;
  static const int VY = 64;
  static const int VZ = 80;
  static const int ROLL = 96;
  static const int PITCH = 112;
  static const int YAW = 128;
  static const int ABX = 144;
  static const int ABY = 160;
  static const int ABZ = 176;
  static const int WBX = 192;
  static const int WBY = 208;
  static const int WBZ = 224;

  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  Covariance();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the covariance
   */
  Covariance(const std::array<double, size> & values);

  /**
   * @brief Sets the covariance to the provided values
   * @param values Values to set the covariance
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the covariance
   */
  std::string to_string() const;

  /**
   * @brief The print operator for easy debugging of the main diagonal
   * @return A string representation of the main diagonal of the covariance
   */
  std::string to_string_diagonal() const;
};


/**
 * @brief gravity vector
 */
struct Gravity
{
  static const std::size_t size = 3;
  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  Gravity();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the gravity vector
   */
  Gravity(const std::array<double, size> & values);

  /**
   * @brief Sets the gravity vector to the provided values
   * @param values Values to set the gravity vector
   */
  void set(const std::array<double, size> & values);
};


/**
 * @brief IMU input measurements
 */
struct Input
{
  static const std::size_t size = 6; // 3 accelerometer + 3 gyroscope
  static const int AX = 0;
  static const int AY = 1;
  static const int AZ = 2;
  static const int WX = 3;
  static const int WY = 4;
  static const int WZ = 5;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  Input();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the input measurements
   */
  Input(const std::array<double, size> & values);

  /**
   * @brief Sets the input measurements to the provided values
   * @param values Values to set the input measurements
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the input measurements
   */
  std::string to_string() const;
};


/**
 * @brief Pose measurement Z_pose
 */
struct PoseMeasurement
{
  static const std::size_t size = 6; // 3 position + 3 orientation
  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;
  static const int ROLL = 3;
  static const int PITCH = 4;
  static const int YAW = 5;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  PoseMeasurement();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the pose measurement
   */
  PoseMeasurement(const std::array<double, size> & values);

  /**
   * @brief Sets the pose measurement to the provided values
   * @param values Values to set the pose measurement
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the pose measurement
   */
  std::string to_string() const;
};


/**
 * @brief Pose measurement covariance diagonal R_pose
 */
struct PoseMeasurementCovariance
{
  static const std::size_t size = 6; // 3 position + 3 orientation
  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;
  static const int ROLL = 3;
  static const int PITCH = 4;
  static const int YAW = 5;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  PoseMeasurementCovariance();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the pose measurement covariance
   */
  PoseMeasurementCovariance(const std::array<double, size> & values);

  /**
   * @brief Sets the pose measurement covariance to the provided values
   * @param values Values to set the pose measurement covariance
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the pose measurement covariance
   */
  std::string to_string() const;
};


/**
 * @brief Velocity measurement Z_velocity
 */
struct VelocityMeasurement
{
  static const std::size_t size = 3; // 3 velocity
  static const int VX = 0;
  static const int VY = 1;
  static const int VZ = 2;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  VelocityMeasurement();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the velocity measurement
   */
  VelocityMeasurement(const std::array<double, size> & values);

  /**
   * @brief Sets the velocity measurement to the provided values
   * @param values Values to set the velocity measurement
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the velocity measurement
   */
  std::string to_string() const;
};


/**
 * @brief Pose and Velocity measurement covariance diagonal R_pose_velocity
 */
struct VelocityMeasurementCovariance
{
  static const std::size_t size = 3; // 3 velocity
  static const int VX = 0;
  static const int VY = 1;
  static const int VZ = 2;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  VelocityMeasurementCovariance();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the velocity measurement covariance
   */
  VelocityMeasurementCovariance(const std::array<double, size> & values);

  /**
   * @brief Sets the velocity measurement covariance to the provided values
   * @param values Values to set the velocity measurement covariance
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the velocity measurement covariance
   */
  std::string to_string() const;
};


/**
 * @brief Odometry
 */
struct Odometry
{
  static const std::size_t size = 12; // 3 position + 3 orientation + 3 linear velocity + 3 angular velocity
  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;
  static const int ROLL = 3;
  static const int PITCH = 4;
  static const int YAW = 5;
  static const int VX = 6;
  static const int VY = 7;
  static const int VZ = 8;
  static const int WX = 9;
  static const int WY = 10;
  static const int WZ = 11;
  std::array<double, size> data;

  /**
   * @brief Constructor
   */
  Odometry();

  /**
   * @brief Constructor with initial values
   * @param values Initial values for the odometry
   */
  Odometry(const std::array<double, size> & values);

  /**
   * @brief Sets the odometry to the provided values
   * @param values Values to set the odometry
   */
  void set(const std::array<double, size> & values);

  /**
   * @brief The print operator for easy debugging
   * @return A string representation of the odometry
   */
  std::string to_string() const;
};

} // namespace ekf

#endif // EKF__EKF_DATATYPE_H
