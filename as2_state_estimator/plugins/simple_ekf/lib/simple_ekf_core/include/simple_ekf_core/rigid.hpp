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
* @file rigid.hpp
*
* The rigid transform the filter composes its frame tree from, and the handful of
* operations it needs from one
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__RIGID_HPP_
#define SIMPLE_EKF_CORE__RIGID_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>

namespace simple_ekf_core
{

/**
 * tf2's LinearMath is a header-only rigid-transform library: no rclcpp, no middleware and
 * no messages, so it costs the filter nothing at runtime. It is aliased here rather than
 * used directly because its getRPY and slerp are part of what makes the filter's output
 * what it is: replacing them is a change to this file, not a search through the library.
 */
using Rigid = tf2::Transform;
using Vector3 = tf2::Vector3;
using Quaternion = tf2::Quaternion;
using RotationMatrix = tf2::Matrix3x3;

inline Rigid eigenMatrix4dToRigid(const Eigen::Matrix4d & matrix)
{
  const Eigen::Quaterniond rotation(Eigen::Matrix3d(matrix.block<3, 3>(0, 0)));
  Quaternion quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w());
  quaternion.normalize();

  Rigid transform;
  transform.setOrigin(Vector3(matrix(0, 3), matrix(1, 3), matrix(2, 3)));
  transform.setRotation(quaternion);
  return transform;
}

inline Eigen::Matrix4d rigidToEigenMatrix4d(const Rigid & transform)
{
  const Quaternion & rotation = transform.getRotation();
  const Eigen::Quaterniond quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z());

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix(0, 3) = transform.getOrigin().x();
  matrix(1, 3) = transform.getOrigin().y();
  matrix(2, 3) = transform.getOrigin().z();
  return matrix;
}

inline Eigen::Matrix3d rotationToEigenMatrix3d(const RotationMatrix & rotation)
{
  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      matrix(row, column) = rotation[row][column];
    }
  }
  return matrix;
}

/**
 * @brief Roll, pitch and yaw of a rotation, each in [-pi, pi]
 */
inline std::array<double, 3> toRollPitchYaw(const Quaternion & rotation)
{
  std::array<double, 3> angles;
  RotationMatrix(rotation).getRPY(angles[0], angles[1], angles[2]);
  return angles;
}

/**
 * @brief One step of an exponential moving average from `prev` towards `next`
 *
 * Position is interpolated linearly and orientation with slerp, which takes the short way
 * round when the two straddle +-pi.
 *
 * @param alpha Weight of `next`, clamped to [0, 1]: 1 returns `next`, 0 returns `prev`
 */
inline Rigid blendTransforms(const Rigid & prev, const Rigid & next, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);

  Quaternion rotation = prev.getRotation().slerp(next.getRotation(), alpha);
  rotation.normalize();

  return Rigid(rotation, prev.getOrigin().lerp(next.getOrigin(), alpha));
}

/**
 * @brief The vector counterpart of @ref blendTransforms
 */
inline Vector3 blendVectors(const Vector3 & prev, const Vector3 & next, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  return prev.lerp(next, alpha);
}

inline bool isSamePosition(
  const Vector3 & first, const Vector3 & second, double position_threshold = 1e-6)
{
  return (first - second).length() < position_threshold;
}

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__RIGID_HPP_
