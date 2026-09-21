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
* @file transform_utils.hpp
*
* The frame tree the filter maintains, and what it takes to move a measurement into the
* map frame or the state back out of it
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__TRANSFORM_UTILS_HPP_
#define SIMPLE_EKF_CORE__TRANSFORM_UTILS_HPP_

#include <Eigen/Dense>

#include <array>

#include "ekf/ekf_datatype.hpp"

#include "simple_ekf_core/rigid.hpp"
#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

/**
 * @brief The three transforms of the tree below the map frame, as the state implies them
 */
struct StateTransforms
{
  Rigid map_to_base = Rigid::getIdentity();
  Rigid map_to_odom = Rigid::getIdentity();
  Rigid odom_to_base = Rigid::getIdentity();

  StateTransforms() = default;

  StateTransforms(
    const Rigid & map_to_base_transform,
    const Rigid & map_to_odom_transform,
    const Rigid & odom_to_base_transform)
  : map_to_base(map_to_base_transform),
    map_to_odom(map_to_odom_transform),
    odom_to_base(odom_to_base_transform)
  {}

  /**
   * @brief map->base from the state's pose, and odom->base as what is left of it once
   *        map->odom is taken out.
   */
  explicit StateTransforms(
    const ekf::State & state,
    const Rigid & map_to_odom_transform = Rigid::getIdentity())
  : map_to_odom(map_to_odom_transform)
  {
    const auto position = state.get_position();
    const auto orientation = state.get_orientation_quaternion();

    Quaternion rotation(orientation[0], orientation[1], orientation[2], orientation[3]);
    rotation.normalize();

    map_to_base.setOrigin(Vector3(position[0], position[1], position[2]));
    map_to_base.setRotation(rotation);
    odom_to_base = map_to_odom.inverse() * map_to_base;
  }
};

/**
 * @brief The transform that takes a measurement from `frame` into the map frame
 */
inline Rigid transformToMapFrame(
  const StateTransforms & transforms,
  const Rigid & earth_to_map,
  SourceFrame frame)
{
  switch (frame) {
    case SourceFrame::EARTH:
      return earth_to_map.inverse();
    case SourceFrame::ODOM:
      return transforms.map_to_odom;
    case SourceFrame::BASE:
      return transforms.map_to_base;
    case SourceFrame::MAP:
    default:
      return Rigid::getIdentity();
  }
}

/// A 6x6 covariance stored row-major in 36 doubles, as the samples carry it
using Covariance6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

/**
 * @brief A pose measurement expressed in the map frame instead of its own
 *
 * Both 3x3 blocks of the covariance are rotated with the pose. The cross terms between
 * position and orientation are not carried over: the EKF only takes the diagonal.
 */
inline PoseSample transformPoseToMapFrame(
  const StateTransforms & transforms,
  const Rigid & earth_to_map,
  const PoseSample & pose)
{
  const Rigid to_map = transformToMapFrame(transforms, earth_to_map, pose.frame);
  const Eigen::Matrix3d rotation = rotationToEigenMatrix3d(to_map.getBasis());

  const Eigen::Map<const Covariance6> covariance_in(pose.covariance.data());
  const Eigen::Matrix3d position_in = covariance_in.block<3, 3>(0, 0);
  const Eigen::Matrix3d orientation_in = covariance_in.block<3, 3>(3, 3);

  // Assigned rather than initialised: Eigen evaluates a chained product differently in the
  // two cases, and switching changes the filter's output in the last bits
  Eigen::Matrix3d position_out;
  Eigen::Matrix3d orientation_out;
  position_out = rotation * position_in * rotation.transpose();
  orientation_out = rotation * orientation_in * rotation.transpose();

  PoseSample result;
  result.stamp = pose.stamp;
  result.frame = SourceFrame::MAP;
  result.pose = to_map * pose.pose;

  Eigen::Map<Covariance6> covariance_out(result.covariance.data());
  covariance_out.block<3, 3>(0, 0) = position_out;
  covariance_out.block<3, 3>(3, 3) = orientation_out;
  return result;
}

/**
 * @brief A velocity measurement expressed in the map frame instead of its own
 *
 * A velocity is a free vector, so only the rotation of its frame applies. Only the
 * diagonal of the rotated covariance is kept, since the velocity update takes one variance
 * per axis: dropping the correlation is small for a source whose axes are equally noisy,
 * and it never understates a variance.
 */
inline TwistSample transformTwistToMapFrame(
  const StateTransforms & transforms,
  const Rigid & earth_to_map,
  const TwistSample & twist)
{
  const Rigid to_map = transformToMapFrame(transforms, earth_to_map, twist.frame);
  const RotationMatrix & rotation_matrix = to_map.getBasis();
  const Eigen::Matrix3d rotation = rotationToEigenMatrix3d(rotation_matrix);

  const Eigen::Matrix3d covariance_in =
    Eigen::Map<const Covariance6>(twist.covariance.data()).block<3, 3>(0, 0);
  const Eigen::Matrix3d covariance_out = rotation * covariance_in * rotation.transpose();

  const Vector3 linear = rotation_matrix *
    Vector3(twist.linear[0], twist.linear[1], twist.linear[2]);

  TwistSample result = twist;
  result.frame = SourceFrame::MAP;
  result.linear = {linear.x(), linear.y(), linear.z()};
  result.covariance[0] = covariance_out(0, 0);
  result.covariance[7] = covariance_out(1, 1);
  result.covariance[14] = covariance_out(2, 2);
  return result;
}

/**
 * @brief The vehicle's twist in its own frame
 *
 * The linear velocity is passed in rather than read from `state` so that the caller can
 * give either the raw EKF velocity or one corrected for the smoothed map->odom (see
 * Filter::updateOutputs). The angular rate is the IMU's, less the estimated gyro bias.
 */
inline TwistInBase ekfStateToTwist(
  const ekf::State & state,
  const Rigid & map_to_base,
  const std::array<double, 3> & angular_velocity,
  const Vector3 & velocity_in_map)
{
  TwistInBase twist;
  twist.linear = map_to_base.getBasis().transpose() * velocity_in_map;
  twist.angular = Vector3(
    angular_velocity[0] - state.data[ekf::State::WBX],
    angular_velocity[1] - state.data[ekf::State::WBY],
    angular_velocity[2] - state.data[ekf::State::WBZ]);
  return twist;
}

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__TRANSFORM_UTILS_HPP_
