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
* @file conversions.hpp
*
* A set of useful conversions between different types
*
* @authors Miguel Fernández Cortizas
*          Rodrigo Da Silva Gómez
*
*/

#ifndef AS2_STATE_ESTIMATOR__UTILS__CONVERSIONS_HPP_
#define AS2_STATE_ESTIMATOR__UTILS__CONVERSIONS_HPP_

#include <tf2/LinearMath/Transform.h>

#include <array>
#include <optional>

#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace as2_state_estimator
{
namespace conversions
{

/**
 * @brief Conversions between tf2 transforms and ROS pose messages, and between the
 * earth-relative and the odom-relative robot pose.
 *
 * A transform named `a_to_b` is the pose of frame `b` expressed in frame `a`, and therefore maps
 * points from `b` into `a`. Frames follow the AS2 canonical chain earth -> map -> odom ->
 * base_link. Translations are in metres and rotations are unit quaternions. tf2::Transform carries
 * neither frame id nor timestamp, so keeping the arguments consistent in frame and in time is
 * entirely the caller's responsibility.
 */

/**
 * @brief Copy a rigid transform into a PoseWithCovariance message, optionally attaching a
 * covariance matrix.
 *
 * The message carries no frame id, so the result is expressed in the parent frame of
 * @p transform and only the caller knows which one that is. The quaternion is copied verbatim,
 * without normalisation. When @p covariance is not given, the covariance field keeps its default
 * all-zeros value, which downstream consumers read as a perfectly known pose rather than as an
 * unknown one.
 *
 * @param transform Rigid transform to convert, i.e. the pose of its child frame in its parent,
 * with the translation in metres.
 * @param covariance Optional ROS 6x6 row-major covariance expressed in the same frame as
 * @p transform, ordered [x, y, z, rot_x, rot_y, rot_z], in m^2, m*rad and rad^2.
 * @return Pose message holding the translation and rotation of @p transform.
 */
inline geometry_msgs::msg::PoseWithCovariance convert_to_pose_with_covariance(
  const tf2::Transform & transform, std::optional<std::array<double, 36>> covariance = std::nullopt)
{
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = transform.getOrigin().getX();
  pose.pose.position.y = transform.getOrigin().getY();
  pose.pose.position.z = transform.getOrigin().getZ();
  pose.pose.orientation.x = transform.getRotation().getX();
  pose.pose.orientation.y = transform.getRotation().getY();
  pose.pose.orientation.z = transform.getRotation().getZ();
  pose.pose.orientation.w = transform.getRotation().getW();
  if (covariance.has_value()) {
    pose.covariance = covariance.value();
  }
  return pose;
}

/**
 * @brief Extract the pose of a PoseWithCovariance message as a rigid transform.
 *
 * The covariance is dropped, and the frame the pose refers to is not recoverable from the result,
 * since tf2::Transform stores neither frame id nor timestamp. The quaternion is taken as is,
 * neither normalised nor validated, so a zero-length one yields an invalid rotation.
 *
 * @param pose Pose with covariance to convert, with the position in metres, expressed in whatever
 * frame the caller obtained it from.
 * @return Rigid transform holding only the translation and rotation of @p pose.
 */
inline tf2::Transform convert_to_tf2_transform(
  const geometry_msgs::msg::PoseWithCovariance & pose)
{
  tf2::Transform transform;
  transform.setOrigin(
    tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  transform.setRotation(
    tf2::Quaternion(
      pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w));
  return transform;
}

/**
 * @brief Rebase a robot pose given in the earth frame into the odom frame.
 *
 * Computes odom_to_baselink = map_to_odom^-1 * earth_to_map^-1 * earth_to_baselink, which is the
 * transform a state estimator plugin publishes as odom -> base_link. The four transforms are
 * assumed to refer to the same instant: nothing here is time stamped, interpolated or checked for
 * staleness, and mixing epochs silently produces a wrong pose instead of an error.
 *
 * @param earth_to_baselink Pose of base_link in the earth frame, translation in metres.
 * @param odom_to_baselink Output pose of base_link in the odom frame, always overwritten.
 * @param earth_to_map Pose of the robot local map frame in the earth frame.
 * @param map_to_odom Pose of the odom frame in the map frame; defaults to identity, which assumes
 * an odometry origin coincident with map, i.e. no map correction applied.
 * @return Always true; the operation cannot fail, so this is not a success flag.
 */
inline bool convert_earth_to_baselink_2_odom_to_baselink_transform(
  const tf2::Transform & earth_to_baselink,
  tf2::Transform & odom_to_baselink,
  const tf2::Transform & earth_to_map,
  const tf2::Transform & map_to_odom = tf2::Transform::getIdentity())
{
  odom_to_baselink = map_to_odom.inverse() * earth_to_map.inverse() * earth_to_baselink;
  return true;
}

/**
 * @brief Rebase a robot pose given in the odom frame into the earth frame.
 *
 * Computes earth_to_baselink = earth_to_map * map_to_odom * odom_to_baselink, the inverse of
 * convert_earth_to_baselink_2_odom_to_baselink_transform(). The four transforms are assumed to
 * refer to the same instant: nothing here is time stamped, interpolated or checked for staleness,
 * and mixing epochs silently produces a wrong pose instead of an error.
 *
 * @param odom_to_baselink Pose of base_link in the odom frame, translation in metres.
 * @param earth_to_baselink Output pose of base_link in the earth frame, always overwritten.
 * @param earth_to_map Pose of the robot local map frame in the earth frame.
 * @param map_to_odom Pose of the odom frame in the map frame; defaults to identity, which assumes
 * an odometry origin coincident with map, i.e. no map correction applied.
 * @return Always true; the operation cannot fail, so this is not a success flag.
 */
inline bool convert_odom_to_baselink_2_earth_to_baselink_transform(
  const tf2::Transform & odom_to_baselink,
  tf2::Transform & earth_to_baselink,
  const tf2::Transform & earth_to_map,
  const tf2::Transform & map_to_odom = tf2::Transform::getIdentity())
{
  earth_to_baselink = earth_to_map * map_to_odom * odom_to_baselink;
  return true;
}

}  // namespace conversions
}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__UTILS__CONVERSIONS_HPP_
