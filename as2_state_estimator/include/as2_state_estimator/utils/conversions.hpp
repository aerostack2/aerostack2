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
*
*/

#ifndef AS2_STATE_ESTIMATOR__CONVERSIONS_HPP_
#define AS2_STATE_ESTIMATOR__CONVERSIONS_HPP_

#include <tf2/LinearMath/Transform.h>

#include <array>
#include <optional>

#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace as2_state_estimator
{
namespace conversions
{

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

inline bool convert_earth_to_baselink_2_odom_to_baselink_transform(
  const tf2::Transform & earth_to_baselink,
  tf2::Transform & odom_to_baselink,
  const tf2::Transform & earth_to_map,
  const tf2::Transform & map_to_odom = tf2::Transform::getIdentity())
{
  odom_to_baselink = map_to_odom.inverse() * earth_to_map.inverse() * earth_to_baselink;
  return true;
}

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

#endif  // AS2_STATE_ESTIMATOR__CONVERSIONS_HPP_
