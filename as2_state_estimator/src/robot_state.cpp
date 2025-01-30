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
* @file robot_state.cpp
*
* Implementation of the RobotState class
*
* @authors Miguel Fernández Cortizas
*/

#include "as2_state_estimator/robot_state.hpp"

#include "as2_state_estimator/as2_state_estimator.hpp"

#include <utility>

namespace as2_state_estimator
{

RobotState::RobotState()
{
  poses.fill(geometry_msgs::msg::PoseWithCovarianceStamped());
  twist = geometry_msgs::msg::TwistWithCovarianceStamped();
  is_static_vec.fill(false);
  has_been_updated.fill(false);

  // TODO(miferco97): initialize frames at the beginning
}


void RobotState::processStateComponent(
  const std::string & authority, const StateComponent & component,
  const as2_state_estimator::TransformInformatonType & type,
  const builtin_interfaces::msg::Time & stamp,
  bool is_static)
{
  std::visit(
    [this, &authority, &type, &stamp, &is_static](auto && arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, geometry_msgs::msg::PoseWithCovariance>) {
        // assert type is not TWIST_IN_BASE
        if (type == TWIST_IN_BASE) {
          throw std::invalid_argument("TWIST_IN_BASE is not a valid type for a Pose");
        }
        processPose(authority, arg, type, stamp, is_static);

      } else if constexpr (std::is_same_v<T, geometry_msgs::msg::TwistWithCovariance>) {
        // assert type is TWIST_IN_BASE
        if (type != TWIST_IN_BASE) {
          throw std::invalid_argument("TWIST_IN_BASE is the only valid type for a Twist");
        }
        processTwist(authority, arg, stamp);
      }
    }, component);
}

void RobotState::processPose(
  const std::string & authority,
  const geometry_msgs::msg::PoseWithCovariance & msg,
  const as2_state_estimator::TransformInformatonType & type,
  const builtin_interfaces::msg::Time & stamp,
  bool is_static)
{
  auto & pose = poses[static_cast<int>(type)];
  pose.pose = msg;
  pose.header.stamp = stamp;
  is_static_vec[static_cast<int>(type)] = is_static;
  has_been_updated[static_cast<int>(type)] = true;
}


void RobotState::processTwist(
  const std::string & authority,
  const geometry_msgs::msg::TwistWithCovariance & msg,
  const builtin_interfaces::msg::Time & stamp)
{
  twist.twist = msg;
  twist.header.stamp = stamp;
  has_been_updated[TransformInformatonType::TWIST_IN_BASE] = true;
}

tf2::Transform RobotState::getTransform(TransformInformatonType type) const
{
  tf2::Transform transform;
  // first check if the transform has been updated
  if (has_been_updated[static_cast<int>(type)]) {
    auto & pose = poses[static_cast<int>(type)];
    tf2::fromMsg(pose.pose.pose, transform);
    return transform;
  }
  // if not look for the transform in the StateEstimator Class

  return as2_state_estimator::StateEstimator::getRobotState().getTransform(type);
}

std::pair<geometry_msgs::msg::TransformStamped, bool> RobotState::getTransformStamped(
  TransformInformatonType type) const
{
  if (!has_been_updated[static_cast<int>(type)]) {
    throw std::runtime_error(
            "getTransformStamped shall only be called after the transform has been updated");
  }
  if (type == TransformInformatonType::TWIST_IN_BASE) {
    throw std::invalid_argument("TWIST_IN_BASE is not a valid type for a TransformStamped");
  }
  geometry_msgs::msg::TransformStamped transform;
  auto & pose = poses[static_cast<int>(type)];
  transform.transform = tf2::toMsg(getTransform(type));
  transform.header = pose.header;
  auto [parent_frame, child_frame] = as2_state_estimator::StateEstimator::getFramesFromType(type);
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  return {transform, is_static_vec[static_cast<int>(type)]};
}

geometry_msgs::msg::TwistStamped RobotState::getTwistStampedInBase()
{
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.frame_id = as2_state_estimator::StateEstimator::getBaseFrame();
  twist_stamped.header.stamp = twist.header.stamp;
  twist_stamped.twist = twist.twist.twist;
  return twist_stamped;
}

geometry_msgs::msg::PoseStamped RobotState::getPoseStampedEarthToBase()
{
  // compose the pose from the transforms

  auto tf_earth_to_map = getTransform(TransformInformatonType::EARTH_TO_MAP);
  auto tf_map_to_odom = getTransform(TransformInformatonType::MAP_TO_ODOM);
  auto tf_odom_to_base = getTransform(TransformInformatonType::ODOM_TO_BASE);
  auto tf_earth_to_base = tf_earth_to_map * tf_map_to_odom * tf_odom_to_base;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = as2_state_estimator::StateEstimator::getEarthFrame();
  pose.header.stamp = poses[TransformInformatonType::ODOM_TO_BASE].header.stamp;
  tf2::toMsg(tf_earth_to_base, pose.pose);
  return pose;
}


}  // namespace as2_state_estimator
