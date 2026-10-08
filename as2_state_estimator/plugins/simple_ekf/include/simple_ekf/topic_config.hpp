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
* @file topic_config.hpp
*
* The configuration of one update topic: how the filter treats its measurements, plus
* what only the plugin needs to know about it
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF__TOPIC_CONFIG_HPP_
#define SIMPLE_EKF__TOPIC_CONFIG_HPP_

#include <string>

#include <simple_ekf_core/types.hpp>

namespace simple_ekf
{

/// The message types an update topic can carry, as the `type` parameter names them
inline constexpr char kPoseStampedType[] = "geometry_msgs/msg/PoseStamped";
inline constexpr char kPoseWithCovarianceStampedType[] =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
inline constexpr char kOdometryType[] = "nav_msgs/msg/Odometry";
inline constexpr char kTwistWithCovarianceStampedType[] =
  "geometry_msgs/msg/TwistWithCovarianceStamped";
inline constexpr char kRigidBodiesType[] = "mocap4r2_msgs/msg/RigidBodies";

/**
 * @brief One update topic: the filter's SourceConfig, and the fields only the plugin reads
 */
struct TopicConfig : simple_ekf_core::SourceConfig
{
  std::string topic;

  /// One of the k*Type names above
  std::string type;

  /// Whether this topic's first pose sets earth->map
  bool set_earth_map = false;

  /// For a RigidBodies topic, the body to track
  std::string rigid_body_name;

  /// For a velocity topic, whether it is in the vehicle's frame whatever its header says
  bool is_body_frame = false;
};

/**
 * @brief Whether a topic's type carries a velocity rather than a pose
 */
inline bool isVelocityType(const std::string & type)
{
  return type == kTwistWithCovarianceStampedType;
}

/**
 * @brief Whether a topic's type carries a covariance of its own
 */
inline bool carriesCovariance(const std::string & type)
{
  return type == kPoseWithCovarianceStampedType || type == kOdometryType ||
         type == kTwistWithCovarianceStampedType;
}

/**
 * @brief Default of a topic's `is_odometry` when the user does not set it
 *
 * Odometry is, by convention, a dead-reckoned pose that drifts with respect to the map, so
 * its correction should be absorbed by odom->base rather than moving map->odom. Every
 * other supported type carries an absolute pose.
 */
inline bool defaultIsOdometryForType(const std::string & type)
{
  return type == kOdometryType;
}

/**
 * @brief Default of a topic's `reject_repeated_positions` when the user does not set it
 *
 * A motion capture system keeps publishing the last known pose when its cameras lose the
 * rigid body, and a tracked body always jitters, so an exactly repeated position means the
 * tracking was lost. Feeding those repeats to the filter makes it increasingly confident
 * about a position nobody is measuring any more. Any other type may legitimately repeat a
 * position, which just means the robot is not moving.
 */
inline bool defaultRejectRepeatedPositionsForType(const std::string & type)
{
  return type == kRigidBodiesType;
}

}  // namespace simple_ekf

#endif  // SIMPLE_EKF__TOPIC_CONFIG_HPP_
