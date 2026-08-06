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
* @file robot_state.hpp
*
* An stucture to store the state of the drone
*
* @authors Miguel Fernández Cortizas
*/

#ifndef AS2_STATE_ESTIMATOR__ROBOT_STATE_HPP_
#define AS2_STATE_ESTIMATOR__ROBOT_STATE_HPP_

#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <string>
#include <variant>
#include <array>
#include <utility>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>


namespace as2_state_estimator
{

using StateComponent = std::variant<
  geometry_msgs::msg::PoseWithCovariance,
  geometry_msgs::msg::TwistWithCovariance>;

enum TransformInformatonType
{
  EARTH_TO_MAP,
  MAP_TO_ODOM,
  ODOM_TO_BASE,
  TWIST_IN_BASE
};

inline std::string TransformInformatonTypeToString(TransformInformatonType type)
{
  switch (type) {
    case EARTH_TO_MAP:
      return "earth_to_map";
    case MAP_TO_ODOM:
      return "map_to_odom";
    case ODOM_TO_BASE:
      return "odom_to_base";
    case TWIST_IN_BASE:
      return "twist_in_base";
  }
  return "unknown";
}

struct RobotState
{
  std::array<geometry_msgs::msg::PoseWithCovarianceStamped, 3> poses;
  // EARTH_TO_MAP, MAP_TO_ODOM, ODOM_TO_BASE
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  std::array<bool, 3> is_static_vec;
  std::array<bool, 4> has_been_updated;


  RobotState();

  void processStateComponent(
    const std::string & authority, const StateComponent & component,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);

  void processPose(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);


  void processTwist(
    const std::string & authority,
    const geometry_msgs::msg::TwistWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp);

  tf2::Transform getTransform(TransformInformatonType type) const;
  std::pair<geometry_msgs::msg::TransformStamped, bool> getTransformStamped(
    TransformInformatonType type) const;

  geometry_msgs::msg::TwistStamped getTwistStampedInBase();
  geometry_msgs::msg::PoseStamped getPoseStampedEarthToBase();
};


}  // namespace as2_state_estimator
#endif  // AS2_STATE_ESTIMATOR__ROBOT_STATE_HPP_
