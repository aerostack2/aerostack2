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

/**
 * @brief Either half of a state update: a pose or a twist, both with covariance.
 *
 * The alternative held selects the branch taken by RobotState::processStateComponent().
 */
using StateComponent = std::variant<
  geometry_msgs::msg::PoseWithCovariance,
  geometry_msgs::msg::TwistWithCovariance>;

/**
 * @brief Identifies which link of the earth -> map -> odom -> base chain an update refers to.
 *
 * The three pose values are used directly as indices into RobotState::poses and
 * RobotState::is_static_vec, so their numeric order is part of the contract. TWIST_IN_BASE is
 * not a link: it indexes only RobotState::has_been_updated and refers to the body twist
 * expressed in the base frame.
 */
enum TransformInformatonType
{
  EARTH_TO_MAP,
  MAP_TO_ODOM,
  ODOM_TO_BASE,
  TWIST_IN_BASE
};

/**
 * @brief Get the snake_case name of a transform information type.
 *
 * @param type Transform information type.
 * @return Its name, or "unknown" if the value is not a valid enumerator.
 */
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

/**
 * @brief Snapshot of the robot state: the three links of the TF chain plus the body twist.
 *
 * Every plugin owns one instance and writes only into it; the StateEstimator owns a shared
 * instance into which it copies the entries whose authority it accepts. Frame ids are never
 * stored here, they are resolved from the StateEstimator frame names when a message is built.
 */
struct RobotState
{
  /**
   * @brief Pose of each link, indexed by TransformInformatonType.
   *
   * Only header.stamp is populated by the process methods; header.frame_id stays empty.
   */
  std::array<geometry_msgs::msg::PoseWithCovarianceStamped, 3> poses;
  // EARTH_TO_MAP, MAP_TO_ODOM, ODOM_TO_BASE

  /**
   * @brief Body twist assumed to be expressed in the base frame. Only header.stamp is populated.
   */
  geometry_msgs::msg::TwistWithCovarianceStamped twist;

  /**
   * @brief Per link, whether it must be broadcast as a static transform.
   *
   * Written by every processPose() call, so the last update wins, and reported as the second
   * element of getTransformStamped(). It has no effect on getTransform().
   */
  std::array<bool, 3> is_static_vec;

  /**
   * @brief Per entry, whether it has ever been written, indexed by TransformInformatonType.
   *
   * Starts all false and is never cleared, so it means "ever received a value", not "fresh".
   * It gates getTransform(), which delegates to the StateEstimator shared state when false, and
   * getTransformStamped(), which throws when false.
   */
  std::array<bool, 4> has_been_updated;


  /**
   * @brief Construct an empty state with every pose and the twist default constructed.
   *
   * All is_static_vec and has_been_updated entries start false, so until the first update
   * getTransform() delegates to the StateEstimator shared state and getTransformStamped() throws.
   */
  RobotState();

  /**
   * @brief Dispatch a state component to processPose() or processTwist() by the alternative held.
   *
   * @param authority Name of the plugin providing the update. Currently unused, no authority
   *        check happens here; the StateEstimator validates it before copying the entry.
   * @param component Pose or twist to store.
   * @param type Entry to update. Must be TWIST_IN_BASE for a twist and any other value for a pose.
   * @param stamp Stamp written into the entry header, not validated against the previous one.
   * @param is_static Whether the link must be broadcast as static. Ignored for a twist.
   * @throw std::invalid_argument If type does not match the alternative held by component.
   */
  void processStateComponent(
    const std::string & authority, const StateComponent & component,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);

  /**
   * @brief Store the pose of one link and mark that link as updated.
   *
   * The previous value is overwritten unconditionally: no covariance, frame or stamp
   * monotonicity check is done, and is_static may flip from one call to the next.
   *
   * @param authority Name of the plugin providing the update. Currently unused.
   * @param msg Pose with covariance to store, copied as is.
   * @param type Link to update. TWIST_IN_BASE indexes past the end of the pose array and is
   *        undefined behaviour; processStateComponent() rejects that combination beforehand.
   * @param stamp Stamp written into the entry header.
   * @param is_static Whether the link must be broadcast as a static transform.
   */
  void processPose(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const as2_state_estimator::TransformInformatonType & type,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);


  /**
   * @brief Store the body twist and mark TWIST_IN_BASE as updated.
   *
   * @param authority Name of the plugin providing the update. Currently unused.
   * @param msg Twist with covariance, assumed to be expressed in the base frame.
   * @param stamp Stamp written into the twist header.
   */
  void processTwist(
    const std::string & authority,
    const geometry_msgs::msg::TwistWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief Transform of a link, falling back to the shared state when this instance lacks it.
   *
   * When has_been_updated is false for @p type, the value is taken from the state estimator shared
   * robot state instead, which is seeded as updated and therefore ends the recursion. A plugin thus
   * reads the links it does not estimate itself as whatever the node last published.
   *
   * @param type Link to read.
   * @return Transform of the link, the identity while nobody has written it.
   */
  tf2::Transform getTransform(TransformInformatonType type) const;

  /**
   * @brief Transform of a link as a stamped message, ready to broadcast.
   *
   * The frame ids are resolved from the state estimator, not stored here, and the returned flag
   * says whether the link must go to the static broadcaster.
   *
   * @param type Link to read. TWIST_IN_BASE is not a link and is rejected.
   * @return The stamped transform and whether it must be broadcast as static.
   * @throw std::runtime_error if the link has never been updated.
   * @throw std::invalid_argument if @p type is TWIST_IN_BASE.
   */
  std::pair<geometry_msgs::msg::TransformStamped, bool> getTransformStamped(
    TransformInformatonType type) const;

  /**
   * @brief Body twist as a stamped message, in the base frame of the state estimator.
   *
   * The covariance is dropped, since TwistStamped has no room for it.
   *
   * @return Twist stamped with the time of the last processTwist() call.
   */
  geometry_msgs::msg::TwistStamped getTwistStampedInBase();

  /**
   * @brief Pose of the robot in the earth frame, composed from the three links.
   *
   * Composes earth->map * map->odom * odom->base_link, each read through getTransform() and so
   * subject to its fallback. The stamp is taken from odom->base_link alone, the fastest link, so
   * the three are not checked for a common instant.
   *
   * @return Pose of base_link in the earth frame.
   */
  geometry_msgs::msg::PoseStamped getPoseStampedEarthToBase();
};


}  // namespace as2_state_estimator
#endif  // AS2_STATE_ESTIMATOR__ROBOT_STATE_HPP_
