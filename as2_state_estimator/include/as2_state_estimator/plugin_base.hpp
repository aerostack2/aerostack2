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
* @file plugin_base.hpp
*
* An state estimation plugin base for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Rodrigo Da Silva Gómez
*/

#ifndef AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
#define AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <vector>
#include <string>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <as2_core/node.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include "as2_state_estimator/robot_state.hpp"

/**
 * @brief Interface a plugin uses to hand its estimate to the state estimator node.
 *
 * The node owns the frame ids and the broadcasting: a plugin reports the links it
 * estimates through the set methods and never publishes them itself.
 */
class StateEstimatorInterface
{
public:
  /**
   * @brief Global earth frame id, shared by every robot.
   *
   * @return Frame id.
   */
  virtual const std::string & getEarthFrame() = 0;

  /**
   * @brief Namespaced map frame id of the robot.
   *
   * @return Frame id.
   */
  virtual const std::string & getMapFrame() = 0;

  /**
   * @brief Namespaced odom frame id of the robot.
   *
   * @return Frame id.
   */
  virtual const std::string & getOdomFrame() = 0;

  /**
   * @brief Namespaced base_link frame id of the robot.
   *
   * @return Frame id.
   */
  virtual const std::string & getBaseFrame() = 0;

  /**
   * @brief Set the pose of the map frame (local to each robot) in the earth frame (global).
   *
   * @param pose Pose of map in earth, with covariance.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  virtual void setEarthToMap(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;

  /**
   * @brief Set the pose of the map frame in the earth frame, without covariance.
   *
   * @param pose Pose of map in earth.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  virtual void setEarthToMap(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;

  /**
   * @brief Set the pose of the odom frame in the map frame.
   *
   * @param pose Pose of odom in map, with covariance.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  virtual void setMapToOdomPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;

  /**
   * @brief Set the pose of the odom frame in the map frame, without covariance.
   *
   * @param pose Pose of odom in map.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  virtual void setMapToOdomPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;

  /**
   * @brief Set the pose of the base_link frame in the odom frame.
   *
   * @param pose Pose of base_link in odom, with covariance.
   * @param stamp Time the pose refers to.
   */
  virtual void setOdomToBaseLinkPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp) = 0;

  /**
   * @brief Set the pose of the base_link frame in the odom frame, without covariance.
   *
   * @param pose Pose of base_link in odom.
   * @param stamp Time the pose refers to.
   */
  virtual void setOdomToBaseLinkPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp) = 0;

  /**
   * @brief Set the twist of the robot, expressed in the base_link frame.
   *
   * @param twist Twist of the robot in base_link, with covariance.
   * @param stamp Time the twist refers to.
   */
  virtual void setTwistInBaseFrame(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp) = 0;

  /**
   * @brief Last earth to map transform known to the state estimator.
   *
   * @return Transform, the identity while no plugin has set it.
   */
  virtual tf2::Transform getEarthToMapTransform() = 0;

  /**
   * @brief Last map to odom transform known to the state estimator.
   *
   * @return Transform, the identity while no plugin has set it.
   */
  virtual tf2::Transform getMapToOdomTransform() = 0;

  /**
   * @brief Last odom to base_link transform known to the state estimator.
   *
   * @return Transform, the identity while no plugin has set it.
   */
  virtual tf2::Transform getOdomToBaseLinkTransform() = 0;
};


namespace as2_state_estimator_plugin_base
{


/**
 * @brief Base class for the state estimator plugins.
 *
 * A plugin estimates one or more links of the TF chain and reports them by calling the
 * set methods of StateEstimatorInterface, which is what makes them reach the rest of the
 * system. The TF tree must be read through the tf_handler_ member.
 */
class StateEstimatorBase
{
protected:
  as2::Node * node_ptr_ = nullptr;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::shared_ptr<StateEstimatorInterface> state_estimator_interface_;

public:
  using SharedPtr = std::shared_ptr<StateEstimatorBase>;

  /**
   * @brief Construct the plugin. It is only usable after setup().
   */
  StateEstimatorBase() {}

  /**
   * @brief Destroy the plugin.
   */
  virtual ~StateEstimatorBase() = default;

  /**
   * @brief Wire the plugin to its node, TF handler and estimator, and hand control to onSetup().
   *
   * @param node Node hosting the plugin.
   * @param tf_handler TF handler shared with the state estimator node.
   * @param state_estimator_interface Interface the plugin reports its estimate through.
   */
  void setup(
    as2::Node * node,
    std::shared_ptr<as2::tf::TfHandler> tf_handler,
    std::shared_ptr<StateEstimatorInterface> state_estimator_interface
  )
  {
    node_ptr_ = node;
    tf_handler_ = tf_handler;
    state_estimator_interface_ = state_estimator_interface;
    onSetup();
  }

  /**
   * @brief Set the plugin up, once its node and interface are available.
   */
  virtual void onSetup() = 0;

  /**
   * @brief Links of the TF chain this plugin is allowed to estimate.
   *
   * The state estimator rejects any update the plugin sends for a link not listed here.
   *
   * @return Transform types the plugin provides.
   */
  virtual std::vector<as2_state_estimator::TransformInformatonType>
  getTransformationTypesAvailable() const = 0;

  /**
   * @brief Generate an identity pose (useful for initializing transforms)
   *
   * @return geometry_msgs::msg::PoseWithCovariance Identity pose with zero covariance
   */
  static geometry_msgs::msg::PoseWithCovariance generateIdentityPose()
  {
    geometry_msgs::msg::PoseWithCovariance identity_pose;
    identity_pose.pose.position.x = 0.0;
    identity_pose.pose.position.y = 0.0;
    identity_pose.pose.position.z = 0.0;
    identity_pose.pose.orientation.x = 0.0;
    identity_pose.pose.orientation.y = 0.0;
    identity_pose.pose.orientation.z = 0.0;
    identity_pose.pose.orientation.w = 1.0;
    return identity_pose;
  }

  /**
   * @brief Check if two poses are the same within a threshold
   *
   * @param pose1 First pose to compare
   * @param pose2 Second pose to compare
   * @param position_threshold Distance threshold for considering poses equal (default: 1e-6)
   * @return true if poses are within threshold, false otherwise
   */
  static bool isSamePose(
    const tf2::Vector3 & pose1, const tf2::Vector3 & pose2,
    double position_threshold = 1e-6)
  {
    return (pose1 - pose2).length() < position_threshold;
  }
};
}  // namespace as2_state_estimator_plugin_base

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
