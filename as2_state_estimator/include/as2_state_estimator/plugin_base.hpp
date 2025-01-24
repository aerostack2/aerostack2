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
*/

#ifndef AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
#define AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
#include <geometry_msgs/msg/detail/pose_with_covariance__struct.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
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
/*
 * @brief Interface for the state estimator plugin to provide the output of the state estimator
 * to the rest of the system
 */

inline geometry_msgs::msg::PoseWithCovariance convert_tf2_to_pose_with_covariance(
  const tf2::Transform & transform)
{
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = transform.getOrigin().getX();
  pose.pose.position.y = transform.getOrigin().getY();
  pose.pose.position.z = transform.getOrigin().getZ();
  pose.pose.orientation.x = transform.getRotation().getX();
  pose.pose.orientation.y = transform.getRotation().getY();
  pose.pose.orientation.z = transform.getRotation().getZ();
  pose.pose.orientation.w = transform.getRotation().getW();
  return pose;
}

inline tf2::Transform convert_pose_with_covariance_to_tf2(
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

class StateEstimatorInterface
{
public:
  virtual const std::string & getEarthFrame() = 0;
  virtual const std::string & getMapFrame() = 0;
  virtual const std::string & getOdomFrame() = 0;
  virtual const std::string & getBaseFrame() = 0;
  /**
   * @brief Set the pose of the map frame (local for each robot) in the earth frame (global)
   * @param pose The pose of the map frame in the earth frame with covariance
   */
  virtual void setEarthToMap(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;
  virtual void setEarthToMap(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;

  /**
   * @brief Set the pose of the robot from the map frame to the odom frame
   * @param pose The pose of the robot from the map frame to the odom frame with covariance
   */
  virtual void setMapToOdomPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  virtual void setMapToOdomPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  /**
   * @brief Set the pose of the robot from the odom frame to the base_link frame
   * @param pose The pose of the robot from the odom frame to the base_link frame with covariance
   */
  virtual void setOdomToBaseLinkPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;
  virtual void setOdomToBaseLinkPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  /**
   * @brief Set the twist of the robot in the base_link frame
   * @param twist The twist of the robot in the base_link frame with covariance
   */
  virtual void setTwistInBaseFrame(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp) = 0;

  virtual tf2::Transform getEarthToMapTransform() = 0;
  virtual tf2::Transform getMapToOdomTransform() = 0;
  virtual tf2::Transform getOdomToBaseLinkTransform() = 0;
};


namespace as2_state_estimator_plugin_base
{

/**
  * @brief Base class for the state estimator plugin
  * This class will be used as a base class for the state estimator plugins
  * this plugin is in charge of providing the state estimation to the rest of the system
  * by calling the set functions of the StateEstimatorInterface class.
  * WARNING: In order to use the TF tree it should be used through the tf_handler_ member of the class.
  */
class StateEstimatorBase
{
protected:
  as2::Node * node_ptr_ = nullptr;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::shared_ptr<StateEstimatorInterface> state_estimator_interface_;

public:
  using SharedPtr = std::shared_ptr<StateEstimatorBase>;
  StateEstimatorBase() {}
  void setup(
    as2::Node * node,
    std::shared_ptr<as2::tf::TfHandler> tf_handler,
    std::shared_ptr<StateEstimatorInterface> state_estimator_interface
  )
  {
    node_ptr_ = node;
    tf_handler_ = tf_handler;
    state_estimator_interface_ = state_estimator_interface;
    // !! WATCHOUT : earth_frame_id_ is not generated because it is a global frame
    onSetup();
  }
  virtual void onSetup() = 0;
};
}  // namespace as2_state_estimator_plugin_base

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
