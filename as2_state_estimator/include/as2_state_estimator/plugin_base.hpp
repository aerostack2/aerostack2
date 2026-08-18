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
#include <nav_msgs/msg/odometry.hpp>

#include <as2_core/node.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_core/names/topics.hpp>

namespace as2_state_estimator_plugin_base
{
class StateEstimatorBase
{
protected:
  as2::Node * node_ptr_;

private:
  std::string earth_frame_id_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();

public:
  /**
   * @brief Construct the State Estimator Base object. The plugin is only
   * usable after setup().
   */
  StateEstimatorBase() {}

  /**
   * @brief Wire the plugin to its node, TF handler and broadcasters, read the
   * frame parameters and hand control to on_setup().
   *
   * @param node Node hosting the plugin.
   * @param tf_handler TF handler shared with the state estimator node.
   * @param tf_broadcaster Broadcaster for the dynamic transforms.
   * @param static_tf_broadcaster Broadcaster for the static transforms.
   */
  void setup(
    as2::Node * node,
    std::shared_ptr<as2::tf::TfHandler> tf_handler,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster)
  {
    node_ptr_ = node;
    tf_handler_ = tf_handler;
    tf_broadcaster_ = tf_broadcaster;
    static_tf_broadcaster_ = static_tf_broadcaster;

    twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
    pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);

    // Declared, read and namespaced once by as2::Node, for the whole stack
    earth_frame_id_ = node_ptr_->getEarthFrameId();
    base_frame_id_ = node_ptr_->getBaseFrameId();
    odom_frame_id_ = node_ptr_->getOdomFrameId();
    map_frame_id_ = node_ptr_->getMapFrameId();

    on_setup();
  }
  /**
   * @brief Set the plugin up, once its node and broadcasters are available.
   */
  virtual void on_setup() = 0;

  /**
   * @brief Get the earth to map transform of this estimator.
   *
   * The default implementation warns and returns the identity, so a plugin
   * that does not localize against a map still produces a valid TF tree.
   *
   * @param transform Output. Earth to map transform.
   * @return true if the transform is valid.
   */
  virtual bool get_earth_to_map_transform(geometry_msgs::msg::TransformStamped & transform)
  {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "get_earth_to_map_transform not implemented using default identity transform");
    transform = as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    return true;
  }

protected:
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  /**
   * @brief Cache the earth to map transform when the given one is that link.
   *
   * @param transform Transform about to be published.
   */
  void check_standard_transform(const geometry_msgs::msg::TransformStamped & transform)
  {
    if (transform.header.frame_id == get_earth_frame() &&
      transform.child_frame_id == get_map_frame())
    {
      earth_to_map_ = tf2::Transform(
        tf2::Quaternion(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w),
        tf2::Vector3(
          transform.transform.translation.x, transform.transform.translation.y,
          transform.transform.translation.z));
    }
  }

  /**
   * @brief Broadcast a dynamic transform.
   *
   * @param transform Transform to broadcast.
   */
  inline void publish_transform(const geometry_msgs::msg::TransformStamped & transform)
  {
    tf_broadcaster_->sendTransform(transform);
  }
  /**
   * @brief Broadcast a static transform.
   *
   * @param transform Transform to broadcast.
   */
  inline void publish_static_transform(const geometry_msgs::msg::TransformStamped & transform)
  {
    static_tf_broadcaster_->sendTransform(transform);
  }

  /**
   * @brief Publish the estimated twist of the vehicle.
   *
   * @param twist Twist to publish.
   */
  inline void publish_twist(const geometry_msgs::msg::TwistStamped & twist)
  {
    twist_pub_->publish(twist);
  }
  /**
   * @brief Publish the estimated pose of the vehicle.
   *
   * @param pose Pose to publish.
   */
  inline void publish_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    pose_pub_->publish(pose);
  }

  /**
   * @brief Global earth frame id, shared by every robot.
   *
   * @return Frame id.
   */
  inline const std::string & get_earth_frame() const {return earth_frame_id_;}
  /**
   * @brief Namespaced map frame id of the robot.
   *
   * @return Frame id.
   */
  inline const std::string & get_map_frame() const {return map_frame_id_;}
  /**
   * @brief Namespaced odom frame id of the robot.
   *
   * @return Frame id.
   */
  inline const std::string & get_odom_frame() const {return odom_frame_id_;}
  /**
   * @brief Namespaced base_link frame id of the robot.
   *
   * @return Frame id.
   */
  inline const std::string & get_base_frame() const {return base_frame_id_;}

  /**
   * @brief Override the earth frame id, for a plugin with its own convention.
   *
   * @param frame Frame id to use.
   */
  inline void set_earth_frame(const std::string & frame) {earth_frame_id_ = frame;}
  /**
   * @brief Override the map frame id, for a plugin with its own convention.
   *
   * @param frame Frame id to use.
   */
  inline void set_map_frame(const std::string & frame) {map_frame_id_ = frame;}
  /**
   * @brief Override the odom frame id, for a plugin with its own convention.
   *
   * @param frame Frame id to use.
   */
  inline void set_odom_frame(const std::string & frame) {odom_frame_id_ = frame;}
  /**
   * @brief Override the base_link frame id, for a plugin with its own convention.
   *
   * @param frame Frame id to use.
   */
  inline void set_base_frame(const std::string & frame) {base_frame_id_ = frame;}

  tf2::Transform odom_to_baselink;
  tf2::Transform earth_to_map;
  tf2::Transform map_to_odom;
  tf2::Transform earth_to_baselink;

  bool static_transforms_published_ = false;
  rclcpp::TimerBase::SharedPtr static_transforms_timer_;

  /**
   * @brief Get the earth to map transform as a tf2::Transform.
   *
   * @param earth_to_map Output. Earth to map transform.
   * @return true if the transform is valid.
   */
  bool get_earth_to_map_transform(tf2::Transform & earth_to_map)
  {
    geometry_msgs::msg::TransformStamped transform;
    if (get_earth_to_map_transform(transform)) {
      tf2::fromMsg(transform.transform, earth_to_map);
      return true;
    }
    return false;
  }

  /**
   * @brief Express a pose known in earth as the odom to base_link transform.
   *
   * @param earth_to_baselink Pose of the vehicle in earth.
   * @param odom_to_baselink Output. Same pose, relative to odom.
   * @param earth_to_map Earth to map transform.
   * @param map_to_odom Map to odom transform. Defaults to the identity.
   * @return true always, the conversion cannot fail.
   */
  bool convert_earth_to_baselink_2_odom_to_baselink_transform(
    const tf2::Transform & earth_to_baselink,
    tf2::Transform & odom_to_baselink,
    const tf2::Transform & earth_to_map,
    const tf2::Transform & map_to_odom = tf2::Transform::getIdentity())
  {
    odom_to_baselink = map_to_odom.inverse() * earth_to_map.inverse() * earth_to_baselink;
    return true;
  }

  /**
   * @brief Express a pose known in odom as the earth to base_link transform.
   *
   * @param odom_to_baselink Pose of the vehicle in odom.
   * @param earth_to_baselink Output. Same pose, relative to earth.
   * @param earth_to_map Earth to map transform.
   * @param map_to_odom Map to odom transform. Defaults to the identity.
   * @return true always, the conversion cannot fail.
   */
  bool convert_odom_to_baselink_2_earth_to_baselink_transform(
    const tf2::Transform & odom_to_baselink,
    tf2::Transform & earth_to_baselink,
    const tf2::Transform & earth_to_map,
    const tf2::Transform & map_to_odom = tf2::Transform::getIdentity())
  {
    earth_to_baselink = earth_to_map * map_to_odom * odom_to_baselink;
    return true;
  }
};
}  // namespace as2_state_estimator_plugin_base

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_BASE_HPP_
