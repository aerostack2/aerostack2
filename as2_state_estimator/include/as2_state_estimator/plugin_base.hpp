/*!*******************************************************************************************
 *  \file       plugin_base.hpp
 *  \brief      An state estimation plugin base for AeroStack2
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef __STATE_ESTIMATOR_PLUGIN_BASE_HPP__
#define __STATE_ESTIMATOR_PLUGIN_BASE_HPP__

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <as2_core/node.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>

#include <as2_core/names/topics.hpp>

namespace as2_state_estimator_plugin_base {
class StateEstimatorBase {
protected:
  as2::Node* node_ptr_;

private:
  std::string earth_frame_id_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_  = tf2::Transform::getIdentity();
  tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();

public:
  StateEstimatorBase(){};
  void setup(as2::Node* node,
             std::shared_ptr<tf2_ros::Buffer> tf_buffer,
             std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
             std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster) {
    node_ptr_              = node;
    tf_buffer_             = tf_buffer;
    tf_broadcaster_        = tf_broadcaster;
    static_tf_broadcaster_ = static_tf_broadcaster;

    twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
    pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);

    // node_ptr_->declare_parameter<std::string>("base_frame", "base_link");
    // node_ptr_->declare_parameter<std::string>("global_ref_frame", "earth");
    // node_ptr_->declare_parameter<std::string>("odom_frame", "odom");
    // node_ptr_->declare_parameter<std::string>("map_frame", "map");

    node_ptr_->get_parameter("base_frame", base_frame_id_);
    node_ptr_->get_parameter("global_ref_frame", earth_frame_id_);
    node_ptr_->get_parameter("odom_frame", odom_frame_id_);
    node_ptr_->get_parameter("map_frame", map_frame_id_);

    base_frame_id_ = as2::tf::generateTfName(node_ptr_, base_frame_id_);
    odom_frame_id_ = as2::tf::generateTfName(node_ptr_, odom_frame_id_);
    map_frame_id_  = as2::tf::generateTfName(node_ptr_, map_frame_id_);
    // !! WATCHOUT : earth_frame_id_ is not generated because it is a global frame

    on_setup();
  };
  virtual void on_setup() = 0;
  virtual bool get_earth_to_map_transform(geometry_msgs::msg::TransformStamped& transform) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "get_earth_to_map_transform not implemented using default identity transform");
    transform = as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    return true;
  };

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  void check_standard_transform(const geometry_msgs::msg::TransformStamped& transform) {
    if (transform.header.frame_id == get_earth_frame() &&
        transform.child_frame_id == get_map_frame()) {
      earth_to_map_ = tf2::Transform(
          tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y,
                          transform.transform.rotation.z, transform.transform.rotation.w),
          tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y,
                       transform.transform.translation.z));
    }
  }

protected:
  inline void publish_transform(const geometry_msgs::msg::TransformStamped& transform) {
    tf_broadcaster_->sendTransform(transform);
  }
  inline void publish_static_transform(const geometry_msgs::msg::TransformStamped& transform) {
    static_tf_broadcaster_->sendTransform(transform);
  }

  inline void publish_twist(const geometry_msgs::msg::TwistStamped& twist) {
    twist_pub_->publish(twist);
  }
  inline void publish_pose(const geometry_msgs::msg::PoseStamped& pose) {
    pose_pub_->publish(pose);
  }

  inline const std::string& get_earth_frame() const { return earth_frame_id_; }
  inline const std::string& get_map_frame() const { return map_frame_id_; }
  inline const std::string& get_odom_frame() const { return odom_frame_id_; }
  inline const std::string& get_base_frame() const { return base_frame_id_; }

  tf2::Transform odom_to_baselink;
  tf2::Transform earth_to_map;
  tf2::Transform earth_to_baselink;

  bool static_transforms_published_ = false;
  rclcpp::TimerBase::SharedPtr static_transforms_timer_;

  bool get_earth_to_map_transform(tf2::Transform& earth_to_map) {
    geometry_msgs::msg::TransformStamped transform;
    if (get_earth_to_map_transform(transform)) {
      tf2::fromMsg(transform.transform, earth_to_map);
      return true;
    }
    return false;
  }

  bool convert_earth_to_baselink_2_odom_to_baselink_transform(
      const tf2::Transform& earth_to_baselink,
      tf2::Transform& odom_to_baselink,
      const tf2::Transform& earth_to_map,
      const tf2::Transform& map_to_odom = tf2::Transform::getIdentity()) {
    odom_to_baselink = map_to_odom.inverse() * earth_to_map.inverse() * earth_to_baselink;
    return true;
  }

  bool convert_odom_to_baselink_2_earth_to_baselink_transform(
      const tf2::Transform& odom_to_baselink,
      tf2::Transform& earth_to_baselink,
      const tf2::Transform& earth_to_map,
      const tf2::Transform& map_to_odom = tf2::Transform::getIdentity()) {
    earth_to_baselink = earth_to_map * map_to_odom * odom_to_baselink;
    return true;
  }
};
}  // namespace as2_state_estimator_plugin_base

#endif
