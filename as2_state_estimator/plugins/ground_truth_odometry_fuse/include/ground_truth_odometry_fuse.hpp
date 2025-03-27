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
* @file ground_truth_odometry_fuse.hpp
*
* An state estimation plugin external odom for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef GROUND_TRUTH_ODOMETRY_FUSE_HPP_
#define GROUND_TRUTH_ODOMETRY_FUSE_HPP_

#include <Eigen/Geometry>
#include <string>
#include <utility>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>

#include "as2_state_estimator/plugin_base.hpp"

namespace ground_truth_odometry_fuse
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  // Odometry and ground_truth sources
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_sub_;

  // Stored data
  geometry_msgs::msg::PoseStamped::SharedPtr ground_truth_ = nullptr;
  nav_msgs::msg::Odometry::SharedPtr odom_ = nullptr;

  // TF frames
  std::string rigid_body_name_ = "";
  geometry_msgs::msg::TransformStamped earth_to_map_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  geometry_msgs::msg::TransformStamped odom_to_baselink_;

  // Flags
  bool map_to_earth_set_ = false;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void on_setup() override
  {
    // Odometry
    std::string odom_topic = as2_names::topics::sensor_measurements::odom;
    node_ptr_->get_parameter("odom_topic", odom_topic);
    odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, as2_names::topics::sensor_measurements::qos,
      std::bind(&Plugin::odomCallback, this, std::placeholders::_1));

    // Ground truth
    std::string mocap_topic;
    node_ptr_->get_parameter("mocap_topic", mocap_topic);
    node_ptr_->get_parameter("rigid_body_name", rigid_body_name_);
    if (mocap_topic.empty() || rigid_body_name_.empty()) {
      std::string ground_truth_topic;
      node_ptr_->get_parameter("ground_truth_topic", ground_truth_topic);
      ground_truth_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        ground_truth_topic,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&Plugin::groundTruthCallback, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Using rigid bodies from topic %s and marker_id %s",
        mocap_topic.c_str(), rigid_body_name_.c_str());

      rigid_bodies_sub_ = node_ptr_->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        mocap_topic,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&Plugin::rigidBodiesCallback, this, std::placeholders::_1));
    }
  }

private:
  /**
   * @brief Computes the transformation from the map frame to the odometry frame using Eigen::Isometry3d.
   *
   * @param odom The odometry data.
   * @param ground_truth The ground truth pose data.
   * @return geometry_msgs::msg::TransformStamped The transformation from the map to odom frame.
  */
  geometry_msgs::msg::TransformStamped getTransform(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & ground_truth)
  {
    geometry_msgs::msg::TransformStamped map_to_odom;

    // Convert ground truth pose to Eigen Isometry3d
    Eigen::Isometry3d T_map = Eigen::Isometry3d::Identity();
    T_map.translation() = Eigen::Vector3d(
      ground_truth.pose.position.x,
      ground_truth.pose.position.y,
      ground_truth.pose.position.z);
    T_map.linear() = Eigen::Quaterniond(
      ground_truth.pose.orientation.w,
      ground_truth.pose.orientation.x,
      ground_truth.pose.orientation.y,
      ground_truth.pose.orientation.z)
      .toRotationMatrix();

    // Convert odometry pose to Eigen Isometry3d
    Eigen::Isometry3d T_odom = Eigen::Isometry3d::Identity();
    T_odom.translation() = Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
    T_odom.linear() = Eigen::Quaterniond(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z)
      .toRotationMatrix();

    // Compute the transformation from map to odom: T_map_to_odom = T_map * T_odom.inverse()
    Eigen::Isometry3d T_map_to_odom = T_map * T_odom.inverse();

    // Convert Eigen transformation to ROS TransformStamped
    map_to_odom.header.stamp = odom.header.stamp;
    map_to_odom.header.frame_id = get_map_frame();
    map_to_odom.child_frame_id = get_odom_frame();

    map_to_odom.transform.translation.x = T_map_to_odom.translation().x();
    map_to_odom.transform.translation.y = T_map_to_odom.translation().y();
    map_to_odom.transform.translation.z = T_map_to_odom.translation().z();

    Eigen::Quaterniond q_map_to_odom(T_map_to_odom.rotation());
    map_to_odom.transform.rotation.x = q_map_to_odom.x();
    map_to_odom.transform.rotation.y = q_map_to_odom.y();
    map_to_odom.transform.rotation.z = q_map_to_odom.z();
    map_to_odom.transform.rotation.w = q_map_to_odom.w();

    return map_to_odom;
  }


  // Callbacks

  /**
   * @brief Callback for the odometry topic
   *
   * @param msg Odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Convert to "odom frame"
    odom_ = msg;
    if (msg->header.frame_id != get_odom_frame()) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Odometry frame_id is %s, but expected %s.", msg->header.frame_id.c_str(),
        get_odom_frame().c_str());

      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position = msg->pose.pose.position;
      pose.pose.orientation = msg->pose.pose.orientation;
      if (!tf_handler_->tryConvert(pose, get_odom_frame())) {
        odom_ = nullptr;
        return;
      }
      odom_->pose.pose.position = pose.pose.position;
      odom_->pose.pose.orientation = pose.pose.orientation;
    }

    // Set transform from odom to base_link
    odom_to_baselink_.header.stamp = odom_->header.stamp;
    odom_to_baselink_.header.frame_id = get_odom_frame();
    odom_to_baselink_.child_frame_id = get_base_frame();
    odom_to_baselink_.transform.translation.x = odom_->pose.pose.position.x;
    odom_to_baselink_.transform.translation.y = odom_->pose.pose.position.y;
    odom_to_baselink_.transform.translation.z = odom_->pose.pose.position.z;
    odom_to_baselink_.transform.rotation = odom_->pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_to_baselink_);
    tf2::fromMsg(odom_to_baselink_.transform, odom_to_baselink);
    RCLCPP_INFO_ONCE(
      node_ptr_->get_logger(),
      "%s to %s transform working", get_odom_frame().c_str(), get_base_frame().c_str());

    // Check if ground truth is available
    if (ground_truth_ == nullptr) {
      return;
    }

    // Transform
    map_to_odom_ = getTransform(*odom_, *ground_truth_);
    tf_broadcaster_->sendTransform(map_to_odom_);
    RCLCPP_INFO_ONCE(
      node_ptr_->get_logger(),
      "%s to %s transform working", get_map_frame().c_str(), get_odom_frame().c_str());
    tf2::fromMsg(map_to_odom_.transform, map_to_odom);

    // Publish self localization
    convert_odom_to_baselink_2_earth_to_baselink_transform(
      odom_to_baselink, earth_to_baselink, earth_to_map, map_to_odom);

    // publish pose in "earth" frame
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = get_earth_frame();
    pose.header.stamp = node_ptr_->now();
    pose.pose.position.x = earth_to_baselink.getOrigin().x();
    pose.pose.position.y = earth_to_baselink.getOrigin().y();
    pose.pose.position.z = earth_to_baselink.getOrigin().z();
    pose.pose.orientation = tf2::toMsg(earth_to_baselink.getRotation());
    publish_pose(pose);

    // publish twist in "base_link" frame from odometry
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = get_base_frame();
    twist.header.stamp = node_ptr_->now();
    twist.twist = odom_->twist.twist;
    publish_twist(twist);
    return;
  }

  /**
   * @brief Callback for the ground truth topic
   *
   * @param msg PoseStamped message
   */
  void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != get_earth_frame()) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Ground truth frame_id is %s, but expected %s.", msg->header.frame_id.c_str(),
        get_earth_frame().c_str());
    }
    if (!map_to_earth_set_) {
      // At startup, map and odom are the same, and map to earth is set by the ground truth
      earth_to_map_.header.stamp = msg->header.stamp;
      earth_to_map_.header.frame_id = get_earth_frame();
      earth_to_map_.child_frame_id = get_map_frame();
      earth_to_map_.transform.translation.x = msg->pose.position.x;
      earth_to_map_.transform.translation.y = msg->pose.position.y;
      earth_to_map_.transform.translation.z = msg->pose.position.z;
      earth_to_map_.transform.rotation = msg->pose.orientation;
      publish_static_transform(earth_to_map_);
      tf2::fromMsg(earth_to_map_.transform, earth_to_map);
      map_to_earth_set_ = true;
      RCLCPP_INFO(
        node_ptr_->get_logger(), "%s to %s transform set to:", get_earth_frame().c_str(),
        get_map_frame().c_str());
      RCLCPP_INFO(
        node_ptr_->get_logger(), "  x: %f, y: %f, z: %f",
        earth_to_map_.transform.translation.x, earth_to_map_.transform.translation.y,
        earth_to_map_.transform.translation.z);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "  qx: %f, qy: %f, qz: %f, qw: %f",
        earth_to_map_.transform.rotation.x, earth_to_map_.transform.rotation.y,
        earth_to_map_.transform.rotation.z, earth_to_map_.transform.rotation.w);
      return;
    }
    geometry_msgs::msg::PoseStamped ground_truth = *msg;
    if (!tf_handler_->tryConvert(ground_truth, get_map_frame())) {
      // TODO(RPS98): Remove this warning
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(), "Ground truth not converted to map frame.");
    }
    ground_truth_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ground_truth);
    return;
  }

  /**
   * @brief Callback for the rigid bodies topic
   *
   * @param msg RigidBodies message
   */
  void rigidBodiesCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    // Convert rigid bodies to PoseStamped
    geometry_msgs::msg::PoseStamped::SharedPtr ground_truth =
      std::make_shared<geometry_msgs::msg::PoseStamped>();
    bool found = false;
    for (auto & body : msg->rigidbodies) {
      if (body.rigid_body_name == rigid_body_name_) {
        // TODO(RPS98): Now, stamp of msgs is not used
        ground_truth->header.stamp = node_ptr_->now();
        ground_truth->header.frame_id = get_earth_frame();
        ground_truth->pose = body.pose;
        found = true;
        break;
      }
    }

    if (!found) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Marker %s not found in rigid bodies message.", rigid_body_name_.c_str());
      return;
    }
    groundTruthCallback(ground_truth);
    return;
  }
};

}  // namespace ground_truth_odometry_fuse

#endif  // GROUND_TRUTH_ODOMETRY_FUSE_HPP_
