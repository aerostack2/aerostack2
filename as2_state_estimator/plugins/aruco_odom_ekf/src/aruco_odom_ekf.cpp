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
* @file aruco_odom_ekf.cpp
*
* An state estimation plugin implementation based on an EKF that fuses ArUco markers and odometry
*
* @authors Pedro Arias Pérez
*/

#include "aruco_odom_ekf.hpp"
#include <pluginlib/class_list_macros.hpp>

void aruco_odom_ekf::Plugin::on_setup()
{
  // Parameter readings
  node_ptr_->get_parameter("use_gazebo_tf", using_gazebo_tf_);

  pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
    std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
  twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos,
    std::bind(&Plugin::twist_callback, this, std::placeholders::_1));

  // publish static transform from earth to map and map to odom
  earth_to_map_ =
    as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
  // TODO(javilinos): MODIFY this to a initial earth to map transform (reading initial position
  // from parameters or msgs )
  publish_static_transform(earth_to_map_);

  geometry_msgs::msg::TransformStamped map_to_odom =
    as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);

  if (using_gazebo_tf_) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Using gazebo tfs");
  }
  publish_static_transform(earth_to_map_);
  publish_static_transform(map_to_odom);
}

void aruco_odom_ekf::Plugin::generate_map_frame_from_ground_truth_pose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  earth_to_map_.transform.translation.x = pose.pose.position.x;
  earth_to_map_.transform.translation.y = pose.pose.position.y;
  earth_to_map_.transform.translation.z = pose.pose.position.z;
  earth_to_map_.transform.rotation.x = pose.pose.orientation.x;
  earth_to_map_.transform.rotation.y = pose.pose.orientation.y;
  earth_to_map_.transform.rotation.z = pose.pose.orientation.z;
  earth_to_map_.transform.rotation.w = pose.pose.orientation.w;
  publish_static_transform(earth_to_map_);
}

void aruco_odom_ekf::Plugin::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // since it's ground_truth we consider that the frame obtained is the world frame (earth)
  // so we need to publish the transform from world to base_link, with map and odom as the
  // identity transform in order to keep the continuity of the tf tree we will modify the
  // odom->base_link transform

  if (msg->header.frame_id != get_earth_frame()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Received pose in frame %s, expected %s",
      msg->header.frame_id.c_str(), get_earth_frame().c_str());
    return;
  }

  if (!earth_to_map_set_) {
    generate_map_frame_from_ground_truth_pose(*msg);
    earth_to_map_set_ = true;
  }

  earth_to_baselink.setOrigin(
    tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  earth_to_baselink.setRotation(
    tf2::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w));

  tf2::fromMsg(earth_to_map_.transform, earth_to_map);

  convert_earth_to_baselink_2_odom_to_baselink_transform(
    earth_to_baselink, odom_to_baselink,
    earth_to_map);

  auto odom_to_baselink_msg = geometry_msgs::msg::TransformStamped();
  odom_to_baselink_msg.header.stamp = msg->header.stamp;
  odom_to_baselink_msg.header.frame_id = get_odom_frame();
  if (using_gazebo_tf_) {
    odom_to_baselink_msg.child_frame_id = as2::tf::generateTfName("", node_ptr_->get_namespace());
  } else {
    odom_to_baselink_msg.child_frame_id = get_base_frame();
  }
  odom_to_baselink_msg.transform = tf2::toMsg(odom_to_baselink);

  publish_transform(odom_to_baselink_msg);

  // Pose should be published in the earth frame (world)
  // so we dont need to modify the frame_id
  publish_pose(*msg);
}

void aruco_odom_ekf::Plugin::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id != get_base_frame()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Received twist in frame %s, expected %s",
      msg->header.frame_id.c_str(), get_base_frame().c_str());
    // TODO(javilinos): convert it to the base_link frame if needed
    return;
  }
  publish_twist(*msg);
}

PLUGINLIB_EXPORT_CLASS(aruco_odom_ekf::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
