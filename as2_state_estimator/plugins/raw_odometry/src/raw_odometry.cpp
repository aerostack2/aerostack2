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
* @file raw_odometry.cpp
*
* An state estimation plugin raw odometry for AeroStack2 implementation
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include "raw_odometry/raw_odometry.hpp"

namespace raw_odometry
{

void Plugin::onSetup()
{
  // Define odometry topic
  std::string odom_sub_topic;
  odom_sub_topic = getParameter<std::string>(node_ptr_, "raw_odometry.odom_sub_topic");

  if (odom_sub_topic == "") {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "No odometry topic provided, raw odometry plugin will not work");
    return;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Using odometry topic: %s", odom_sub_topic.c_str());

  // Set earth to map from parameters
  set_earth_map_manually_ = getParameter<bool>(
    node_ptr_,
    "raw_odometry.earth_map_transform.set_earth_map");

  if (!set_earth_map_manually_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Setting earth to map transform to identity (0, 0, 0)");
    // Set to identity (already initialized as identity)
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map transform from parameters");
    double initial_x, initial_y, initial_z;
    initial_x = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.x");
    initial_y = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.y");
    initial_z = getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.position.z");
    double initial_roll, initial_pitch, initial_yaw;
    initial_roll = getParameter<double>(
      node_ptr_,
      "raw_odometry.earth_map_transform.orientation.roll");
    initial_pitch = getParameter<double>(
      node_ptr_,
      "raw_odometry.earth_map_transform.orientation.pitch");
    initial_yaw =
      getParameter<double>(node_ptr_, "raw_odometry.earth_map_transform.orientation.yaw");
    earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
    tf2::Quaternion q;
    q.setRPY(initial_roll, initial_pitch, initial_yaw);
    earth_to_map_.setRotation(q);
  }

  // Create subscription
  odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    odom_sub_topic, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::odometryCallback, this, std::placeholders::_1));
}

std::vector<as2_state_estimator::TransformInformatonType>
Plugin::getTransformationTypesAvailable() const
{
  return {as2_state_estimator::TransformInformatonType::EARTH_TO_MAP,
    as2_state_estimator::TransformInformatonType::MAP_TO_ODOM,
    as2_state_estimator::TransformInformatonType::ODOM_TO_BASE,
    as2_state_estimator::TransformInformatonType::TWIST_IN_BASE};
}

void Plugin::setupTfTree()
{
  // Set earth to map from parameters if not set with topic
  if (!earth_to_map_set_) {
    state_estimator_interface_->setEarthToMap(earth_to_map_, node_ptr_->now(), true);
    earth_to_map_set_ = true;
  }

  if (!map_to_odom_set_) {
    geometry_msgs::msg::PoseWithCovariance map_to_odom = generateIdentityPose();
    state_estimator_interface_->setMapToOdomPose(map_to_odom, node_ptr_->now(), true);
    map_to_odom_set_ = true;
  }
}

void Plugin::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Odometry should have frame_id = odom and child_frame_id = base_link
  // We publish the transform from odom to base_link directly
  // The transforms from earth to map and map to odom will be identity transforms

  if (msg->header.frame_id != state_estimator_interface_->getOdomFrame()) {
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(), "Received odometry in frame %s, expected %s. "
      "Frame id will be changed to expected one",
      msg->header.frame_id.c_str(), state_estimator_interface_->getOdomFrame().c_str());
  }

  if (msg->child_frame_id != state_estimator_interface_->getBaseFrame()) {
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(),
      "Received odometry child_frame_id in frame %s, expected %s. "
      "Child frame id will be changed to expected one",
      msg->child_frame_id.c_str(), state_estimator_interface_->getBaseFrame().c_str());
  }

  // Setup tf tree if not already setup
  setupTfTree();

  // Extract pose with covariance from odometry
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  pose_with_covariance.pose = msg->pose.pose;
  pose_with_covariance.covariance = msg->pose.covariance;

  // Publish odom to base_link pose
  state_estimator_interface_->setOdomToBaseLinkPose(pose_with_covariance, msg->header.stamp);

  // Extract twist with covariance from odometry
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = msg->twist.twist;
  twist_with_covariance.covariance = msg->twist.covariance;

  // Publish twist in base frame
  state_estimator_interface_->setTwistInBaseFrame(twist_with_covariance, msg->header.stamp);
}

}  // namespace raw_odometry

PLUGINLIB_EXPORT_CLASS(raw_odometry::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
