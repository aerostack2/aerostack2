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
* @file ground_truth.hpp
*
* An state estimation plugin ground truth for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef GROUND_TRUTH_HPP_
#define GROUND_TRUTH_HPP_

#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/utils/conversions.hpp"
#include "as2_core/names/topics.hpp"


namespace ground_truth
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;


  // bool using_gazebo_tf_ = false;
  bool set_earth_map_manually_ = false;
  bool earth_to_map_set_ = false;
  bool map_to_odom_set_ = false;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform earth_to_baselink_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_baselink_ = tf2::Transform::getIdentity();

public:
  Plugin() : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void onSetup() override
  {
    std::string pose_sub_topic;
    pose_sub_topic = getParameter<std::string>(node_ptr_, "ground_truth.pose_sub_topic");
    std::string twist_sub_topic;
    twist_sub_topic = getParameter<std::string>(node_ptr_, "ground_truth.twist_sub_topic");

    pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_sub_topic, as2_names::topics::ground_truth::qos,
      std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
    twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_sub_topic, as2_names::topics::ground_truth::qos,
      std::bind(&Plugin::twist_callback, this, std::placeholders::_1));

    set_earth_map_manually_ = getParameter<bool>(node_ptr_, "ground_truth.earth_map_transform.set_earth_map");
    if (!set_earth_map_manually_) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Setting origin on start with the first received pose in the topic");
    } else {
      RCLCPP_INFO(node_ptr_->get_logger(), "Not setting map origin with fixed pose");
      double initial_x, initial_y, initial_z;
      initial_x = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.x");
      initial_y = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.y");
      initial_z = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.z");
      double initial_roll, initial_pitch, initial_yaw;
      initial_roll = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.orientation.roll");
      initial_pitch = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.orientation.pitch");
      initial_yaw = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.orientation.yaw");
      earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
      tf2::Quaternion q;
      q.setRPY(initial_roll, initial_pitch, initial_yaw);
      earth_to_map_.setRotation(q);
    }
  }

  std::vector<as2_state_estimator::TransformInformatonType> getTransformationTypesAvailable() const override
  {
    return {as2_state_estimator::TransformInformatonType::EARTH_TO_MAP,
      as2_state_estimator::TransformInformatonType::MAP_TO_ODOM,
      as2_state_estimator::TransformInformatonType::ODOM_TO_BASE,
      as2_state_estimator::TransformInformatonType::TWIST_IN_BASE};
  }

private:
  // TODO: Do a method to setup tf tree
  void setupTfTree()
  {
    // Set earth to map from parameters if not set with topic
    if (!earth_to_map_set_) {
      state_estimator_interface_->setEarthToMap(earth_to_map_, node_ptr_->now(), true);
      earth_to_map_set_ = true;
    }

    if (!map_to_odom_set_) {
      geometry_msgs::msg::PoseWithCovariance map_to_odom = generate_initial_odom_frame();
      state_estimator_interface_->setMapToOdomPose(map_to_odom, node_ptr_->now(), true);
      map_to_odom_set_ = true;
    }
  }

  geometry_msgs::msg::PoseWithCovariance generate_initial_odom_frame()
  {
    // identity transform
    geometry_msgs::msg::PoseWithCovariance map_to_odom;
    map_to_odom.pose.position.x = 0.0;
    map_to_odom.pose.position.y = 0.0;
    map_to_odom.pose.position.z = 0.0;
    map_to_odom.pose.orientation.x = 0.0;
    map_to_odom.pose.orientation.y = 0.0;
    map_to_odom.pose.orientation.z = 0.0;
    map_to_odom.pose.orientation.w = 1.0;
    return map_to_odom;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // since it's ground_truth we consider that the frame obtained is the world frame (earth)
    // so we need to publish the transform from world to base_link, with map and odom as the
    // identity transform in order to keep the continuity of the tf tree we will modify the
    // odom->base_link transform

    if (msg->header.frame_id != state_estimator_interface_->getEarthFrame()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Received pose in frame %s, expected %s",
        msg->header.frame_id.c_str(), state_estimator_interface_->getEarthFrame().c_str());
      return;
    }

    if (!set_earth_map_manually_ && !earth_to_map_set_) {
        RCLCPP_INFO_ONCE(
          node_ptr_->get_logger(),
          "Setting map origin with the first received pose in the topic. Ignoring the rest of poses for setting the map origin");
        
      // Earth to map from pose
      earth_to_map_.setOrigin(
        tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
      earth_to_map_.setRotation(
        tf2::Quaternion(
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w));
    }

    // Setup tf tree if not already setup
    setupTfTree();

    earth_to_baselink_.setOrigin(
      tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    earth_to_baselink_.setRotation(
      tf2::Quaternion(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w));

    as2_state_estimator::conversions::convert_earth_to_baselink_2_odom_to_baselink_transform(
      earth_to_baselink_, odom_to_baselink_,
      state_estimator_interface_->getEarthToMapTransform(),
      state_estimator_interface_->getMapToOdomTransform());

    state_estimator_interface_->setOdomToBaseLinkPose(
      odom_to_baselink_,
      msg->header.stamp);
  }

  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != state_estimator_interface_->getBaseFrame()) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Received twist in frame %s, expected %s. "
        "Changed to expected one",
        msg->header.frame_id.c_str(), state_estimator_interface_->getBaseFrame().c_str());
    }
    static geometry_msgs::msg::TwistWithCovariance twist_with_covariance_msg_;
    twist_with_covariance_msg_.twist = msg->twist;
    state_estimator_interface_->setTwistInBaseFrame(twist_with_covariance_msg_, msg->header.stamp);
  }
};      // class GroundTruth
}       // namespace ground_truth
#endif  // GROUND_TRUTH_HPP_
