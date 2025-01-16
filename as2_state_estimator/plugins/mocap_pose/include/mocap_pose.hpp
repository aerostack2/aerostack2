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
* @file mocap_pose.hpp
*
* An state estimation plugin mocap_pose for AeroStack2
*
* @authors Miguel Fernández Cortizas
*          Pedro Arias Pérez
*          Javier Melero Deza
*          Rafael Pérez Seguí
*          David Pérez Saura
*/

#ifndef MOCAP_POSE_HPP_
#define MOCAP_POSE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>
#include <vector>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <rclcpp/duration.hpp>

#include "as2_state_estimator/plugin_base.hpp"

namespace mocap_pose
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_sub_;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();

  bool has_earth_to_map_ = false;
  bool smooth_orientation_ = true;
  std::string mocap_topic_;
  std::string rigid_body_name_;
  double twist_alpha_ = 1.0;
  double orientation_alpha_ = 1.0;
  geometry_msgs::msg::TwistWithCovariance twist_with_covariance_msg_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void onSetup() override
  {
    node_ptr_->get_parameter("mocap_topic", mocap_topic_);
    if (mocap_topic_.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Parameter 'mocap_topic' not set");
      throw std::runtime_error("Parameter 'mocap_topic' not set");
    }
    node_ptr_->get_parameter("rigid_body_name", rigid_body_name_);
    if (rigid_body_name_.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Parameter 'rigid_body_name' not set");
      throw std::runtime_error("Parameter 'rigid_body_name' not set");
    }

    try {
      twist_alpha_ = node_ptr_->get_parameter("twist_smooth_filter_cte").as_double();
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Parameter 'twist_smooth_filter_cte' not set. Filter disabled. Using default value: %f",
        twist_alpha_);
    }

    try {
      orientation_alpha_ = node_ptr_->get_parameter("orientation_smooth_filter_cte").as_double();
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Parameter 'orientation_smooth_filter_cte' not set. Filter disabled. Using "
        "default value: %f",
        orientation_alpha_);
    }

    rigid_bodies_sub_ = node_ptr_->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      mocap_topic_, rclcpp::QoS(10),
      std::bind(&Plugin::rigid_bodies_callback, this, std::placeholders::_1));

    // This transform is static and should be published only once
    state_estimator_interface_->setMapToOdomPose(
      tf2::Transform::getIdentity(),
      node_ptr_->now(), true);
  }

  const geometry_msgs::msg::TwistWithCovariance & twist_from_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    std::vector<tf2::Transform> * data = nullptr)
  {
    // Compute twist by differentiating the pose over time and filtering it with a low pass filter

    // last_pose is static to keep the last pose between calls
    // last_twist is static to keep the last twist between calls
    // dt is static to keep the last dt between calls
    static tf2::Vector3 last_pose(pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z);

    static tf2::Vector3 last_vel(0, 0, 0);
    static auto last_time = pose.header.stamp;

    auto dt = (rclcpp::Time(pose.header.stamp) - last_time).seconds();
    if (dt <= 0) {
      RCLCPP_WARN(node_ptr_->get_logger(), "dt <= 0");
      return twist_with_covariance_msg_;
    }

    tf2::Vector3 current_pose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    tf2::Vector3 vel_in_earth_frame = (current_pose - last_pose) / dt;
    last_pose = current_pose;

    // TODO(javilinos): check if this is correct
    vel_in_earth_frame = tf2::quatRotate(
      (odom_to_base_.inverse() * map_to_odom_.inverse() * earth_to_map_.inverse()).getRotation(),
      vel_in_earth_frame);

    tf2::Vector3 vel_in_base_frame = twist_alpha_ * vel_in_earth_frame + (1 - twist_alpha_) *
      last_vel;
    last_vel = vel_in_base_frame;

    twist_with_covariance_msg_.twist.linear.x = vel_in_base_frame.x();
    twist_with_covariance_msg_.twist.linear.y = vel_in_base_frame.y();
    twist_with_covariance_msg_.twist.linear.z = vel_in_base_frame.z();
    // TODO(javilinos): add angular velocity -> this_could_be_obtained_from_imu
    twist_with_covariance_msg_.twist.angular.x = 0;
    twist_with_covariance_msg_.twist.angular.y = 0;
    twist_with_covariance_msg_.twist.angular.z = 0;
    // TODO(dps): add covariance

    return twist_with_covariance_msg_;
  }

private:
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = msg->header;
    for (const auto & rigid_body : msg->rigidbodies) {
      if (rigid_body.rigid_body_name == rigid_body_name_) {
        pose_msg.pose = rigid_body.pose;
        break;
      }
    }
    process_mocap_pose(pose_msg);
  }

  void process_mocap_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    // mocap_pose could have a different frame_id, we will publish the transform from earth to
    // base_link without checking origin frame_id

    if (!has_earth_to_map_) {
      earth_to_map_ = tf2::Transform(
        tf2::Quaternion(
          msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
          msg.pose.orientation.w),
        tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
      // This transform is static and should be published only once
      state_estimator_interface_->setEarthToMap(earth_to_map_, msg.header.stamp, true);
      has_earth_to_map_ = true;
    }
    odom_to_base_ =
      map_to_odom_.inverse() * earth_to_map_.inverse() *
      tf2::Transform(
      tf2::Quaternion(
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w),
      tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    // Publish pose
    geometry_msgs::msg::PoseWithCovariance pose_msg;

    pose_msg.pose = msg.pose;
    // TODO(dps): add covariance

    if (smooth_orientation_) {
      // TODO(dps):  check this orientation smoothing
      static auto last_pose_msg = pose_msg;
      pose_msg.pose.orientation.x = orientation_alpha_ * msg.pose.orientation.x +
        (1 - orientation_alpha_) * last_pose_msg.pose.orientation.x;
      pose_msg.pose.orientation.y = orientation_alpha_ * msg.pose.orientation.y +
        (1 - orientation_alpha_) * last_pose_msg.pose.orientation.y;
      pose_msg.pose.orientation.z = orientation_alpha_ * msg.pose.orientation.z +
        (1 - orientation_alpha_) * last_pose_msg.pose.orientation.z;
      pose_msg.pose.orientation.w = orientation_alpha_ * msg.pose.orientation.w +
        (1 - orientation_alpha_) * last_pose_msg.pose.orientation.w;
      last_pose_msg = pose_msg;
    }
    state_estimator_interface_->setOdomToBaseLinkPose(pose_msg, msg.header.stamp);
    state_estimator_interface_->setTwistInLocalFrame(
      twist_from_pose(msg), msg.header.stamp);
  }
};  // class Plugin

}  // namespace mocap_pose

#endif  // MOCAP_POSE_HPP_
