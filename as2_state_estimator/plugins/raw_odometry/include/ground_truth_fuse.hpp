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
* @file ground_truth_fuse.hpp
*
* Ground truth fusion with raw odometry plugin for Aerostack2
*
* @authors Rafael Pérez Seguí
*/

#ifndef GROUND_TRUTH_FUSE_HPP_
#define GROUND_TRUTH_FUSE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <as2_core/core_functions.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/names/topics.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>

namespace as2_state_estimator
{

class GroundTruthFuse : public as2::Node
{
public:
  explicit GroundTruthFuse(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : as2::Node("state_estimator", options)
  {
    // TF
    tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    std::string map_frame = this->declare_parameter("map_frame", "map");
    map_frame_ = as2::tf::generateTfName(this, map_frame);
    std::string odom_frame = this->declare_parameter("odom_frame", "odom");
    odom_frame_ = as2::tf::generateTfName(this, odom_frame);

    // Odometry and ground_truth sources
    std::string odom_topic = this->declare_parameter(
      "odom_topic",
      as2_names::topics::sensor_measurements::odom);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, as2_names::topics::sensor_measurements::qos,
      std::bind(&GroundTruthFuse::odomCallback, this, std::placeholders::_1));

    std::string rigid_bodies_topic = this->declare_parameter(
      "rigid_bodies_topic", "");
    marker_id_ = this->declare_parameter("rigid_body_name", "");
    if (rigid_bodies_topic.empty() || marker_id_.empty()) {
      std::string ground_truth_topic = this->declare_parameter(
        "ground_truth_topic",
        as2_names::topics::ground_truth::pose);
      ground_truth_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ground_truth_topic,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&GroundTruthFuse::groundTruthCallback, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(
        this->get_logger(), "Using rigid bodies from topic %s and marker_id %s",
        rigid_bodies_topic.c_str(), marker_id_.c_str());

      rigid_bodies_sub_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        rigid_bodies_topic,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&GroundTruthFuse::rigidBodiesCallback, this, std::placeholders::_1));
    }

    // Timer
    double frequency = this->declare_parameter("ground_truth_fuse_frequency", 50.0);
    timer_ = this->create_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / frequency)),
      std::bind(&GroundTruthFuse::timerCallback, this));
  }

  ~GroundTruthFuse() {}

private:
  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Odometry and ground_truth sources
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_sub_;

  // Stored data
  nav_msgs::msg::Odometry::SharedPtr odom_ = nullptr;
  geometry_msgs::msg::PoseStamped::SharedPtr ground_truth_ = nullptr;

  std::string odom_frame_;
  std::string map_frame_;
  std::string marker_id_ = "";
  mocap4r2_msgs::msg::RigidBodies::SharedPtr rigid_bodies_ = nullptr;

private:
  /**
   * @brief Modify the node options to allow undeclared parameters
   */
  static rclcpp::NodeOptions getModifiedOptions(const rclcpp::NodeOptions & options)
  {
    // Create a copy of the options and modify it
    rclcpp::NodeOptions modified_options = options;
    modified_options.allow_undeclared_parameters(true);
    modified_options.automatically_declare_parameters_from_overrides(true);
    return modified_options;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
    return;
  }

  void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ground_truth_ = msg;
    return;
  }

  void rigidBodiesCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    rigid_bodies_ = msg;
    return;
  }

  void timerCallback()
  {
    // Initialization checks
    if (odom_ == nullptr) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for odometry");
      return;
    }
    if (rigid_bodies_ != nullptr) {
      // Convert to PoseStamped
      rigidBodiesToPoseStamped();
    }

    if (ground_truth_ == nullptr) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for ground truth");
      return;
    } else {
      if (!processGroundTruth()) {
        return;
      }
    }
    if (!processOdometry()) {
      return;
    }

    // Process
    double x_diff = ground_truth_->pose.position.x - odom_->pose.pose.position.x;
    double y_diff = ground_truth_->pose.position.y - odom_->pose.pose.position.y;
    double z_diff = ground_truth_->pose.position.z - odom_->pose.pose.position.z;

    double gt_yaw = as2::frame::getYawFromQuaternion(ground_truth_->pose.orientation);
    double odom_yaw = as2::frame::getYawFromQuaternion(odom_->pose.pose.orientation);
    double yaw_diff = as2::frame::angleMinError(gt_yaw, odom_yaw);

    // Transform
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = x_diff;
    map_to_odom.transform.translation.y = y_diff;
    map_to_odom.transform.translation.z = z_diff;
    as2::frame::eulerToQuaternion(0, 0, yaw_diff, map_to_odom.transform.rotation);

    RCLCPP_DEBUG(
      this->get_logger(), "Ground truth to odometry transform: x=%f, y=%f, z=%f, yaw=%f",
      x_diff, y_diff, z_diff, yaw_diff);

    tf_broadcaster_->sendTransform(map_to_odom);

    // Reset data
    odom_ = nullptr;
    ground_truth_ = nullptr;
    rigid_bodies_ = nullptr;
  }

  void rigidBodiesToPoseStamped()
  {
    // Convert rigid bodies to PoseStamped
    ground_truth_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    for (auto & body : rigid_bodies_->rigidbodies) {
      if (body.rigid_body_name == marker_id_) {
        // TODO(RPS98): Now, stamp of msgs is not used
        ground_truth_->header.stamp = this->now();
        ground_truth_->header.frame_id = "earth";
        ground_truth_->pose = body.pose;
        return;
      }
    }
  }

  bool processGroundTruth()
  {
    return tf_handler_->tryConvert(*ground_truth_, map_frame_);
  }

  bool processOdometry()
  {
    // Convert to "map frame"
    if (odom_->header.frame_id != odom_frame_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Odometry frame_id is %s, but expected %s.", odom_->header.frame_id.c_str(),
        odom_frame_.c_str());

      geometry_msgs::msg::PoseStamped pose;
      pose.header = odom_->header;
      pose.pose.position = odom_->pose.pose.position;
      pose.pose.orientation = odom_->pose.pose.orientation;
      if (!tf_handler_->tryConvert(pose, odom_frame_)) {
        return false;
      }
      odom_->pose.pose.position = pose.pose.position;
      odom_->pose.pose.orientation = pose.pose.orientation;
      odom_->header.frame_id = odom_frame_;
    }
    return true;
  }
};  // class GroundTruthFuse
}  // namespace as2_state_estimator

#endif  // GROUND_TRUTH_FUSE_HPP_
