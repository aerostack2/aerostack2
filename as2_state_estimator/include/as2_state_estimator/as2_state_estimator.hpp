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
* @file as2_state_estimator.hpp
*
* An state estimation server for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_
#define AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <string>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>

#include "as2_state_estimator/plugin_base.hpp"
namespace as2_state_estimator
{

class StateEstimator : public as2::Node
{
public:
  explicit StateEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StateEstimator() {}

  bool filterTransformRule(const geometry_msgs::msg::TransformStamped & transform)
  {
    if (transform.header.frame_id == earth_frame_id_ && transform.child_frame_id == map_frame_id_) {
      return false;
    }
    if (transform.header.frame_id == map_frame_id_ && transform.child_frame_id == odom_frame_id_) {
      return false;
    }
    if (transform.header.frame_id == odom_frame_id_ && transform.child_frame_id == base_frame_id_) {
      return false;
    }
    return true;
  }

private:
  using StateEstimatorBase = as2_state_estimator_plugin_base::StateEstimatorBase;
  std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
  loader_;
  // std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase> plugin_ptr_;

  std::unordered_map<std::string, std::shared_ptr<StateEstimatorBase>> plugins_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  // std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::unordered_map<std::string,
    std::shared_ptr<StateEstimatorInterface>> state_estimator_interfaces_;
  std::unordered_map<std::string, std::shared_ptr<as2::tf::TfHandler>> tf_handlers_;

  void declareRosInterfaces()
  {
    // tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);
  }

  bool loadPlugin(const std::string & plugin_name);


  void readParameters()
  {
    // TODO(miferco97): Add try to get_declared_parameter for
    // avoid needing to always declare the standard parameters in the launch file

    // node_ptr_->declare_parameter<std::string>("base_frame", "base_link");
    // node_ptr_->declare_parameter<std::string>("global_ref_frame", "earth");
    // node_ptr_->declare_parameter<std::string>("odom_frame", "odom");
    // node_ptr_->declare_parameter<std::string>("map_frame", "map");

    this->get_parameter("base_frame", base_frame_id_);
    this->get_parameter("global_ref_frame", earth_frame_id_);
    this->get_parameter("odom_frame", odom_frame_id_);
    this->get_parameter("map_frame", map_frame_id_);

    base_frame_id_ = as2::tf::generateTfName(this, base_frame_id_);
    odom_frame_id_ = as2::tf::generateTfName(this, odom_frame_id_);
    map_frame_id_ = as2::tf::generateTfName(this, map_frame_id_);

    RCLCPP_INFO(this->get_logger(), "Frame names:");
    RCLCPP_INFO(this->get_logger(), "\tEarth frame: %s", earth_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tMap frame: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tOdom frame: %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "\tBase frame: %s", base_frame_id_.c_str());
  }

  friend class MetacontrollerInterface;

private:
  std::string earth_frame_id_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;

  void processEarthToMap(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);
  void processMapToOdom(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);
  void processOdomToBase(
    const std::string & authority,
    const geometry_msgs::msg::PoseWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp);
  void processTwist(
    const std::string & authority,
    const geometry_msgs::msg::TwistWithCovariance & msg,
    const builtin_interfaces::msg::Time & stamp);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);

  void publishTransform(
    const geometry_msgs::msg::PoseWithCovariance & pose, const std::string & parent_frame,
    const std::string & child_frame, const builtin_interfaces::msg::Time & stamp,
    bool is_static = false);


  void publish_initial_transforms();

  void publishTwist(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = stamp;
    twist_msg.header.frame_id = base_frame_id_;
    twist_msg.twist = twist.twist;
    twist_pub_->publish(twist_msg);
  }

  void publishPoseInEarthFrame(
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    tf2::Transform earth_to_base = earth_to_map_ * map_to_odom_ * odom_to_base_;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = earth_frame_id_;
    auto pose = tf2::toMsg(earth_to_base);
    pose_msg.pose.position.x = pose.translation.x;
    pose_msg.pose.position.y = pose.translation.y;
    pose_msg.pose.position.z = pose.translation.z;
    pose_msg.pose.orientation = pose.rotation;
    pose_pub_->publish(pose_msg);
  }


  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  /**
   * @brief Modify the node options to allow undeclared parameters
   */
  static rclcpp::NodeOptions get_modified_options(const rclcpp::NodeOptions & options);
};  // class StateEstimator
}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__AS2_STATE_ESTIMATOR_HPP_
