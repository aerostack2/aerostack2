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
* @file ground_truth.cpp
*
* An state estimation plugin ground truth for AeroStack2 implementation
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
#include "ground_truth/ground_truth.hpp"

namespace ground_truth
{

void Plugin::onSetup()
{
  // Define pose topic
  std::string pose_sub_topic;
  pose_sub_topic = getParameter<std::string>(node_ptr_, "ground_truth.pose_sub_topic");

  std::string mocap_sub_topic;
  mocap_sub_topic = getParameter<std::string>(node_ptr_, "ground_truth.mocap_sub_topic");

  if (mocap_sub_topic != "") {
    RCLCPP_INFO(node_ptr_->get_logger(), "Using motion capture topic for ground truth");
    // Try to get rigid_body_name as string, if it fails try as integer and convert
    try {
      rigid_body_name_ = getParameter<std::string>(node_ptr_, "ground_truth.rigid_body_name");
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      // Parameter might be an integer, try to get it and convert to string
      int rigid_body_id = getParameter<int>(node_ptr_, "ground_truth.rigid_body_name");
      rigid_body_name_ = std::to_string(rigid_body_id);
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "rigid_body_name parameter was an integer (%d), converted to string '%s'. "
        "Consider using quotes in your YAML: rigid_body_name: \"%d\"",
        rigid_body_id, rigid_body_name_.c_str(), rigid_body_id);
    }
  }

  if (pose_sub_topic == "" && mocap_sub_topic == "") {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "No pose topic provided, ground truth plugin will not work");
    return;
  }

  // Define twist topic or set to integrate pose for twist estimation if not provided
  std::string twist_sub_topic;
  twist_sub_topic = getParameter<std::string>(node_ptr_, "ground_truth.twist_sub_topic");

  if (twist_sub_topic == "") {
    integrate_pose_for_twist_ = true;
    RCLCPP_INFO(
      node_ptr_->get_logger(), "No twist topic provided, integrating pose for twist estimation");
    twist_smooth_filter_cte_ = getParameter<double>(
      node_ptr_,
      "ground_truth.twist_smooth_filter_cte");
  }

  // Set earth to map from parameters if not set with first topic message
  set_earth_map_manually_ = getParameter<bool>(
    node_ptr_,
    "ground_truth.earth_map_transform.set_earth_map");
  if (!set_earth_map_manually_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Setting origin on start with the first received pose in the topic");
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Not setting map origin with fixed pose");
    double initial_x, initial_y, initial_z;
    initial_x = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.x");
    initial_y = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.y");
    initial_z = getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.position.z");
    double initial_roll, initial_pitch, initial_yaw;
    initial_roll = getParameter<double>(
      node_ptr_,
      "ground_truth.earth_map_transform.orientation.roll");
    initial_pitch = getParameter<double>(
      node_ptr_,
      "ground_truth.earth_map_transform.orientation.pitch");
    initial_yaw =
      getParameter<double>(node_ptr_, "ground_truth.earth_map_transform.orientation.yaw");
    earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
    tf2::Quaternion q;
    q.setRPY(initial_roll, initial_pitch, initial_yaw);
    earth_to_map_.setRotation(q);
  }

  // Create subscriptions
  if (mocap_sub_topic != "") {
    mocap_sub_ = node_ptr_->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      mocap_sub_topic, as2_names::topics::ground_truth::qos,
      std::bind(&Plugin::mocapCallback, this, std::placeholders::_1));
  } else {
    pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_sub_topic, as2_names::topics::ground_truth::qos,
      std::bind(&Plugin::poseCallback, this, std::placeholders::_1));
  }

  if (!integrate_pose_for_twist_) {
    twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_sub_topic, as2_names::topics::ground_truth::qos,
      std::bind(&Plugin::twistCallback, this, std::placeholders::_1));
  }
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

const geometry_msgs::msg::TwistWithCovariance & Plugin::computeTwistFromPose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Compute twist by differentiating the pose over time and filtering it with a low pass filter

  if (!first_pose_received_) {
    // Initialize on first call
    last_pose_ = tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    last_time_ = pose.header.stamp;
    first_pose_received_ = true;
    return twist_with_covariance_msg_;
  }

  auto dt = (rclcpp::Time(pose.header.stamp) - last_time_).seconds();
  if (dt <= 1e-6) {
    RCLCPP_WARN(node_ptr_->get_logger(), "dt <= 0, skipping twist computation");
    return twist_with_covariance_msg_;
  }

  tf2::Vector3 current_pose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  tf2::Vector3 vel_in_earth_frame = (current_pose - last_pose_) / dt;

  // Transform velocity from earth frame to base frame using current orientation
  tf2::Quaternion current_orientation(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);

  // Rotate velocity from earth frame to base frame (inverse rotation)
  tf2::Vector3 vel_transformed = tf2::quatRotate(current_orientation.inverse(), vel_in_earth_frame);

  // Apply low-pass filter
  tf2::Vector3 vel_in_base_frame = twist_smooth_filter_cte_ * vel_transformed +
    (1 - twist_smooth_filter_cte_) * last_vel_;

  // Update state for next iteration
  last_pose_ = current_pose;
  last_vel_ = vel_in_base_frame;
  last_time_ = pose.header.stamp;

  twist_with_covariance_msg_.twist.linear.x = vel_in_base_frame.x();
  twist_with_covariance_msg_.twist.linear.y = vel_in_base_frame.y();
  twist_with_covariance_msg_.twist.linear.z = vel_in_base_frame.z();
  // TODO(rdasilva01): add angular velocity -> this could be obtained from imu
  twist_with_covariance_msg_.twist.angular.x = 0;
  twist_with_covariance_msg_.twist.angular.y = 0;
  twist_with_covariance_msg_.twist.angular.z = 0;
  // TODO(rdasilva01): add covariance

  return twist_with_covariance_msg_;
}

bool Plugin::isSamePose(
  const tf2::Vector3 & pose1, const tf2::Vector3 & pose2,
  double position_threshold)
{
  return (pose1 - pose2).length() < position_threshold;
}

void Plugin::processPose(const geometry_msgs::msg::PoseStamped & msg)
{
  // since it's ground_truth we consider that the frame obtained is the world frame (earth)
  // so we need to publish the transform from world to base_link, with map and odom as the
  // identity transform in order to keep the continuity of the tf tree we will modify the
  // odom->base_link transform

  if (msg.header.frame_id != state_estimator_interface_->getEarthFrame()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Received pose in frame %s, expected %s",
      msg.header.frame_id.c_str(), state_estimator_interface_->getEarthFrame().c_str());
    return;
  }

  if (!set_earth_map_manually_ && !earth_to_map_set_) {
    RCLCPP_INFO_ONCE(
      node_ptr_->get_logger(),
      "Setting map origin with the first received pose in the topic. "
      "Ignoring the rest of poses for setting the map origin");

    // Earth to map from pose
    earth_to_map_.setOrigin(
      tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
    earth_to_map_.setRotation(
      tf2::Quaternion(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w));
  }

  // Setup tf tree if not already setup
  setupTfTree();

  earth_to_baselink_.setOrigin(
    tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
  earth_to_baselink_.setRotation(
    tf2::Quaternion(
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z,
      msg.pose.orientation.w));

  as2_state_estimator::conversions::convert_earth_to_baselink_2_odom_to_baselink_transform(
    earth_to_baselink_, odom_to_baselink_,
    state_estimator_interface_->getEarthToMapTransform(),
    state_estimator_interface_->getMapToOdomTransform());

  state_estimator_interface_->setOdomToBaseLinkPose(
    odom_to_baselink_,
    msg.header.stamp);

  // Compute and publish twist if integration is enabled
  if (integrate_pose_for_twist_) {
    state_estimator_interface_->setTwistInBaseFrame(
      computeTwistFromPose(msg),
      msg.header.stamp);
  }
}

void Plugin::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  tf2::Vector3 current_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  if (isSamePose(current_pose, last_pose_)) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Received the same pose as the last one, skipping it to avoid computing the same twist");
    return;
  }
  processPose(*msg);
}

void Plugin::mocapCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  for (const auto & rigid_body : msg->rigidbodies) {
    if (rigid_body.rigid_body_name == rigid_body_name_) {
      // Check if all pose is 0.0, if it is,
      // skip it since it means that the rigid body is not detected
      if (rigid_body.pose.position.x == 0.0 && rigid_body.pose.position.y == 0.0 &&
        rigid_body.pose.position.z == 0.0 &&
        rigid_body.pose.orientation.x == 0.0 && rigid_body.pose.orientation.y == 0.0 &&
        rigid_body.pose.orientation.z == 0.0)
      {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Rigid body %s has all pose values equal to 0, skipping it since it means that the "
          "rigid body is not detected",
          rigid_body_name_.c_str());
      } else {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = state_estimator_interface_->getEarthFrame();
        // Override frame to earth frame
        pose_msg.pose = rigid_body.pose;
        tf2::Vector3 current_pose(pose_msg.pose.position.x, pose_msg.pose.position.y,
          pose_msg.pose.position.z);
        if (isSamePose(current_pose, last_pose_)) {
          RCLCPP_WARN(
            node_ptr_->get_logger(),
            "Received the same pose as the last one, ",
            "skipping it to avoid computing the same twist");
          return;
        }
        processPose(pose_msg);
        return;
      }
    }
  }
  RCLCPP_WARN(
    node_ptr_->get_logger(), "Rigid body %s not found in mocap message",
    rigid_body_name_.c_str());
  RCLCPP_WARN(
    node_ptr_->get_logger(), "Available rigid bodies in the message are:");
  for (const auto & rigid_body : msg->rigidbodies) {
    RCLCPP_WARN(node_ptr_->get_logger(), "- %s", rigid_body.rigid_body_name.c_str());
  }
}

void Plugin::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
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

}  // namespace ground_truth

PLUGINLIB_EXPORT_CLASS(ground_truth::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
