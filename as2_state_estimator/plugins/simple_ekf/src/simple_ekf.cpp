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
* @file simple_ekf.cpp
*
* An state estimation plugin simple_ekf for AeroStack2 implementation
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
#include "simple_ekf/simple_ekf.hpp"

namespace simple_ekf
{

void Plugin::onSetup()
{
  // Verbose logging for debugging purposes
  verbose_ = getParameter<bool>(node_ptr_, "simple_ekf.verbose");

  // Set earth to map from parameters if not set with first topic message
  set_earth_map_manually_ = getParameter<bool>(
    node_ptr_,
    "simple_ekf.earth_map_transform.set_earth_map");
  if (!set_earth_map_manually_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Setting origin on start with the first received pose in the topic");
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Not setting map origin with fixed pose");
    double initial_x, initial_y, initial_z;
    initial_x = getParameter<double>(node_ptr_, "simple_ekf.earth_map_transform.position.x");
    initial_y = getParameter<double>(node_ptr_, "simple_ekf.earth_map_transform.position.y");
    initial_z = getParameter<double>(node_ptr_, "simple_ekf.earth_map_transform.position.z");
    double initial_roll, initial_pitch, initial_yaw;
    initial_roll = getParameter<double>(
      node_ptr_,
      "simple_ekf.earth_map_transform.orientation.roll");
    initial_pitch = getParameter<double>(
      node_ptr_,
      "simple_ekf.earth_map_transform.orientation.pitch");
    initial_yaw =
      getParameter<double>(node_ptr_, "simple_ekf.earth_map_transform.orientation.yaw");
    earth_to_map_.setOrigin(tf2::Vector3(initial_x, initial_y, initial_z));
    tf2::Quaternion q;
    q.setRPY(initial_roll, initial_pitch, initial_yaw);
    earth_to_map_.setRotation(q);
  }

  // Read prediction topic
  std::string predict_topic = getParameter<std::string>(node_ptr_, "simple_ekf.predict_topic");

  // Read update topics
  std::vector<std::string> topic_ids;
  node_ptr_->get_parameter("simple_ekf.update_topics", topic_ids);

  RCLCPP_INFO(node_ptr_->get_logger(), "Configuring %zu pose topic(s):", topic_ids.size());
  for (const auto & id : topic_ids) {
    RCLCPP_INFO(node_ptr_->get_logger(), "  - %s", id.c_str());
  }

  for (const auto & topic_id : topic_ids) {
    PoseTopicConfig config;
    std::string prefix = "simple_ekf." + topic_id;

    config.topic = getParameter<std::string>(node_ptr_, prefix + ".topic");
    config.type = getParameter<std::string>(node_ptr_, prefix + ".type");
    config.set_earth_map = getParameter<bool>(node_ptr_, prefix + ".set_earth_map");
    config.use_message_covariance = getParameter<bool>(
      node_ptr_, prefix + ".use_message_covariance");

    if (config.use_message_covariance) {
      std::vector<double> pos_mult;
      std::vector<double> ori_mult;
      node_ptr_->get_parameter(prefix + ".position_multiplier", pos_mult);
      node_ptr_->get_parameter(prefix + ".orientation_multiplier", ori_mult);
      std::copy_n(pos_mult.begin(), 3, config.position_values.begin());
      std::copy_n(ori_mult.begin(), 3, config.orientation_values.begin());

      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] position_multiplier: [%.3f, %.3f, %.3f]",
        topic_id.c_str(), pos_mult[0], pos_mult[1], pos_mult[2]);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] orientation_multiplier: [%.3f, %.3f, %.3f]",
        topic_id.c_str(), ori_mult[0], ori_mult[1], ori_mult[2]);
    } else {
      std::vector<double> pos_cov;
      std::vector<double> ori_cov;
      node_ptr_->get_parameter(prefix + ".position_covariance", pos_cov);
      node_ptr_->get_parameter(prefix + ".orientation_covariance", ori_cov);
      std::copy_n(pos_cov.begin(), 3, config.position_values.begin());
      std::copy_n(ori_cov.begin(), 3, config.orientation_values.begin());

      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] position_covariance: [%.3f, %.3f, %.3f]",
        topic_id.c_str(), pos_cov[0], pos_cov[1], pos_cov[2]);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] orientation_covariance: [%.3f, %.3f, %.3f]",
        topic_id.c_str(), ori_cov[0], ori_cov[1], ori_cov[2]);
    }
    if (config.set_earth_map) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] Topic %s will be used to set the earth to map transform",
        topic_id.c_str(), config.topic.c_str());
      set_earth_map_from_topic_ = true;
    }

    update_pose_configs_.push_back(config);
  }

  // Create subscriptions
  predict_sub_ = node_ptr_->template create_subscription<sensor_msgs::msg::Imu>(
    predict_topic, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::imuCallback, this, std::placeholders::_1));

  for (const auto & config : update_pose_configs_) {
    if (config.type == "geometry_msgs/msg/PoseStamped") {
      auto sub = node_ptr_->template create_subscription<geometry_msgs::msg::PoseStamped>(
        config.topic, as2_names::topics::sensor_measurements::qos,
        [this, config](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          this->poseCallback(msg, config);
        });
      update_pose_subs_.push_back(sub);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Created PoseStamped subscription for topic: %s", config.topic.c_str());
    } else if (config.type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
      auto sub = node_ptr_->template create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        config.topic, as2_names::topics::sensor_measurements::qos,
        [this, config](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          this->poseWithCovarianceCallback(msg, config);
        });
      update_pose_cov_subs_.push_back(sub);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Created PoseWithCovarianceStamped subscription for topic: %s", config.topic.c_str());
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Unknown message type '%s' for topic %s. Supported types: "
        "geometry_msgs/msg/PoseStamped, geometry_msgs/msg/PoseWithCovarianceStamped",
        config.type.c_str(), config.topic.c_str());
    }
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

void Plugin::publishState()
{
  // Get current time for timestamping the transforms
  builtin_interfaces::msg::Time current_time = node_ptr_->now();

  // Publish map to odom transform
  state_estimator_interface_->setMapToOdomPose(
    map_to_odom_, current_time);

  // Publish odom to base_link transform
  state_estimator_interface_->setOdomToBaseLinkPose(
    odom_to_baselink_, current_time);

  // Publish twist in base frame
  state_estimator_interface_->setTwistInBaseFrame(
    twist_in_base_, current_time);
}

void Plugin::updateStateFromEkf()
{
  // Get the current EKF state
  ekf::State current_state = ekf_wrapper_.get_state();

  // Update all transforms from EKF state
  Eigen::Matrix4d map_to_odom_matrix = ekf_wrapper_.get_map_to_odom();
  map_to_odom_ = eigenMatrix4dToTf2Transform(map_to_odom_matrix);
  StateTransforms transforms(current_state, map_to_odom_);
  odom_to_baselink_ = transforms.odom_to_base;
  twist_in_base_ = ekfStateToTwist(current_state, transforms.map_to_base, last_imu_msg_);
}

void Plugin::processImu(const sensor_msgs::msg::Imu & msg)
{
  // Create EKF input from IMU message
  ekf::Input input(
    {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z});

  // Get time delta from last IMU message
  double dt = 0.0;
  if (last_imu_msg_.header.stamp.sec > 0 || last_imu_msg_.header.stamp.nanosec > 0) {
    dt = (msg.header.stamp.sec - last_imu_msg_.header.stamp.sec) +
      (msg.header.stamp.nanosec - last_imu_msg_.header.stamp.nanosec) * 1e-9;
  } else {
    RCLCPP_WARN(node_ptr_->get_logger(), "Received first IMU message, initializing EKF state");
  }

  // Perform EKF prediction step
  ekf_wrapper_.predict(input, dt);
}

void Plugin::processPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  // TODO(user): Implement EKF update step with pose measurement
  // This should:
  // 1. Convert PoseWithCovarianceStamped to ekf::PoseMeasurement
  // 2. Call ekf_wrapper_.update() with the measurement
  // 3. Update the state from EKF
  if (verbose_) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Processing pose measurement at time %d.%09d",
      msg.header.stamp.sec, msg.header.stamp.nanosec);
  }
}

void Plugin::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!set_earth_map_from_topic_) {
    setupTfTree();
  }
  if (earth_to_map_set_) {
    processImu(*msg);
    updateStateFromEkf();
    publishState();
  }
  last_imu_msg_ = *msg;
}

void Plugin::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg,
  const PoseTopicConfig & config)
{
  if (earth_to_map_set_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose.pose = msg->pose;

    // Set covariance from config using utility function
    pose_msg.pose.covariance = generateCovarianceFromConfig(config);

    if (config.use_message_covariance) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Pose message on topic %s has no covariance, but use_message_covariance is true. "
        "Using zero covariance as placeholder.",
        config.topic.c_str());
    }

    processPose(pose_msg);
    updateStateFromEkf();
    publishState();
  } else {
    if (config.set_earth_map) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Setting earth to map transform from first received pose in topic %s",
        config.topic.c_str());
      // Set earth to map transform from the first received pose
      tf2::fromMsg(msg->pose, earth_to_map_);
      state_estimator_interface_->setEarthToMap(earth_to_map_, msg->header.stamp, true);
      setupTfTree();
    } else {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Received pose message on topic %s but earth to map transform is not set.",
        config.topic.c_str());
    }
  }
}

void Plugin::poseWithCovarianceCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const PoseTopicConfig & config)
{
  if (earth_to_map_set_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg = *msg;

    // Get covariance based on config (replaces or multiplies existing values)
    pose_msg.pose.covariance = getCovarianceWithConfig(msg->pose.covariance, config);

    processPose(pose_msg);
    updateStateFromEkf();
    publishState();
  } else {
    if (config.set_earth_map) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Setting earth to map transform from first received pose in topic %s",
        config.topic.c_str());
      // Set earth to map transform from the first received pose
      tf2::fromMsg(msg->pose.pose, earth_to_map_);
      state_estimator_interface_->setEarthToMap(earth_to_map_, msg->header.stamp, true);
      setupTfTree();
    } else {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Received pose message on topic %s but earth to map transform is not set.",
        config.topic.c_str());
    }
  }
}

}  // namespace simple_ekf

PLUGINLIB_EXPORT_CLASS(simple_ekf::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
