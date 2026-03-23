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
#include <nav_msgs/msg/odometry.hpp>
#include "simple_ekf/simple_ekf.hpp"

namespace simple_ekf
{

void Plugin::setupWrapper()
{
  // Initial state covariance — read from parameters
  double position_cov = getParameter<double>(
    node_ptr_, "simple_ekf.initial_covariance.position");
  double velocity_cov = getParameter<double>(
    node_ptr_, "simple_ekf.initial_covariance.velocity");
  double orientation_cov = getParameter<double>(
    node_ptr_, "simple_ekf.initial_covariance.orientation");
  double bias_acc_cov = getParameter<double>(
    node_ptr_, "simple_ekf.initial_covariance.bias_acc");
  double bias_gyro_cov = getParameter<double>(
    node_ptr_, "simple_ekf.initial_covariance.bias_gyro");

  std::array<double, ekf::Covariance::size> initial_covariance_values;
  initial_covariance_values.fill(0.0);
  initial_covariance_values[ekf::Covariance::X] = position_cov;
  initial_covariance_values[ekf::Covariance::Y] = position_cov;
  initial_covariance_values[ekf::Covariance::Z] = position_cov;
  initial_covariance_values[ekf::Covariance::VX] = velocity_cov;
  initial_covariance_values[ekf::Covariance::VY] = velocity_cov;
  initial_covariance_values[ekf::Covariance::VZ] = velocity_cov;
  initial_covariance_values[ekf::Covariance::ROLL] = orientation_cov;
  initial_covariance_values[ekf::Covariance::PITCH] = orientation_cov;
  initial_covariance_values[ekf::Covariance::YAW] = orientation_cov;
  initial_covariance_values[ekf::Covariance::ABX] = bias_acc_cov;
  initial_covariance_values[ekf::Covariance::ABY] = bias_acc_cov;
  initial_covariance_values[ekf::Covariance::ABZ] = bias_acc_cov;
  initial_covariance_values[ekf::Covariance::WBX] = bias_gyro_cov;
  initial_covariance_values[ekf::Covariance::WBY] = bias_gyro_cov;
  initial_covariance_values[ekf::Covariance::WBZ] = bias_gyro_cov;
  ekf_wrapper_.reset(ekf::State(), ekf::Covariance(initial_covariance_values));

  // Gravity
  double gravity = getParameter<double>(node_ptr_, "simple_ekf.gravity");
  ekf_wrapper_.set_gravity(
    ekf::Gravity(std::array<double, ekf::Gravity::size>({0.0, 0.0, gravity})));

  // IMU noise parameters
  Eigen::Vector<double, 6> imu_noise;
  imu_noise << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  double accelerometer_noise_density = getParameter<double>(
    node_ptr_, "simple_ekf.imu_params.accelerometer_noise_density");
  double gyroscope_noise_density = getParameter<double>(
    node_ptr_, "simple_ekf.imu_params.gyroscope_noise_density");
  double accelerometer_random_walk = getParameter<double>(
    node_ptr_, "simple_ekf.imu_params.accelerometer_random_walk");
  double gyroscope_random_walk = getParameter<double>(
    node_ptr_, "simple_ekf.imu_params.gyroscope_random_walk");
  ekf_wrapper_.set_noise_parameters(
    imu_noise, accelerometer_noise_density, gyroscope_noise_density,
    accelerometer_random_walk, gyroscope_random_walk);
}

void Plugin::onSetup()
{
  // Verbose logging for debugging purposes
  verbose_ = getParameter<bool>(node_ptr_, "simple_ekf.verbose");
  debug_verbose_ = getParameter<bool>(node_ptr_, "simple_ekf.debug_verbose");

  // Setup EKF wrapper (initial covariance, gravity, IMU noise)
  setupWrapper();

  // Whether to publish the earth→map transform as tf_static (true) or dynamic tf (false)
  earth_to_map_static_tf_ = getParameter<bool>(
    node_ptr_, "simple_ekf.earth_map_transform.static_tf");
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Earth to map transform will be published as %s",
    earth_to_map_static_tf_ ? "tf_static" : "tf (dynamic)");

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

    // Read rigid_body_name for mocap topics (ignored for other types)
    if (config.type == "mocap4r2_msgs/msg/RigidBodies") {
      try {
        config.rigid_body_name = getParameter<std::string>(
          node_ptr_, prefix + ".rigid_body_name");
      } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
        // Parameter might be an integer — convert to string
        int rigid_body_id = getParameter<int>(node_ptr_, prefix + ".rigid_body_name");
        config.rigid_body_name = std::to_string(rigid_body_id);
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "  [%s] rigid_body_name was an integer (%d), converted to string '%s'. "
          "Consider using quotes in YAML: rigid_body_name: \"%d\"",
          topic_id.c_str(), rigid_body_id, config.rigid_body_name.c_str(), rigid_body_id);
      }
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "  [%s] rigid_body_name: %s", topic_id.c_str(), config.rigid_body_name.c_str());
    }

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

  std::string offboard_topic = getParameter<std::string>(node_ptr_, "simple_ekf.offboard_topic");
  offboard_sub_ = node_ptr_->template create_subscription<as2_msgs::msg::PlatformInfo>(
    offboard_topic, as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::platformInfoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Subscribed to platform info topic: %s", offboard_topic.c_str());

  double timer_hz = getParameter<double>(node_ptr_, "simple_ekf.timer_hz");
  timer_ = node_ptr_->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_hz),
    std::bind(&Plugin::timerCallback, this));
  RCLCPP_INFO(node_ptr_->get_logger(), "EKF publish timer set to %.1f Hz", timer_hz);

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
    } else if (config.type == "nav_msgs/msg/Odometry") {
      auto sub = node_ptr_->template create_subscription<nav_msgs::msg::Odometry>(
        config.topic, as2_names::topics::sensor_measurements::qos,
        [this, config](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odometryCallback(msg, config);
        });
      update_odom_subs_.push_back(sub);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Created Odometry subscription for topic: %s", config.topic.c_str());
    } else if (config.type == "mocap4r2_msgs/msg/RigidBodies") {
      auto sub = node_ptr_->template create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        config.topic, as2_names::topics::sensor_measurements::qos,
        [this, config](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
          this->mocapCallback(msg, config);
        });
      update_mocap_subs_.push_back(sub);
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Created RigidBodies subscription for topic: %s (rigid_body_name: %s)",
        config.topic.c_str(), config.rigid_body_name.c_str());
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Unknown message type '%s' for topic %s. Supported types: "
        "geometry_msgs/msg/PoseStamped, geometry_msgs/msg/PoseWithCovarianceStamped, "
        "nav_msgs/msg/Odometry, mocap4r2_msgs/msg/RigidBodies",
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
    state_estimator_interface_->setEarthToMap(
      earth_to_map_, node_ptr_->now(), earth_to_map_static_tf_);
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

  // Get time delta from last IMU message.
  // Use rclcpp::Time arithmetic to avoid unsigned underflow: nanosec fields are uint32_t,
  // so direct subtraction wraps around when crossing a second boundary and produces a
  // spuriously large dt.
  double dt = 0.0;
  if (last_imu_msg_.header.stamp.sec > 0 || last_imu_msg_.header.stamp.nanosec > 0) {
    dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(last_imu_msg_.header.stamp)).seconds();
  } else {
    if (verbose_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Received first IMU message, initializing EKF state");
    }
  }

  // Perform EKF prediction step
  ekf_wrapper_.predict(input, dt);

  if (debug_verbose_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Processed IMU message [%.6f, %.6f, %.6f] for EKF prediction with dt = %.6f seconds",
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
      dt);
  }
}

void Plugin::processPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg, bool is_odom)
{
  // TODO(user): Implement EKF update step with pose measurement
  // This should:
  // 1. Convert PoseWithCovarianceStamped to ekf::PoseMeasurement
  // 2. Call ekf_wrapper_.update() with the measurement
  // 3. Update the state from EKF
  if (debug_verbose_) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Processing pose measurement at time %d.%09d",
      msg.header.stamp.sec, msg.header.stamp.nanosec);
  }

  // Get the current (freshly predicted) EKF state and build the transforms from it,
  // instead of using the stale last-published map_to_odom_ * odom_to_baselink_ product.
  // This avoids frame-transform inconsistencies when a pose update arrives between IMU callbacks.
  ekf::State current_state = ekf_wrapper_.get_state();
  StateTransforms transforms(current_state, map_to_odom_);

  // Transform the incoming pose measurement to the map frame
  geometry_msgs::msg::PoseWithCovarianceStamped measurement_in_map = transformPoseToMapFrame(
    transforms, earth_to_map_, msg);

  if (debug_verbose_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Transformed pose measurement to map frame for EKF update");
  }

  // Convert to EKF measurement format, passing the current state so that the measured
  // Euler angles are unwrapped relative to the state angles (Fix 1: angle unwrapping).
  ekf::PoseMeasurement measurement = poseWithCovarianceToEkfMeasurement(
    measurement_in_map, current_state);
  ekf::PoseMeasurementCovariance measurement_cov = poseWithCovarianceToEkfMeasurementCovariance(
    measurement_in_map.pose);

  if (debug_verbose_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Converted pose measurement to EKF format:"
      "[x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]",
      measurement.data[ekf::PoseMeasurement::X],
      measurement.data[ekf::PoseMeasurement::Y],
      measurement.data[ekf::PoseMeasurement::Z],
      measurement.data[ekf::PoseMeasurement::ROLL],
      measurement.data[ekf::PoseMeasurement::PITCH],
      measurement.data[ekf::PoseMeasurement::YAW]);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Pose measurement covariance:"
      "[c_x=%.6f, c_y=%.6f, c_z=%.6f, c_roll=%.6f, c_pitch=%.6f, c_yaw=%.6f]",
      measurement_cov.data[ekf::PoseMeasurementCovariance::X],
      measurement_cov.data[ekf::PoseMeasurementCovariance::Y],
      measurement_cov.data[ekf::PoseMeasurementCovariance::Z],
      measurement_cov.data[ekf::PoseMeasurementCovariance::ROLL],
      measurement_cov.data[ekf::PoseMeasurementCovariance::PITCH],
      measurement_cov.data[ekf::PoseMeasurementCovariance::YAW]);
  }

  // Perform EKF update step
  if (!is_odom) {
    ekf_wrapper_.update_pose(measurement, measurement_cov);
  } else {
    ekf_wrapper_.update_pose_odom(measurement, measurement_cov);
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

    // Print state for debugging purposes
    if (debug_verbose_) {
      ekf::State state = ekf_wrapper_.get_state();
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Predicted EKF state from imu message:"
        "[x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]",
        state.get_position()[0], state.get_position()[1], state.get_position()[2],
        state.get_orientation()[0], state.get_orientation()[1], state.get_orientation()[2]);
      ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Predicted EKF covariance after imu message:"
        "[c_x=%.6f, c_y=%.6f, c_z=%.6f, c_roll=%.6f, c_pitch=%.6f, c_yaw=%.6f]",
        covariance.data[ekf::Covariance::X],
        covariance.data[ekf::Covariance::Y],
        covariance.data[ekf::Covariance::Z],
        covariance.data[ekf::Covariance::ROLL],
        covariance.data[ekf::Covariance::PITCH],
        covariance.data[ekf::Covariance::YAW]);
    }

    // Only track the timestamp of messages that were actually fed into the EKF.
    // If we updated last_imu_msg_ unconditionally, messages received before
    // earth_to_map is set would create a huge dt on the first real prediction.
    last_imu_msg_ = *msg;
  }
}

void Plugin::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  drone_offboard_ = msg->offboard;
}

void Plugin::timerCallback()
{
  // Republish earth→map as dynamic tf if not using tf_static
  if (earth_to_map_set_ && !earth_to_map_static_tf_) {
    state_estimator_interface_->setEarthToMap(
      earth_to_map_, node_ptr_->now(), false);
  }

  // TODO(rdasilva01): Offboard check. If false, update with 0 0 0. If true, do nothing.
  if (earth_to_map_set_) {
    if (!drone_offboard_) {
      geometry_msgs::msg::PoseWithCovarianceStamped zero_pose;
      zero_pose.header.stamp = node_ptr_->now();
      zero_pose.pose.pose.position.x = 0.0;
      zero_pose.pose.pose.position.y = 0.0;
      zero_pose.pose.pose.position.z = 0.0;
      zero_pose.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
      zero_pose.pose.covariance.fill(1e-5);  // Very low covariance to trust this measurement
      processPose(zero_pose);
      updateStateFromEkf();
      publishState();
      if (debug_verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Offboard is false, updated EKF with zero pose measurement");
      }
    }
  }
}

bool Plugin::setEarthToMapFromFirstPose(
  const tf2::Transform & pose,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  const std::string & earth_frame = state_estimator_interface_->getEarthFrame();
  const std::string & map_frame = state_estimator_interface_->getMapFrame();

  // Strip leading slash if present (e.g. "/drone0/map" → "drone0/map")
  const std::string bare_frame = (!frame_id.empty() && frame_id[0] == '/') ?
    frame_id.substr(1) : frame_id;

  if (bare_frame == earth_frame) {
    earth_to_map_ = pose;
  } else if (bare_frame == map_frame) {
    // Pose is expressed in map frame (i.e. map→drone), so earth→map is its inverse
    earth_to_map_ = pose.inverse();
  } else {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Cannot set earth→map from frame '%s'. Expected '%s' (earth) or '%s' (map). Ignoring."
      "[bare_frame='%s']",
      frame_id.c_str(), earth_frame.c_str(), map_frame.c_str(), bare_frame.c_str());
    return false;
  }

  state_estimator_interface_->setEarthToMap(
    earth_to_map_, stamp, earth_to_map_static_tf_);
  return true;
}

void Plugin::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg,
  const PoseTopicConfig & config)
{
  if (earth_to_map_set_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose.pose = msg->pose;

    // Get state
    ekf::State state = ekf_wrapper_.get_state();

    // Set covariance from config using utility function
    pose_msg.pose.covariance = generateCovarianceFromConfig(config);

    if (config.use_message_covariance && verbose_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Pose message on topic %s has no covariance, but use_message_covariance is true. "
        "Using zero covariance as placeholder.",
        config.topic.c_str());
    }

    processPose(pose_msg);
    updateStateFromEkf();
    publishState();

    // Print state for debugging purposes
    if (debug_verbose_) {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Updated EKF state from pose message on topic %s:"
        "[x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]",
        config.topic.c_str(),
        state.get_position()[0], state.get_position()[1], state.get_position()[2],
        state.get_orientation()[0], state.get_orientation()[1], state.get_orientation()[2]);
      ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Updated EKF covariance after pose message on topic %s:"
        "[c_x=%.6f, c_y=%.6f, c_z=%.6f, c_roll=%.6f, c_pitch=%.6f, c_yaw=%.6f]",
        config.topic.c_str(),
        covariance.data[ekf::Covariance::X],
        covariance.data[ekf::Covariance::Y],
        covariance.data[ekf::Covariance::Z],
        covariance.data[ekf::Covariance::ROLL],
        covariance.data[ekf::Covariance::PITCH],
        covariance.data[ekf::Covariance::YAW]);
    }

    // Check if the jump in the pose is too large and warn if so (only for debugging purposes)
    if (debug_verbose_) {
      ekf::State current_state = ekf_wrapper_.get_state();
      double dx = current_state.get_position()[0] - state.get_position()[0];
      double dy = current_state.get_position()[1] - state.get_position()[1];
      double dz = current_state.get_position()[2] - state.get_position()[2];
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance > 1.0) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Received pose message on topic %s, distance from previous state: %.3f m",
          config.topic.c_str(), distance);
        // Stop everything to analyze the issue
        rclcpp::shutdown();
      }
    }

  } else {
    if (config.set_earth_map) {
      if (verbose_) {
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Setting earth to map transform from first received pose in topic %s",
          config.topic.c_str());
      }
      // Set earth to map transform from the first received pose
      tf2::Transform raw_pose;
      tf2::fromMsg(msg->pose, raw_pose);
      if (setEarthToMapFromFirstPose(raw_pose, msg->header.frame_id, msg->header.stamp)) {
        setupTfTree();
      }
    } else {
      if (verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Received pose message on topic %s but earth to map transform is not set.",
          config.topic.c_str());
      }
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
      if (verbose_) {
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Setting earth to map transform from first received pose in topic %s",
          config.topic.c_str());
      }
      // Set earth to map transform from the first received pose
      tf2::Transform raw_pose;
      tf2::fromMsg(msg->pose.pose, raw_pose);
      if (setEarthToMapFromFirstPose(raw_pose, msg->header.frame_id, msg->header.stamp)) {
        setupTfTree();
      }
    } else {
      if (verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Received pose message on topic %s but earth to map transform is not set.",
          config.topic.c_str());
      }
    }
  }
}

void Plugin::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg,
  const PoseTopicConfig & config)
{
  // Reuse the PoseWithCovarianceStamped path: extract the pose part from the odometry
  // message (ignoring twist) and forward it. The child_frame_id of the odometry becomes
  // the frame_id of the pose so that transformPoseToMapFrame picks the right transform.
  if (earth_to_map_set_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose;

    // Apply covariance config (replace or multiply)
    pose_msg.pose.covariance = getCovarianceWithConfig(msg->pose.covariance, config);

    processPose(pose_msg, true);
    updateStateFromEkf();
    publishState();
  } else {
    if (config.set_earth_map) {
      if (verbose_) {
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Setting earth to map transform from first received odometry on topic %s",
          config.topic.c_str());
      }
      // Use the pose part of the odometry to set the earth-to-map transform
      tf2::Transform raw_pose;
      tf2::fromMsg(msg->pose.pose, raw_pose);
      if (setEarthToMapFromFirstPose(raw_pose, msg->header.frame_id, msg->header.stamp)) {
        setupTfTree();
      }
    } else {
      if (verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Received odometry on topic %s but earth to map transform is not set.",
          config.topic.c_str());
      }
    }
  }
}

void Plugin::mocapCallback(
  const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
  const PoseTopicConfig & config)
{
  // Find the rigid body matching the configured name
  for (const auto & rigid_body : msg->rigidbodies) {
    if (rigid_body.rigid_body_name != config.rigid_body_name) {
      continue;
    }

    // Skip all-zero poses: the mocap system publishes zeros when the body is not detected
    if (rigid_body.pose.position.x == 0.0 && rigid_body.pose.position.y == 0.0 &&
      rigid_body.pose.position.z == 0.0 &&
      rigid_body.pose.orientation.x == 0.0 && rigid_body.pose.orientation.y == 0.0 &&
      rigid_body.pose.orientation.z == 0.0)
    {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Rigid body '%s' has all-zero pose, skipping (body not detected)",
        config.rigid_body_name.c_str());
      return;
    }

    // Skip duplicate poses to avoid a zero-dt update
    tf2::Vector3 current_pose(
      rigid_body.pose.position.x,
      rigid_body.pose.position.y,
      rigid_body.pose.position.z);
    auto it = last_mocap_pose_.find(config.rigid_body_name);
    if (it != last_mocap_pose_.end() && isSamePose(current_pose, it->second)) {
      if (debug_verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Received the same pose as the last one for rigid body '%s', skipping duplicate",
          config.rigid_body_name.c_str());
      }
      return;
    }
    last_mocap_pose_[config.rigid_body_name] = current_pose;

    // Build a PoseStamped in the earth frame — mocap data is always in the earth/world frame
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = state_estimator_interface_->getEarthFrame();
    pose_msg.pose = rigid_body.pose;

    // Reuse the standard PoseStamped pipeline (handles earth→map transform, covariance, etc.)
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = pose_msg.header;
    pose_cov_msg.pose.pose = pose_msg.pose;
    pose_cov_msg.pose.covariance = generateCovarianceFromConfig(config);

    if (earth_to_map_set_) {
      processPose(pose_cov_msg);
      updateStateFromEkf();
      publishState();
    } else if (config.set_earth_map) {
      if (verbose_) {
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Setting earth to map transform from first mocap pose for rigid body '%s'",
          config.rigid_body_name.c_str());
      }
      tf2::Transform raw_pose;
      tf2::fromMsg(pose_msg.pose, raw_pose);
      if (setEarthToMapFromFirstPose(raw_pose, pose_msg.header.frame_id, msg->header.stamp)) {
        setupTfTree();
      }
    } else {
      if (verbose_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Received mocap pose for '%s' but earth to map transform is not set.",
          config.rigid_body_name.c_str());
      }
    }
    return;
  }

  // Rigid body not found in this message
  RCLCPP_WARN(
    node_ptr_->get_logger(),
    "Rigid body '%s' not found in mocap message. Available bodies:",
    config.rigid_body_name.c_str());
  for (const auto & rigid_body : msg->rigidbodies) {
    RCLCPP_WARN(node_ptr_->get_logger(), "  - %s", rigid_body.rigid_body_name.c_str());
  }
}

}  // namespace simple_ekf

PLUGINLIB_EXPORT_CLASS(simple_ekf::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
