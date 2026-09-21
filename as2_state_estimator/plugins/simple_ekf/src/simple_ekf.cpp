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

#include "simple_ekf/simple_ekf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "as2_core/names/topics.hpp"
#include "simple_ekf/ros_conversions.hpp"

namespace simple_ekf
{

void Plugin::onSetup()
{
  frame_ids_ = {
    state_estimator_interface_->getEarthFrame(), state_estimator_interface_->getMapFrame(),
    state_estimator_interface_->getOdomFrame(), state_estimator_interface_->getBaseFrame()};

  verbose_ = node_ptr_->getParameter<bool>("simple_ekf.verbose");
  debug_verbose_ = node_ptr_->getParameter<bool>("simple_ekf.debug_verbose");

  filter_ = std::make_unique<simple_ekf_core::Filter>(readFilterConfig(), makeLogSink());

  earth_to_map_static_tf_ =
    node_ptr_->getParameter<bool>("simple_ekf.earth_map_transform.static_tf");
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Earth to map transform will be published as %s",
    earth_to_map_static_tf_ ? "tf_static" : "tf (dynamic)");

  if (node_ptr_->getParameter<bool>("simple_ekf.earth_map_transform.set_earth_map")) {
    const std::string prefix = "simple_ekf.earth_map_transform.";
    simple_ekf_core::Quaternion rotation;
    rotation.setRPY(
      node_ptr_->getParameter<double>(prefix + "orientation.roll"),
      node_ptr_->getParameter<double>(prefix + "orientation.pitch"),
      node_ptr_->getParameter<double>(prefix + "orientation.yaw"));
    filter_->setEarthToMap(
      simple_ekf_core::Rigid(
        rotation, simple_ekf_core::Vector3(
          node_ptr_->getParameter<double>(prefix + "position.x"),
          node_ptr_->getParameter<double>(prefix + "position.y"),
          node_ptr_->getParameter<double>(prefix + "position.z"))));
    RCLCPP_INFO(node_ptr_->get_logger(), "Earth to map transform set from parameters");
  }

  std::vector<std::string> topic_ids;
  node_ptr_->get_parameter("simple_ekf.update_topics", topic_ids);
  RCLCPP_INFO(node_ptr_->get_logger(), "Configuring %zu update topic(s):", topic_ids.size());
  std::vector<std::pair<TopicConfig, SourceId>> update_topics;
  for (const auto & topic_id : topic_ids) {
    const TopicConfig config = readTopicConfig(topic_id);
    if (config.set_earth_map && !isVelocityType(config.type)) {
      RCLCPP_INFO(
        node_ptr_->get_logger(), "  [%s] Topic %s will be used to set the earth to map transform",
        topic_id.c_str(), config.topic.c_str());
      set_earth_map_from_topic_ = true;
    }
    update_topics.emplace_back(config, filter_->addSource(config));
  }

  imu_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
    node_ptr_->getParameter<std::string>("simple_ekf.predict_topic"),
    as2_names::topics::sensor_measurements::qos,
    std::bind(&Plugin::imuCallback, this, std::placeholders::_1));

  const std::string platform_topic =
    node_ptr_->getParameter<std::string>("simple_ekf.platform_topic");
  if (platform_topic.empty()) {
    filter_->setOffboard(true);
    RCLCPP_INFO(node_ptr_->get_logger(), "Offboard topic is empty, assuming offboard always true");
  } else {
    platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
      platform_topic, as2_names::topics::sensor_measurements::qos,
      std::bind(&Plugin::platformInfoCallback, this, std::placeholders::_1));
  }
  use_arm_ = node_ptr_->getParameter<bool>("simple_ekf.use_arm");
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Using %s status for EKF reset logic", use_arm_ ? "arm" : "offboard");

  const double timer_hz = node_ptr_->getParameter<double>("simple_ekf.timer_hz");
  timer_ = node_ptr_->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_hz), std::bind(&Plugin::timerCallback, this));

  const double map_odom_alpha = filter_->config().map_odom_alpha;
  if (map_odom_alpha == 1.0) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Output smoothing disabled, publishing raw EKF state");
  } else {
    // log1p(-alpha) rather than log(1 - alpha): alpha is meant to be small (1e-4 is heavy
    // smoothing), and the subtraction would throw away most of its significant digits
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Output smoothing alpha %g at %.1f Hz (time constant %.0f ms)",
      map_odom_alpha, timer_hz, -1000.0 / (timer_hz * std::log1p(-map_odom_alpha)));
  }

  const std::string internal_debug_base =
    node_ptr_->getParameter<std::string>("simple_ekf.internal_ekf_debug_topics");
  if (!internal_debug_base.empty()) {
    internal_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      internal_debug_base + "/pose", 10);
    internal_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      internal_debug_base + "/twist", 10);
    internal_map_to_odom_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      internal_debug_base + "/map_to_odom", 10);
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Publishing raw internal EKF state under %s/",
      internal_debug_base.c_str());
  }

  // Last, so that the IMU subscription comes first: the executor takes ready subscriptions
  // in creation order, and an IMU reading and a measurement that arrive together must be
  // predicted with before the measurement corrects.
  for (const auto & [config, source] : update_topics) {
    subscribe(config, source);
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

simple_ekf_core::Config Plugin::readFilterConfig()
{
  const auto get = [this](const std::string & name) {
      return node_ptr_->getParameter<double>("simple_ekf." + name);
    };

  simple_ekf_core::Config config;
  config.initial_position_covariance = get("initial_covariance.position");
  config.initial_velocity_covariance = get("initial_covariance.velocity");
  config.initial_orientation_covariance = get("initial_covariance.orientation");
  config.initial_bias_acc_covariance = get("initial_covariance.bias_acc");
  config.initial_bias_gyro_covariance = get("initial_covariance.bias_gyro");
  config.gravity = get("gravity");
  config.accelerometer_noise_density = get("imu_params.accelerometer_noise_density");
  config.gyroscope_noise_density = get("imu_params.gyroscope_noise_density");
  config.accelerometer_random_walk = get("imu_params.accelerometer_random_walk");
  config.gyroscope_random_walk = get("imu_params.gyroscope_random_walk");
  config.max_update_latency_ms = get("max_update_latency_ms");
  config.unobserved_variance = node_ptr_->getParameter<double>(
    "simple_ekf.unobserved_variance", config.unobserved_variance);
  config.map_odom_alpha = get("map_odom_alpha");
  config.verbose = verbose_;
  config.debug_verbose = debug_verbose_;

  // Optional: without them, the pre-flight correction holds the drone at the map origin
  const auto get_or = [this](const std::string & name, double default_value) {
      return node_ptr_->getParameter<double>(
        "simple_ekf.preflight_correction." + name,
        default_value);
    };
  config.preflight_variance = get_or("variance", config.preflight_variance);
  simple_ekf_core::Quaternion rotation;
  rotation.setRPY(
    get_or("orientation.roll", 0.0), get_or("orientation.pitch", 0.0),
    get_or("orientation.yaw", 0.0));
  config.preflight_pose = simple_ekf_core::Rigid(
    rotation, simple_ekf_core::Vector3(
      get_or("position.x", 0.0), get_or("position.y", 0.0), get_or("position.z", 0.0)));
  return config;
}

TopicConfig Plugin::readTopicConfig(const std::string & topic_id)
{
  const std::string prefix = "simple_ekf." + topic_id + ".";
  RCLCPP_INFO(node_ptr_->get_logger(), "  - %s", topic_id.c_str());

  TopicConfig config;
  config.topic = node_ptr_->getParameter<std::string>(prefix + "topic");
  config.name = config.topic;
  config.type = node_ptr_->getParameter<std::string>(prefix + "type");
  config.set_earth_map = node_ptr_->getParameter<bool>(prefix + "set_earth_map");
  config.use_message_covariance =
    node_ptr_->getParameter<bool>(prefix + "use_message_covariance");
  config.update_rate_hz = node_ptr_->getParameter<double>(prefix + "update_rate_hz", 0.0);
  config.is_odometry = readTypeDefaultedFlag(
    topic_id, "is_odometry", defaultIsOdometryForType(config.type));
  config.reject_repeated_positions = readTypeDefaultedFlag(
    topic_id, "reject_repeated_positions", defaultRejectRepeatedPositionsForType(config.type));
  config.repeated_position_threshold = node_ptr_->getParameter<double>(
    prefix + "repeated_position_threshold", config.repeated_position_threshold);

  // Off by default: a source that alone observes a state has nothing to be gated against
  config.innovation_gate = node_ptr_->getParameter<double>(prefix + "innovation_gate", 0.0);
  config.innovation_gate_timeout =
    node_ptr_->getParameter<double>(prefix + "innovation_gate_timeout", 1.0);
  if (config.innovation_gate > 0.0) {
    RCLCPP_INFO(
      node_ptr_->get_logger(), "  [%s] innovation_gate: %.1f sigma, forced through after %.1f s",
      topic_id.c_str(), config.innovation_gate, config.innovation_gate_timeout);
  }

  if (config.use_message_covariance && !carriesCovariance(config.type)) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "  [%s] use_message_covariance is true, but %s carries no covariance: every component "
      "will read as unmeasured and the topic will change nothing. Set it to false",
      topic_id.c_str(), config.type.c_str());
  }

  if (config.type == kRigidBodiesType) {
    const std::string name = prefix + "rigid_body_name";
    try {
      config.rigid_body_name = node_ptr_->getParameter<std::string>(name);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
      const int rigid_body_id = node_ptr_->getParameter<int>(name);
      config.rigid_body_name = std::to_string(rigid_body_id);
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "  [%s] rigid_body_name was an integer (%d), converted to string '%s'. "
        "Consider using quotes in YAML: rigid_body_name: \"%d\"",
        topic_id.c_str(), rigid_body_id, config.rigid_body_name.c_str(), rigid_body_id);
    }
  }

  // %g rather than %.3f: realistic variances are 1e-4 and smaller, which %.3f prints as
  // "0.000", indistinguishable from a zero that would mean infinite trust
  const char * kind = config.use_message_covariance ? "multiplier" : "covariance";
  if (isVelocityType(config.type)) {
    // A twist correction only touches the velocity states, so only the linear values matter
    config.linear_values = config.use_message_covariance ?
      readTriple(prefix + "linear_multiplier", {1.0, 1.0, 1.0}) :
      readTriple(prefix + "linear_covariance", {1e-2, 1e-2, 1e-2});
    config.is_body_frame = node_ptr_->getParameter<bool>(prefix + "is_body_frame", true);
    RCLCPP_INFO(
      node_ptr_->get_logger(), "  [%s] linear_%s: [%g, %g, %g]", topic_id.c_str(), kind,
      config.linear_values[0], config.linear_values[1], config.linear_values[2]);
  } else {
    config.position_values = config.use_message_covariance ?
      readTriple(prefix + "position_multiplier", {1.0, 1.0, 1.0}) :
      readTriple(prefix + "position_covariance", {1e-4, 1e-4, 1e-4});
    config.orientation_values = config.use_message_covariance ?
      readTriple(prefix + "orientation_multiplier", {1.0, 1.0, 1.0}) :
      readTriple(prefix + "orientation_covariance", {1e-5, 1e-5, 1e-5});
    RCLCPP_INFO(
      node_ptr_->get_logger(), "  [%s] position_%s: [%g, %g, %g], orientation_%s: [%g, %g, %g]",
      topic_id.c_str(), kind,
      config.position_values[0], config.position_values[1], config.position_values[2], kind,
      config.orientation_values[0], config.orientation_values[1], config.orientation_values[2]);
  }

  return config;
}

bool Plugin::readTypeDefaultedFlag(
  const std::string & topic_id, const std::string & key, bool default_value)
{
  const std::string name = "simple_ekf." + topic_id + "." + key;
  bool explicitly_set = node_ptr_->has_parameter(name);
  bool value = default_value;
  try {
    value = node_ptr_->getParameter<bool>(name, default_value);
  } catch (const std::runtime_error & e) {
    // A non-bool value (`1`, `"true"`) is a formatting slip, not a reason to kill the node
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Parameter '%s' is not a boolean (%s). Use an unquoted YAML bool (true/false). "
      "Falling back to the default for its type: %s",
      name.c_str(), e.what(), default_value ? "true" : "false");
    explicitly_set = false;
  }
  RCLCPP_INFO(
    node_ptr_->get_logger(), "  [%s] %s: %s (%s)", topic_id.c_str(), key.c_str(),
    value ? "true" : "false", explicitly_set ? "explicitly configured" : "default for type");
  return value;
}

std::array<double, 3> Plugin::readTriple(
  const std::string & name, const std::array<double, 3> & default_value)
{
  const std::vector<double> values = node_ptr_->getParameter<std::vector<double>>(
    name, std::vector<double>(default_value.begin(), default_value.end()));
  if (values.size() != default_value.size()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Parameter '%s' needs 3 values but has %zu. Using the default",
      name.c_str(), values.size());
    return default_value;
  }
  return {values[0], values[1], values[2]};
}

void Plugin::subscribe(const TopicConfig & config, SourceId source)
{
  const auto & qos = as2_names::topics::sensor_measurements::qos;

  if (config.type == kPoseStampedType) {
    update_subs_.push_back(
      node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        config.topic, qos,
        [this, config, source](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          poseCallback(msg, config, source);
        }));
  } else if (config.type == kPoseWithCovarianceStampedType) {
    update_subs_.push_back(
      node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        config.topic, qos,
        [this, config, source](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          poseWithCovarianceCallback(msg, config, source);
        }));
  } else if (config.type == kOdometryType) {
    update_subs_.push_back(
      node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        config.topic, qos,
        [this, config, source](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odometryCallback(msg, config, source);
        }));
  } else if (config.type == kTwistWithCovarianceStampedType) {
    update_subs_.push_back(
      node_ptr_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        config.topic, qos,
        [this, config, source](
          const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
          twistWithCovarianceCallback(msg, config, source);
        }));
  } else if (config.type == kRigidBodiesType) {
    update_subs_.push_back(
      node_ptr_->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        config.topic, qos,
        [this, config, source](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
          mocapCallback(msg, config, source);
        }));
  } else {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Unknown message type '%s' for topic %s. Supported types: %s, %s, %s, %s, %s",
      config.type.c_str(), config.topic.c_str(), kPoseStampedType,
      kPoseWithCovarianceStampedType, kTwistWithCovarianceStampedType, kOdometryType,
      kRigidBodiesType);
    return;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Subscribed to %s (%s)", config.topic.c_str(), config.type.c_str());
}

simple_ekf_core::LogSink Plugin::makeLogSink() const
{
  const rclcpp::Logger logger = node_ptr_->get_logger();
  return [logger](simple_ekf_core::LogLevel level, const std::string & message) {
           switch (level) {
             case simple_ekf_core::LogLevel::INFO:
               RCLCPP_INFO(logger, "%s", message.c_str());
               break;
             case simple_ekf_core::LogLevel::WARN:
               RCLCPP_WARN(logger, "%s", message.c_str());
               break;
             case simple_ekf_core::LogLevel::ERROR:
               RCLCPP_ERROR(logger, "%s", message.c_str());
               break;
           }
         };
}

simple_ekf_core::Nanoseconds Plugin::nowNanoseconds() const
{
  return node_ptr_->now().nanoseconds();
}

void Plugin::startEstimation()
{
  if (filter_->isEarthToMapSet()) {
    return;
  }

  const rclcpp::Time now = node_ptr_->now();
  state_estimator_interface_->setEarthToMap(
    filter_->outputs().earth_to_map, now, earth_to_map_static_tf_);
  // Dynamic like every later update: a static identity would stay latched on /tf_static,
  // and TF buffers would return it instead of the corrected transform
  state_estimator_interface_->setMapToOdomPose(generateIdentityPose(), now, false);
  filter_->markEarthToMapSet();
}

void Plugin::publishState()
{
  const builtin_interfaces::msg::Time stamp = node_ptr_->now();
  const simple_ekf_core::Outputs & outputs = filter_->outputs();

  state_estimator_interface_->setMapToOdomPose(outputs.published_map_to_odom, stamp);
  state_estimator_interface_->setOdomToBaseLinkPose(outputs.odom_to_base, stamp);
  state_estimator_interface_->setTwistInBaseFrame(twistToMsg(outputs.twist_in_base), stamp);
  publishInternalDebugState(stamp);
}

void Plugin::publishInternalDebugState(const builtin_interfaces::msg::Time & stamp)
{
  if (!internal_pose_pub_) {
    return;
  }

  const simple_ekf_core::Outputs & outputs = filter_->outputs();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = state_estimator_interface_->getEarthFrame();
  pose.pose = rigidToPoseMsg(outputs.earth_to_map * outputs.map_to_odom * outputs.odom_to_base);
  internal_pose_pub_->publish(pose);

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = stamp;
  twist.header.frame_id = state_estimator_interface_->getBaseFrame();
  twist.twist = twistToMsg(outputs.internal_twist_in_base).twist;
  internal_twist_pub_->publish(twist);

  geometry_msgs::msg::PoseStamped map_to_odom;
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = state_estimator_interface_->getMapFrame();
  map_to_odom.pose = rigidToPoseMsg(outputs.map_to_odom);
  internal_map_to_odom_pub_->publish(map_to_odom);
}

void Plugin::logFilterState(const std::string & context) const
{
  const auto & x = filter_->state().data;
  const auto & p = filter_->stateCovariance().data;
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "EKF state %s: [x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f]", context.c_str(),
    x[ekf::State::X], x[ekf::State::Y], x[ekf::State::Z],
    x[ekf::State::ROLL], x[ekf::State::PITCH], x[ekf::State::YAW]);
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "EKF covariance %s: [c_x=%.6f, c_y=%.6f, c_z=%.6f, c_roll=%.6f, c_pitch=%.6f, c_yaw=%.6f]",
    context.c_str(),
    p[ekf::Covariance::X], p[ekf::Covariance::Y], p[ekf::Covariance::Z],
    p[ekf::Covariance::ROLL], p[ekf::Covariance::PITCH], p[ekf::Covariance::YAW]);
}

bool Plugin::setEarthToMapFromFirstPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame_id)
{
  // Exact names only: a guessed frame is not trusted to place the whole map
  const auto frame = matchFrameId(frame_id, frame_ids_);
  if (frame == simple_ekf_core::SourceFrame::EARTH || frame == simple_ekf_core::SourceFrame::MAP) {
    return filter_->setEarthToMapFromFirstPose(poseMsgToRigid(pose), *frame);
  }

  RCLCPP_WARN(
    node_ptr_->get_logger(),
    "Cannot set earth→map from frame '%s'. Expected '%s' (earth) or '%s' (map). Ignoring.",
    frame_id.c_str(), frame_ids_.earth.c_str(), frame_ids_.map.c_str());
  return false;
}

simple_ekf_core::SourceFrame Plugin::resolveFrame(
  const std::string & frame_id, const std::string & topic,
  simple_ekf_core::SourceFrame (* guess)(const std::string &))
{
  if (const auto frame = matchFrameId(frame_id, frame_ids_)) {
    return *frame;
  }

  const simple_ekf_core::SourceFrame guessed = guess(frame_id);
  if (guessed_frames_.emplace(topic, frame_id).second) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Topic %s: frame id '%s' is none of the state estimator's frames (%s, %s, %s, %s). "
      "Taking it as %s, from its name",
      topic.c_str(), frame_id.c_str(), frame_ids_.earth.c_str(), frame_ids_.map.c_str(),
      frame_ids_.odom.c_str(), frame_ids_.base.c_str(), frameIdOf(guessed, frame_ids_).c_str());
  }
  return guessed;
}

void Plugin::fusePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  const TopicConfig & config, SourceId source)
{
  if (filter_->isEarthToMapSet()) {
    const simple_ekf_core::SourceFrame frame =
      resolveFrame(msg.header.frame_id, config.topic, guessPoseSourceFrame);
    filter_->onPose(source, toPoseSample(msg, frame), nowNanoseconds());
    publishState();
    return;
  }

  if (!config.set_earth_map) {
    if (verbose_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Received a pose on topic %s but earth to map transform is not set.",
        config.topic.c_str());
    }
    return;
  }

  if (verbose_) {
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Setting earth to map transform from the first pose on topic %s",
      config.topic.c_str());
  }
  if (setEarthToMapFromFirstPose(msg.pose.pose, msg.header.frame_id)) {
    startEstimation();
  }
}

void Plugin::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!set_earth_map_from_topic_) {
    startEstimation();
  }
  if (!filter_->isEarthToMapSet()) {
    return;
  }

  filter_->onImu(toImuSample(*msg));
  publishState();

  if (debug_verbose_) {
    logFilterState("after an IMU message");
  }
}

void Plugin::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  filter_->setOffboard(use_arm_ ? msg->armed : msg->offboard);
}

void Plugin::timerCallback()
{
  if (filter_->isEarthToMapSet() && !earth_to_map_static_tf_) {
    state_estimator_interface_->setEarthToMap(
      filter_->outputs().earth_to_map, node_ptr_->now(), false);
  }

  // Only the pre-flight correction changes the state, so only then is there anything new
  // to publish
  if (filter_->onTick(nowNanoseconds())) {
    publishState();
  }
}

void Plugin::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg,
  const TopicConfig & config, SourceId source)
{
  if (filter_->shouldThrottleUpdate(source, toNanoseconds(msg->header.stamp)) ||
    filter_->isRepeatedPosition(source, pointToVector3(msg->pose.position), nowNanoseconds()))
  {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = msg->header;
  pose.pose.pose = msg->pose;
  pose.pose.covariance = simple_ekf_core::generateCovarianceFromConfig(config);

  const bool fusing = filter_->isEarthToMapSet();
  const auto position_before = filter_->state().get_position();
  fusePose(pose, config, source);
  if (!fusing || !debug_verbose_) {
    return;
  }

  logFilterState("after a pose on topic " + config.topic);

  // A jump this large is a divergence worth stopping everything for, to inspect it
  const auto position_after = filter_->state().get_position();
  const double jump = std::hypot(
    position_after[0] - position_before[0],
    position_after[1] - position_before[1],
    position_after[2] - position_before[2]);
  if (jump > 1.0) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Received pose message on topic %s, distance from previous state: %.3f m",
      config.topic.c_str(), jump);
    rclcpp::shutdown();
  }
}

void Plugin::poseWithCovarianceCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const TopicConfig & config, SourceId source)
{
  if (filter_->shouldThrottleUpdate(source, toNanoseconds(msg->header.stamp)) ||
    filter_->isRepeatedPosition(
      source, pointToVector3(msg->pose.pose.position), nowNanoseconds()))
  {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose = *msg;
  pose.pose.covariance = simple_ekf_core::getCovarianceWithConfig(msg->pose.covariance, config);
  fusePose(pose, config, source);
}

void Plugin::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg,
  const TopicConfig & config, SourceId source)
{
  if (filter_->shouldThrottleUpdate(source, toNanoseconds(msg->header.stamp)) ||
    filter_->isRepeatedPosition(
      source, pointToVector3(msg->pose.pose.position), nowNanoseconds()))
  {
    return;
  }

  // The header is kept as it is: its frame_id ("odom") is what the pose is expressed in
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose;
  pose.pose.covariance = simple_ekf_core::getCovarianceWithConfig(msg->pose.covariance, config);
  fusePose(pose, config, source);
}

void Plugin::mocapCallback(
  const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
  const TopicConfig & config, SourceId source)
{
  const auto rigid_body = std::find_if(
    msg->rigidbodies.begin(), msg->rigidbodies.end(),
    [&config](const auto & body) {return body.rigid_body_name == config.rigid_body_name;});

  if (rigid_body == msg->rigidbodies.end()) {
    std::string available;
    for (const auto & body : msg->rigidbodies) {
      available += " '" + body.rigid_body_name + "'";
    }
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
      "Rigid body '%s' not found in mocap message. Available bodies:%s",
      config.rigid_body_name.c_str(), available.c_str());
    return;
  }

  if (filter_->shouldThrottleUpdate(source, toNanoseconds(msg->header.stamp))) {
    return;
  }

  // The mocap system publishes all zeros while it does not see the body
  const auto & body_pose = rigid_body->pose;
  if (body_pose.position.x == 0.0 && body_pose.position.y == 0.0 &&
    body_pose.position.z == 0.0 && body_pose.orientation.x == 0.0 &&
    body_pose.orientation.y == 0.0 && body_pose.orientation.z == 0.0)
  {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
      "Rigid body '%s' has all-zero pose, skipping (body not detected)",
      config.rigid_body_name.c_str());
    return;
  }

  if (filter_->isRepeatedPosition(source, pointToVector3(body_pose.position), nowNanoseconds())) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = state_estimator_interface_->getEarthFrame();
  pose.pose.pose = body_pose;
  pose.pose.covariance = simple_ekf_core::generateCovarianceFromConfig(config);
  fusePose(pose, config, source);
}

void Plugin::twistWithCovarianceCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
  const TopicConfig & config, SourceId source)
{
  if (filter_->shouldThrottleUpdate(source, toNanoseconds(msg->header.stamp))) {
    return;
  }

  // A velocity says nothing about where the map is, so it cannot set earth->map
  if (!filter_->isEarthToMapSet()) {
    if (verbose_) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
        "Received a velocity on topic %s but earth to map transform is not set.",
        config.topic.c_str());
    }
    return;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped twist = *msg;
  twist.twist.covariance =
    simple_ekf_core::getLinearCovarianceWithConfig(msg->twist.covariance, config);
  const simple_ekf_core::SourceFrame frame = config.is_body_frame ?
    simple_ekf_core::SourceFrame::BASE :
    resolveFrame(msg->header.frame_id, config.topic, guessTwistSourceFrame);

  filter_->onTwist(source, toTwistSample(twist, frame), nowNanoseconds());
  publishState();
}

}  // namespace simple_ekf

PLUGINLIB_EXPORT_CLASS(simple_ekf::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
