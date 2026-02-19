// Copyright 2025 Universidad Politécnica de Madrid
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
* @file ekf_fuse.hpp
*
* An state estimation plugin ekf for AeroStack2
*
* @authors Rodrigo da Silva
*/

#ifndef EKF_FUSE_HPP_
#define EKF_FUSE_HPP_

#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <ekf/ekf_datatype.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <as2_msgs/msg/pose_with_covariance_stamped_array.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"


#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/utils/conversions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/u_int16_multi_array_stamped.hpp"

#include <ekf/ekf_wrapper.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include "ekf_fuse/ekf_fuse_pose_updater.hpp"
#include "ekf_fuse/ekf_fuse_utils.hpp"


namespace ekf_fuse
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  // geometry_msgs::msg::TwistStamped last_twist_msg_;

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> predict_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> predict_odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> update_pose_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
  update_pose_cov_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> update_pose_velocity_subs_;
  std::vector<rclcpp::Subscription<as2_msgs::msg::PoseWithCovarianceStampedArray>::SharedPtr>
  update_pose_array_subs_;

  rclcpp::Subscription<as2_msgs::msg::UInt16MultiArrayStamped>::SharedPtr offboard_control_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    debug_ekf_state_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    debug_ekf_real_corrections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    debug_ekf_state_velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_ekf_input_odom_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_ptr_;

  // EKF
  ekf::EKFWrapper ekf_wrapper_;
  double last_imu_t_ = 0.0;
  // std::array<double, 3> last_angular_velocity_ = {0.0, 0.0, 0.0};

  // Initial state and covariance
  ekf::State initial_state_;
  ekf::Covariance initial_covariance_;

  // Parameters
  bool pose_set_earth_map_ = false;
  bool pose_use_message_measurement_covariances_ = false;
  std::array<double, 3> pose_meas_position_cov_ = {1e-8, 1e-8, 1e-8};        // x, y, z
  std::array<double, 3> pose_meas_orientation_cov_ = {1e-7, 1e-7, 1e-7};     // r, p, y
  bool odom_set_earth_map_ = false;
  bool odom_use_message_measurement_covariances_ = false;
  std::array<double, 3> odom_meas_position_cov_ = {1e-8, 1e-8, 1e-8};        // x, y, z
  std::array<double, 3> odom_meas_velocity_cov_ = {1e-2, 1e-2, 1e-2};        // x, y, z
  std::array<double, 3> odom_meas_orientation_cov_ = {1e-7, 1e-7, 1e-7};     // r, p, y
  bool apply_multiplicative_factors_ = false;
  // Factor to multiply x-axis covariance (default 1.0)
  double odom_x_covariance_factor_ = 1.0;
  // Factor to multiply y-axis covariance (default 1.0)
  double odom_y_covariance_factor_ = 1.0;
  // Factor to multiply z-axis covariance (default 1.0)
  double odom_z_covariance_factor_ = 1.0;
  double fixed_earth_map_x_ = 0.0;
  double fixed_earth_map_y_ = 0.0;
  double fixed_earth_map_z_ = 0.0;
  double fixed_earth_map_roll_ = 0.0;
  double fixed_earth_map_pitch_ = 0.0;
  double fixed_earth_map_yaw_ = 0.0;
  double distance_to_origin_ = 3.0;  // Distance to start updating
  bool first_distance_check_ = true;
  bool verbose_ = false;
  bool can_update_ = true;
  bool can_predict_ = true;
  bool offboard_activated_ = true;
  bool roll_pitch_fixed_ = false;

  int offboard_control_channel_ = 6;  // Default channel for offboard control

  double max_covariance_ = 1e1;

  ekf::Input last_imu_input_ = ekf::Input();
  bool use_imu_to_predict_ = false;

  ekf::PoseMeasurement last_odometry_pose_ = ekf::PoseMeasurement();
  ekf::VelocityMeasurement last_odometry_velocity_ = ekf::VelocityMeasurement();

  // For orientation derivative calculation in debug topic
  std::array<double, 3> last_orientation_ = {0.0, 0.0, 0.0};  // roll, pitch, yaw
  double last_orientation_time_ = 0.0;
  bool orientation_initialized_ = false;
  // filtered d_roll, d_pitch, d_yaw
  std::array<double, 3> filtered_orientation_derivative_ = {0.0, 0.0, 0.0};
  // Low-pass filter coefficient (0 = no new data, 1 = no filtering)
  double derivative_filter_alpha_ = 0.2;
  // Minimum time difference (5ms) to compute valid derivative
  double min_dt_for_derivative_ = 0.005;
  // Maximum reasonable angular velocity (rad/s) for clamping
  double max_orientation_derivative_ = 10.0;

  std::vector<ekf::PoseMeasurement> accumulated_poses_;
  std::vector<ekf::PoseMeasurementCovariance> accumulated_poses_covariances_;
  std::vector<std_msgs::msg::Header> accumulated_poses_headers_;
  std::string pose_accumulation_type_ = "";  // "", "time" or "number"
  double pose_accumulation_time_ = 0.0;  // seconds
  int pose_accumulation_number_ = 0;  // number of accumulated poses

  bool use_gazebo_ = false;

  bool take_into_account_image_delay_ = false;
  bool rotate_covariance_from_odom_to_map_ = false;

  // TF related
  bool map_to_odom_initialized_ = false;
  double map_odom_alpha_ = 0.2;  // Smoothing factor for map to odom transformation
  // Enable/disable stepped smoothing (true = smooth, false = direct EKF output)
  bool activate_step_smoothing_ = true;
  // Maximum step size for position smoothing in meters
  // (smaller = smoother, larger = faster convergence)
  double max_step_ = 0.05;
  bool earth_to_map_set_ = false;
  Eigen::Matrix4d T_earth_to_map_ = Eigen::Matrix4d::Identity();
  Eigen::Vector<double, 7> global_map_to_odom_ = Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 3> global_map_to_odom_velocity_ = Eigen::Vector<double, 3>::Zero();
  tf2::Transform earth_to_baselink = tf2::Transform::getIdentity();
  tf2::Transform odom_to_baselink = tf2::Transform::getIdentity();

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool tf_initialized_ = false;

  // Pose updater
  EKFFusePoseUpdater pose_updater_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}
  void onSetup() override
  {
    pose_updater_ = EKFFusePoseUpdater(ekf_wrapper_, node_ptr_);

    global_map_to_odom_[6] = 1.0;  // w quaternion

    // Set initial state and covariance
    std::array<double, ekf::State::size> initial_state_values = {
      0.0, 0.0, 0.0,   // position
      0.0, 0.0, 0.0,   // velocity
      0.0, 0.0, 0.0,   // orientation
      0.0, 0.0, 0.0,   // bias accelerometer
      0.0, 0.0, 0.0    // bias gyroscope
    };
    initial_state_.set(initial_state_values);
    double position_cov = 0.0;      // 1e-3
    double velocity_cov = 0.0;      // 1e-5
    double orientation_cov = 0.0;   // 1e-6
    double bias_acc_cov = 1e-8;      // 1e-8
    double bias_gyro_cov = 1e-8;     // 1e-8

    verbose_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.verbose");
    use_gazebo_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.use_gazebo");

    // Get orientation derivative filter parameter (default 0.2 for moderate filtering)
    // Lower values (e.g., 0.05-0.1) = more filtering/smoother but slower response
    // Higher values (e.g., 0.5-0.8) = less filtering/faster response but more noise
    if (node_ptr_->has_parameter("ekf_fuse.derivative_filter_alpha")) {
      derivative_filter_alpha_ = getParameter<double>(
        node_ptr_, "ekf_fuse.derivative_filter_alpha");
    }

    // Get minimum dt for derivative calculation (default 0.005s = 5ms)
    // This prevents spikes from very small time differences or duplicate timestamps
    if (node_ptr_->has_parameter("ekf_fuse.min_dt_for_derivative")) {
      min_dt_for_derivative_ = getParameter<double>(
        node_ptr_, "ekf_fuse.min_dt_for_derivative");
    }

    // Get max orientation derivative for clamping (default 10 rad/s)
    if (node_ptr_->has_parameter("ekf_fuse.max_orientation_derivative")) {
      max_orientation_derivative_ = getParameter<double>(
        node_ptr_, "ekf_fuse.max_orientation_derivative");
    }

    position_cov = getParameter<double>(
      node_ptr_, "ekf_fuse.initial_covariances.position");
    velocity_cov = getParameter<double>(
      node_ptr_, "ekf_fuse.initial_covariances.velocity");
    orientation_cov = getParameter<double>(
      node_ptr_, "ekf_fuse.initial_covariances.orientation");
    bias_acc_cov = getParameter<double>(
      node_ptr_, "ekf_fuse.initial_covariances.bias_accelerometer");
    bias_gyro_cov = getParameter<double>(
      node_ptr_, "ekf_fuse.initial_covariances.bias_gyroscope");

    std::array<double, ekf::Covariance::size> initial_covariance_values;
    initial_covariance_values.fill(0.0);
    // {
    //   position_cov, position_cov, position_cov,
    //   velocity_cov, velocity_cov, velocity_cov,
    //   orientation_cov, orientation_cov, orientation_cov,
    //   bias_acc_cov, bias_acc_cov, bias_acc_cov,
    //   bias_gyro_cov, bias_gyro_cov, bias_gyro_cov
    // };
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
    initial_covariance_ = ekf::Covariance(initial_covariance_values);
    ekf_wrapper_.reset(
      initial_state_,
      initial_covariance_
    );

    // Print state
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Initial EKF State:\n %s",
      ekf_wrapper_.get_state().to_string().c_str());
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Initial EKF Covariance:\n %s",
      ekf_wrapper_.get_state_covariance().to_string_diagonal().c_str());

    // Print initial_covariance_values
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Initial EKF Covariance Values:\n");
    for (size_t i = 0; i < ekf::Covariance::size; i++) {
      RCLCPP_INFO(
        node_ptr_->get_logger(), "  Cov[%zu] = %f", i, initial_covariance_values[i]);
    }

    // Get max_covariance param
    max_covariance_ = getParameter<double>(node_ptr_, "ekf_fuse.max_covariance");

    // Set gravity Vector
    ekf::Gravity gravity = ekf::Gravity(std::array<double, ekf::Gravity::size>({0.00, 0.0, 9.81}));
    ekf_wrapper_.set_gravity(gravity);

    // IMU noise parameters
    Eigen::Vector<double, 6> imu_noise;
    imu_noise << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    double accelerometer_noise_density = 1e-3;
    double gyroscope_noise_density = 1e-4;
    double accelerometer_random_walk = 1e-4;
    double gyroscope_random_walk = 1e-5;

    accelerometer_noise_density = getParameter<double>(
      node_ptr_, "ekf_fuse.imu_params.accelerometer_noise_density");
    gyroscope_noise_density = getParameter<double>(
      node_ptr_, "ekf_fuse.imu_params.gyroscope_noise_density");
    accelerometer_random_walk = getParameter<double>(
      node_ptr_, "ekf_fuse.imu_params.accelerometer_random_walk");
    gyroscope_random_walk = getParameter<double>(
      node_ptr_, "ekf_fuse.imu_params.gyroscope_random_walk");

    ekf_wrapper_.set_noise_parameters(
      imu_noise,
      accelerometer_noise_density,
      gyroscope_noise_density,
      accelerometer_random_walk,
      gyroscope_random_walk);

    // Pose parameters
    pose_set_earth_map_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.pose_params.set_earth_map");
    pose_use_message_measurement_covariances_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.pose_params.use_message_measurement_covariances");
    pose_meas_position_cov_[0] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.position.x");
    pose_meas_position_cov_[1] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.position.y");
    pose_meas_position_cov_[2] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.position.z");
    pose_meas_orientation_cov_[0] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.orientation.r");
    pose_meas_orientation_cov_[1] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.orientation.p");
    pose_meas_orientation_cov_[2] = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_params.measurement_covariances.orientation.y");

    // Odometry Parameters
    odom_set_earth_map_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.odom_params.set_earth_map");
    odom_use_message_measurement_covariances_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.odom_params.use_message_measurement_covariances");
    odom_meas_position_cov_[0] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.position.x");
    odom_meas_position_cov_[1] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.position.y");
    odom_meas_position_cov_[2] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.position.z");
    odom_meas_velocity_cov_[0] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.velocity.x");
    odom_meas_velocity_cov_[1] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.velocity.y");
    odom_meas_velocity_cov_[2] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.velocity.z");
    odom_meas_orientation_cov_[0] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.orientation.r");
    odom_meas_orientation_cov_[1] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.orientation.p");
    odom_meas_orientation_cov_[2] = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.measurement_covariances.orientation.y");

    // Apply covariance factors
    apply_multiplicative_factors_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.odom_params.apply_multiplicative_factors");
    odom_x_covariance_factor_ = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.x_covariance_factor");
    odom_y_covariance_factor_ = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.y_covariance_factor");
    odom_z_covariance_factor_ = getParameter<double>(
      node_ptr_, "ekf_fuse.odom_params.z_covariance_factor");

    // Fixed earth to map parameters
    fixed_earth_map_x_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.position.x");
    fixed_earth_map_y_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.position.y");
    fixed_earth_map_z_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.position.z");
    fixed_earth_map_roll_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.orientation.r");
    fixed_earth_map_pitch_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.orientation.p");
    fixed_earth_map_yaw_ = getParameter<double>(
      node_ptr_, "ekf_fuse.fixed_earth_map.orientation.y");

    distance_to_origin_ = getParameter<double>(
      node_ptr_, "ekf_fuse.distance_to_origin");

    // Set map to odom smoothing factor
    map_odom_alpha_ = getParameter<double>(
      node_ptr_, "ekf_fuse.map_odom_alpha");
    if (map_odom_alpha_ < 0.0 || map_odom_alpha_ >= 1.0) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.map_odom_alpha> must be between 0.0 and 1.0 (not included), "
        "using default (0.2)");
      map_odom_alpha_ = 0.2;
    }

    // Set step smoothing parameters
    activate_step_smoothing_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.activate_step_smoothing");
    max_step_ = getParameter<double>(
      node_ptr_, "ekf_fuse.max_step");

    // take_into_account_image_delay parameter
    take_into_account_image_delay_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.take_into_account_image_delay");

    // rotate_covariance_from_odom_to_map parameter
    rotate_covariance_from_odom_to_map_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.rotate_covariance_from_odom_to_map");

    // pose_accumulation_type parameter
    pose_accumulation_type_ = getParameter<std::string>(
      node_ptr_, "ekf_fuse.pose_accumulation.type");
    if (pose_accumulation_type_ != "" && pose_accumulation_type_ != "time" &&
      pose_accumulation_type_ != "number")
    {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.pose_accumulation.type> must be '', 'time' or 'number', "
        "using default ('')");
      pose_accumulation_type_ = "";
    }

    // pose_accumulation_time parameter
    pose_accumulation_time_ = getParameter<double>(
      node_ptr_, "ekf_fuse.pose_accumulation.time");

    // pose_accumulation_number parameter
    pose_accumulation_number_ = getParameter<int>(
      node_ptr_, "ekf_fuse.pose_accumulation.number");

    pose_updater_.setPoseAccumulationParameters(
      pose_accumulation_type_,
      pose_accumulation_time_,
      pose_accumulation_number_);

    std::string debug_ekf_state_pose_topic = "";
    // Debug topic to publish ekf_state
    debug_ekf_state_pose_topic = getParameter<std::string>(
      node_ptr_, "ekf_fuse.debug_ekf_state_pose_topic");
    if (debug_ekf_state_pose_topic != "") {
      debug_ekf_state_pose_pub_ =
        node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        debug_ekf_state_pose_topic, as2_names::topics::self_localization::qos);
    }

    // Debug topic to publish ekf real corrections
    std::string debug_ekf_real_corrections_topic = "";
    debug_ekf_real_corrections_topic = getParameter<std::string>(
      node_ptr_, "ekf_fuse.debug_ekf_real_corrections_topic");
    if (debug_ekf_real_corrections_topic != "") {
      debug_ekf_real_corrections_pub_ =
        node_ptr_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        debug_ekf_real_corrections_topic, as2_names::topics::self_localization::qos);
    }

    // Debug topic to publish ekf state velocity
    std::string debug_ekf_state_velocity_topic = "";
    debug_ekf_state_velocity_topic = getParameter<std::string>(
      node_ptr_, "ekf_fuse.debug_ekf_state_velocity_topic");
    if (debug_ekf_state_velocity_topic != "") {
      debug_ekf_state_velocity_pub_ =
        node_ptr_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        debug_ekf_state_velocity_topic, as2_names::topics::self_localization::qos);
    }

    // Debug topic to publish ekf input odom
    std::string debug_ekf_input_odom_topic = "";
    debug_ekf_input_odom_topic = getParameter<std::string>(
      node_ptr_, "ekf_fuse.debug_ekf_input_odom_topic");
    if (debug_ekf_input_odom_topic != "") {
      debug_ekf_input_odom_pub_ = node_ptr_->create_publisher<nav_msgs::msg::Odometry>(
        debug_ekf_input_odom_topic, as2_names::topics::self_localization::qos);
    }

    // Use IMU to predict
    use_imu_to_predict_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.predict_with_imu");

    // Wait for an update before predicting
    can_predict_ = !getParameter<bool>(
      node_ptr_, "ekf_fuse.wait_for_update_to_predict");

    // Predict with imu topics
    std::vector<std::string> predict_topic_names_;
    if (node_ptr_->has_parameter("ekf_fuse.predict_topics")) {
      node_ptr_->get_parameter<std::vector<std::string>>(
        "ekf_fuse.predict_topics",
        predict_topic_names_);
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Parameter <ekf_fuse.predict_topics> not defined");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Using topics:");
    for (const auto & topic_name : predict_topic_names_) {
      RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
      auto predict_sub = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
        topic_name, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::predict_callback, this, std::placeholders::_1));
      predict_subs_.push_back(predict_sub);
    }

    // Predict with odom topics
    std::vector<std::string> predict_odom_topic_names_;
    if (node_ptr_->has_parameter("ekf_fuse.predict_odom_topics")) {
      node_ptr_->get_parameter<std::vector<std::string>>(
        "ekf_fuse.predict_odom_topics",
        predict_odom_topic_names_);
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Parameter <ekf_fuse.predict_odom_topics> not defined");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Using odom predict topics:");
    for (const auto & topic_name : predict_odom_topic_names_) {
      RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
      auto predict_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::predict_with_odom_callback, this, std::placeholders::_1));
      predict_odom_subs_.push_back(predict_sub);
    }

    // Update pose topics
    std::vector<std::string> update_pose_topic_names_;
    if (node_ptr_->has_parameter("ekf_fuse.update_pose_topics")) {
      node_ptr_->get_parameter<std::vector<std::string>>(
        "ekf_fuse.update_pose_topics",
        update_pose_topic_names_);
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Parameter <ekf_fuse.update_pose_topics> not defined");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Using update pose topics:");
    if (!pose_use_message_measurement_covariances_) {
      for (const auto & topic_name : update_pose_topic_names_) {
        RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
        auto update_sub = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic_name, as2_names::topics::self_localization::qos,
          std::bind(&Plugin::update_pose_callback, this, std::placeholders::_1));
        update_pose_subs_.push_back(update_sub);
      }
    } else {
      for (const auto & topic_name : update_pose_topic_names_) {
        RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
        auto update_sub =
          node_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic_name, as2_names::topics::self_localization::qos,
          std::bind(&Plugin::update_pose_with_covariance_callback, this, std::placeholders::_1));
        update_pose_cov_subs_.push_back(update_sub);
      }
    }

    // Update pose velocity topics
    std::vector<std::string> update_pose_velocity_topic_names_;
    if (node_ptr_->has_parameter("ekf_fuse.update_pose_velocity_topics")) {
      node_ptr_->get_parameter<std::vector<std::string>>(
        "ekf_fuse.update_pose_velocity_topics",
        update_pose_velocity_topic_names_);
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.update_pose_velocity_topics> not defined");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Using update pose velocity topics:");
    for (const auto & topic_name : update_pose_velocity_topic_names_) {
      RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
      auto update_sub = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::update_pose_velocity_callback, this, std::placeholders::_1));
      update_pose_velocity_subs_.push_back(update_sub);
      // If topic does not contain "placeholder", we assume it is a real odometry topic
      // if (topic_name.find("placeholder") == std::string::npos) {
      //   can_predict_ = false; // If we use odometry to update pose and velocity,
      //   we cannot predict with imu
      // }
    }

    // Update pose array topics
    std::vector<std::string> update_pose_array_topic_names_;
    if (node_ptr_->has_parameter("ekf_fuse.update_pose_array_topics")) {
      node_ptr_->get_parameter<std::vector<std::string>>(
        "ekf_fuse.update_pose_array_topics",
        update_pose_array_topic_names_);
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.update_pose_array_topics> not defined");
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Using update pose array topics:");
    for (const auto & topic_name : update_pose_array_topic_names_) {
      RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
      auto update_sub =
        node_ptr_->create_subscription<as2_msgs::msg::PoseWithCovarianceStampedArray>(
        topic_name, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::update_pose_array_callback, this, std::placeholders::_1));
      update_pose_array_subs_.push_back(update_sub);
    }

    // Offboard control topic
    std::string offboard_control_topic = getParameter<std::string>(
      node_ptr_, "ekf_fuse.offboard_control_topic");
    if (offboard_control_topic != "") {
      offboard_control_sub_ =
        node_ptr_->create_subscription<as2_msgs::msg::UInt16MultiArrayStamped>(
        offboard_control_topic, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::offboard_control_callback, this, std::placeholders::_1));
      // If we use offboard control, we cannot predict with imu until enabled
      // can_predict_ = false;
      offboard_activated_ = false;
    }

    // Offboard control channel
    offboard_control_channel_ = getParameter<int>(
      node_ptr_, "ekf_fuse.offboard_control_channel");

    // roll_pitch_fixed parameter
    roll_pitch_fixed_ = getParameter<bool>(
      node_ptr_, "ekf_fuse.roll_pitch_fixed");


    // TF publish rate
    double tf_publish_hz = 100.0;
    if (node_ptr_->has_parameter("ekf_fuse.tf_publish_hz")) {
      tf_publish_hz = node_ptr_->get_parameter("ekf_fuse.tf_publish_hz").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.tf_publish_hz> not defined, using default (100.0)");
    }

    // Timer to publish transforms
    double timer_period_millisec = 1000.0 / tf_publish_hz;
    auto timer_period = std::chrono::milliseconds(static_cast<long long>(timer_period_millisec));
    auto timer_callback = std::bind(&Plugin::timer_callback, this);
    timer_ptr_ = node_ptr_->create_wall_timer(timer_period, timer_callback);

    // pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    //   as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
    //   std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
    // twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
    //   as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos,
    //   std::bind(&Plugin::twist_callback, this, std::placeholders::_1));
  }

  std::vector<as2_state_estimator::TransformInformatonType>
  getTransformationTypesAvailable() const override
  {
    return {as2_state_estimator::TransformInformatonType::EARTH_TO_MAP,
      as2_state_estimator::TransformInformatonType::MAP_TO_ODOM,
      as2_state_estimator::TransformInformatonType::ODOM_TO_BASE,
      as2_state_estimator::TransformInformatonType::TWIST_IN_BASE};
  }

private:
  void generate_map_frame_from_ground_truth_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::PoseWithCovariance earth_to_map;
    earth_to_map.pose = pose.pose;
    state_estimator_interface_->setEarthToMap(earth_to_map, pose.header.stamp, true);
  }

  void offboard_control_callback(
    const as2_msgs::msg::UInt16MultiArrayStamped::SharedPtr msg)
  {
    uint16_t reading = msg->data[offboard_control_channel_];
    if (reading > 1500) {
      offboard_activated_ = true;
    }
  }


  void predict_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double now_dt = node_ptr_->now().seconds();
    if (!can_predict_) {
      if (verbose_) {
        RCLCPP_INFO_ONCE(
          node_ptr_->get_logger(),
          "Cannot predict yet, waiting for first correction");
      }
      return;
    }

    // State and covariance before prediction
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();

    // Prepare IMU input
    ekf::Input imu_input;
    std::array<double, ekf::Input::size> imu_values = {
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z
    };

    if (use_imu_to_predict_) {
      imu_input.set(imu_values);

      // Compute time difference
      double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      double dt = 0.0;
      if (last_imu_t_ > 0.0) {
        dt = msg_time - last_imu_t_;
      } else {
        dt = 0.1;   // default value for the first message
      }
      last_imu_t_ = msg_time;


      // Predict
      ekf_wrapper_.predict(imu_input, dt);

      // Correct covariance with max value
      if (max_covariance_ > 0.0) {
        state = ekf_wrapper_.get_state();
        covariance = ekf_wrapper_.get_state_covariance();
        bool clamped = false;
        for (size_t i = 0; i < ekf::Covariance::size; i++) {
          if (covariance.data[i] > max_covariance_) {
            if (verbose_) {
              RCLCPP_INFO(
                node_ptr_->get_logger(), "IMU PREDICT: Clamping covariance[%zu] from %f to %f",
                i, covariance.data[i], max_covariance_);
            }
            covariance.data[i] = max_covariance_;
            clamped = true;
          }
        }
        if (clamped) {
          ekf_wrapper_.reset(state, covariance);
        }
      }

      last_imu_input_.set(imu_values);

      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "PREDICT. DT = %f", dt);

      //   state = ekf_wrapper_.get_state();
      //   covariance = ekf_wrapper_.get_state_covariance();
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Current state prediction: \n%s", "
      //   "state.to_string().c_str());
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Current covariance prediction: \n%s",
      //               covariance.to_string_diagonal().c_str());
      // }
      // can_update_ = true;
    } else {
      // Save imu input for next odom prediction
      last_imu_input_.set(imu_values);
    }
    double after_dt = node_ptr_->now().seconds();
    double processing_time = after_dt - now_dt;
    if (verbose_) {
      RCLCPP_INFO(
        node_ptr_->get_logger(), "IMU PREDICT processing time: %f seconds", processing_time);
    }
  }


  void predict_with_odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // If earth to map is not set, set it with fixed parameters
    if (!pose_set_earth_map_ && !odom_set_earth_map_ && !earth_to_map_set_) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Neither <ekf_fuse.pose_params.set_earth_map> nor <ekf_fuse.odom_params.set_earth_map> "
        "are true, setting earth to map with fixed parameters");
      T_earth_to_map_ = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          fixed_earth_map_x_,
          fixed_earth_map_y_,
          fixed_earth_map_z_),
        Eigen::Vector3d(
          fixed_earth_map_roll_,
          fixed_earth_map_pitch_,
          fixed_earth_map_yaw_),
        Eigen::Matrix4d::Identity());
      Eigen::Vector<double,
        7> pose_earth_map = ekf::EKFWrapper::transform_to_pose(T_earth_to_map_);
      geometry_msgs::msg::PoseStamped earth_to_map_msg = geometry_msgs::msg::PoseStamped();
      earth_to_map_msg.header.frame_id = "earth";
      earth_to_map_msg.header.stamp = node_ptr_->now();
      earth_to_map_msg.pose.position.x = pose_earth_map[0];
      earth_to_map_msg.pose.position.y = pose_earth_map[1];
      earth_to_map_msg.pose.position.z = pose_earth_map[2];
      earth_to_map_msg.pose.orientation.x = pose_earth_map[3];
      earth_to_map_msg.pose.orientation.y = pose_earth_map[4];
      earth_to_map_msg.pose.orientation.z = pose_earth_map[5];
      earth_to_map_msg.pose.orientation.w = pose_earth_map[6];
      generate_map_frame_from_ground_truth_pose(earth_to_map_msg);
      earth_to_map_set_ = true;
    }

    if (use_gazebo_) {
      // Get map to odom transform
      Eigen::Matrix4d T_map_odom = ekf_wrapper_.get_map_to_odom();
      // Compute earth to odom
      // Eigen::Matrix4d T_earth_odom = T_map_odom * T_earth_to_map_;
      Eigen::Matrix4d T_earth_odom = T_earth_to_map_ * T_map_odom;
      // Transform pose from earth to odom
      Eigen::Matrix4d T_odom_base = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z),
        EKFFuseUtils::quaternionToEuler(
          Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z)),
        T_earth_odom);
      Eigen::Vector<double, 7> pose_odom_base = ekf::EKFWrapper::transform_to_pose(T_odom_base);
      // Update pose_msg with the new values
      msg->pose.pose.position.x = pose_odom_base[0];
      msg->pose.pose.position.y = pose_odom_base[1];
      msg->pose.pose.position.z = pose_odom_base[2];
      Eigen::Quaterniond q_odom(
        pose_odom_base[6],
        pose_odom_base[3],
        pose_odom_base[4],
        pose_odom_base[5]);
      q_odom.normalize();
      msg->pose.pose.orientation.w = q_odom.w();
      msg->pose.pose.orientation.x = q_odom.x();
      msg->pose.pose.orientation.y = q_odom.y();
      msg->pose.pose.orientation.z = q_odom.z();
    }


    // State and covariance before prediction
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();

    // Store velocity before any prediction
    ekf::VelocityMeasurement last_velocity;
    last_velocity.data[ekf::VelocityMeasurement::VX] = state.data[ekf::State::VX];
    last_velocity.data[ekf::VelocityMeasurement::VY] = state.data[ekf::State::VY];
    last_velocity.data[ekf::VelocityMeasurement::VZ] = state.data[ekf::State::VZ];

    // Compute time difference
    double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt = 0.0;
    if (last_imu_t_ > 0.0) {
      dt = msg_time - last_imu_t_;
    } else {
      dt = 0.1;   // default value for the first message
    }
    last_imu_t_ = msg_time;

    // Predict with last imu input to get new covariance
    if (!use_imu_to_predict_) {
      ekf_wrapper_.predict(last_imu_input_, dt);
    }


    // Prepare the odometry with covariances
    // Odometry pose
    // geometry_msgs::msg::PoseStamped::SharedPtr pose_msg =
    //   std::make_shared<geometry_msgs::msg::PoseStamped>();
    // pose_msg->header = msg->header;
    // pose_msg->pose = msg->pose.pose;
    std::array<double, 6> pose_covariance_diagonal;

    // if (odom_use_message_measurement_covariances_) {
    //   for (size_t i = 0; i < 6; i++) {
    //     pose_covariance_diagonal[i] = msg->pose.covariance[i * 6 + i];
    //   }
    // } else {
    //   pose_covariance_diagonal = {
    //     odom_meas_position_cov_, odom_meas_position_cov_, odom_meas_position_cov_,
    //     odom_meas_orientation_cov_, odom_meas_orientation_cov_, odom_meas_orientation_cov_
    //   };
    // }
    for (size_t i = 0; i < 6; i++) {
      pose_covariance_diagonal[i] = msg->pose.covariance[i * 6 + i];
    }

    // Odometry twist
    // geometry_msgs::msg::TwistStamped::SharedPtr twist_msg =
    //   std::make_shared<geometry_msgs::msg::TwistStamped>();
    // twist_msg->header = msg->header;
    // twist_msg->twist = msg->twist.twist;
    std::array<double, 6> twist_covariance_diagonal;
    if (odom_use_message_measurement_covariances_) {
      for (size_t i = 0; i < 6; i++) {
        twist_covariance_diagonal[i] = msg->twist.covariance[i * 6 + i];
      }
    } else {
      twist_covariance_diagonal = {
        odom_meas_velocity_cov_[0], odom_meas_velocity_cov_[1], odom_meas_velocity_cov_[2],
        0.0, 0.0, 0.0
      };
    }

    // Prepare velocity from odometry
    ekf::VelocityMeasurement velocity_odom;
    // Get odom-base_link transform
    // Convert quaternion to euler
    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    // pose_msg->pose.orientation.w,
    // pose_msg->pose.orientation.x,
    // pose_msg->pose.orientation.y,
    // pose_msg->pose.orientation.z);
    q.normalize();
    // Get odom-base_link transform in euler to compute map-base_link
    Eigen::Vector3d euler_odom_base = EKFFuseUtils::quaternionToEuler(q);
    Eigen::Vector3d pos_odom_base = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    // pose_msg->pose.position.x,
    // pose_msg->pose.position.y,
    // pose_msg->pose.position.z);
    Eigen::Vector3d ori_odom_base = Eigen::Vector3d(
      euler_odom_base[0],
      euler_odom_base[1],
      euler_odom_base[2]);

    // Get map to odom transform
    Eigen::Matrix4d T_map_odom = ekf_wrapper_.get_map_to_odom();

    // Transform from odom-base_link to map-base_link
    Eigen::Matrix4d T_map_base = ekf_wrapper_.get_T_a_c(
      pos_odom_base,
      ori_odom_base,
      T_map_odom);

    Eigen::Vector<double, 7> pose_map_base = ekf::EKFWrapper::transform_to_pose(T_map_base);

    Eigen::Vector3d euler_map_base = EKFFuseUtils::quaternionToEuler(
      Eigen::Quaterniond(
        pose_map_base[6],
        pose_map_base[3],
        pose_map_base[4],
        pose_map_base[5]));

    // Get state
    state = ekf_wrapper_.get_state();
    covariance = ekf_wrapper_.get_state_covariance();
    // Correct angles to be near the current state
    std::array<double, 3> state_euler_array = {
      state.data[ekf::State::ROLL],
      state.data[ekf::State::PITCH],
      state.data[ekf::State::YAW]
    };
    Eigen::Vector3d corrected_euler_map_base = EKFFuseUtils::correct_measured_euler(
      Eigen::Vector3d(
        state_euler_array[0],
        state_euler_array[1],
        state_euler_array[2]),
      euler_map_base);

    // Correct velocity to be in map frame
    // Rotate from base_link to map frame
    Eigen::Matrix3d R_map_base = T_map_base.block<3, 3>(0, 0);
    Eigen::Vector3d vel_base = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    // twist_msg->twist.linear.x,
    // twist_msg->twist.linear.y,
    // twist_msg->twist.linear.z);
    Eigen::Vector3d vel_map_base = R_map_base * vel_base;

    std::array<double, ekf::VelocityMeasurement::size> velocity_values = {
      vel_map_base[0],   // vx
      vel_map_base[1],   // vy
      vel_map_base[2]   // vz
    };
    velocity_odom.set(velocity_values);

    // Prepare pose Covariance
    ekf::VelocityMeasurementCovariance velocity_cov;
    std::array<double, ekf::VelocityMeasurementCovariance::size> velocity_cov_values;
    velocity_cov_values[0] = twist_covariance_diagonal[0];   // vx
    velocity_cov_values[1] = twist_covariance_diagonal[1];   // vy
    velocity_cov_values[2] = twist_covariance_diagonal[2];   // vz
    velocity_cov.set(velocity_cov_values);

    ekf::PoseMeasurement odometry_pose_in_map;
    odometry_pose_in_map.data[0] = pose_map_base[0];
    odometry_pose_in_map.data[1] = pose_map_base[1];
    odometry_pose_in_map.data[2] = pose_map_base[2];
    odometry_pose_in_map.data[3] = corrected_euler_map_base[0];
    odometry_pose_in_map.data[4] = corrected_euler_map_base[1];
    odometry_pose_in_map.data[5] = corrected_euler_map_base[2];
    ekf::PoseMeasurementCovariance measurement_noise_covariance;
    measurement_noise_covariance.data[0] = pose_covariance_diagonal[0];
    measurement_noise_covariance.data[1] = pose_covariance_diagonal[1];
    measurement_noise_covariance.data[2] = pose_covariance_diagonal[2];
    measurement_noise_covariance.data[3] = pose_covariance_diagonal[3];
    measurement_noise_covariance.data[4] = pose_covariance_diagonal[4];
    measurement_noise_covariance.data[5] = pose_covariance_diagonal[5];

    // Compute the innovation (difference between last odom and current odom)
    ekf::PoseMeasurement new_state_odom_innovation;
    // For position, add the difference between last and current odom to the current state
    // new_state_odom_innovation.data[0] = state.data[ekf::State::X] +
    //   (odometry_pose_in_map.data[0] - last_odometry_pose_.data[0]);
    // new_state_odom_innovation.data[1] = state.data[ekf::State::Y] +
    //   (odometry_pose_in_map.data[1] - last_odometry_pose_.data[1]);
    // new_state_odom_innovation.data[2] = state.data[ekf::State::Z] +
    //   (odometry_pose_in_map.data[2] - last_odometry_pose_.data[2]);
    new_state_odom_innovation.data[0] = odometry_pose_in_map.data[0];
    new_state_odom_innovation.data[1] = odometry_pose_in_map.data[1];
    new_state_odom_innovation.data[2] = odometry_pose_in_map.data[2];
    // For orientation, keep the same as current odom.
    new_state_odom_innovation.data[3] = odometry_pose_in_map.data[3];
    new_state_odom_innovation.data[4] = odometry_pose_in_map.data[4];
    new_state_odom_innovation.data[5] = odometry_pose_in_map.data[5];
    // For velocity, do the same as pose (adding the difference)
    // new_state_odom_innovation.data[6] = last_velocity.data[ekf::VelocityMeasurement::VX] +
    //   (velocity_odom.data[ekf::VelocityMeasurement::VX] -
    //   last_odometry_velocity_.data[ekf::VelocityMeasurement::VX]);
    // new_state_odom_innovation.data[7] = last_velocity.data[ekf::VelocityMeasurement::VY] +
    //   (velocity_odom.data[ekf::VelocityMeasurement::VY] -
    //   last_odometry_velocity_.data[ekf::VelocityMeasurement::VY]);
    // new_state_odom_innovation.data[8] = last_velocity.data[ekf::VelocityMeasurement::VZ] +
    //   (velocity_odom.data[ekf::VelocityMeasurement::VZ] -
    //   last_odometry_velocity_.data[ekf::VelocityMeasurement::VZ]);
    // For velocity, just use the one from odom
    new_state_odom_innovation.data[6] = velocity_odom.data[ekf::VelocityMeasurement::VX];
    new_state_odom_innovation.data[7] = velocity_odom.data[ekf::VelocityMeasurement::VY];
    new_state_odom_innovation.data[8] = velocity_odom.data[ekf::VelocityMeasurement::VZ];

    // Change state with odom values but keep covariance from prediction
    ekf::State odometry_state = state;
    odometry_state.data[ekf::State::X] = new_state_odom_innovation.data[0];
    odometry_state.data[ekf::State::Y] = new_state_odom_innovation.data[1];
    odometry_state.data[ekf::State::Z] = new_state_odom_innovation.data[2];
    odometry_state.data[ekf::State::ROLL] = new_state_odom_innovation.data[3];
    odometry_state.data[ekf::State::PITCH] = new_state_odom_innovation.data[4];
    odometry_state.data[ekf::State::YAW] = new_state_odom_innovation.data[5];
    odometry_state.data[ekf::State::VX] = new_state_odom_innovation.data[6];
    odometry_state.data[ekf::State::VY] = new_state_odom_innovation.data[7];
    odometry_state.data[ekf::State::VZ] = new_state_odom_innovation.data[8];
    // // Set position, orientation and velocity from odometry
    // odometry_state.data[ekf::State::X] = odometry_pose_in_map.data[0];
    // odometry_state.data[ekf::State::Y] = odometry_pose_in_map.data[1];
    // odometry_state.data[ekf::State::Z] = odometry_pose_in_map.data[2];
    // odometry_state.data[ekf::State::ROLL] = odometry_pose_in_map.data[3];
    // odometry_state.data[ekf::State::PITCH] = odometry_pose_in_map.data[4];
    // odometry_state.data[ekf::State::YAW] = odometry_pose_in_map.data[5];
    // odometry_state.data[ekf::State::VX] = velocity_odom.data[0];
    // odometry_state.data[ekf::State::VY] = velocity_odom.data[1];
    // odometry_state.data[ekf::State::VZ] = velocity_odom.data[2];
    // // Keep velocities from prediction as odometry velocities have error
    // odometry_state.data[ekf::State::VX] = state.data[ekf::State::VX];
    // odometry_state.data[ekf::State::VY] = state.data[ekf::State::VY];
    // odometry_state.data[ekf::State::VZ] = state.data[ekf::State::VZ];
    ekf_wrapper_.set_state(odometry_state);

    // Save last odometry pose and velocity
    last_odometry_pose_.set(odometry_pose_in_map.data);
    last_odometry_velocity_.set(velocity_odom.data);


    // Correct covariance with max value
    if (max_covariance_ > 0.0) {
      state = ekf_wrapper_.get_state();
      covariance = ekf_wrapper_.get_state_covariance();
      bool clamped = false;
      for (size_t i = 0; i < ekf::Covariance::size; i++) {
        if (covariance.data[i] > max_covariance_) {
          // if (verbose_) {
          //   RCLCPP_INFO(node_ptr_->get_logger(), "ODOM PREDICT: Clamping covariance[%zu]
          // from %f to %f",
          //               i, covariance.data[i], max_covariance_);
          // }
          covariance.data[i] = max_covariance_;
          clamped = true;
        }
      }
      if (clamped) {
        ekf_wrapper_.reset(state, covariance);
      }
    }

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "PREDICT. DT = %f", dt);

    //   state = ekf_wrapper_.get_state();
    //   covariance = ekf_wrapper_.get_state_covariance();
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current state prediction: \n%s", "
    //   "state.to_string().c_str());
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current covariance prediction: \n%s",
    //               covariance.to_string_diagonal().c_str());
    // }

    can_update_ = true;
  }


  void update_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!earth_to_map_set_ && pose_set_earth_map_) {
      // As earth to map is not set, we assume that the first pose received is in the earth
      generate_map_frame_from_ground_truth_pose(*msg);
      tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
      if (verbose_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map from ground truth");
        RCLCPP_INFO(
          node_ptr_->get_logger(), "Earth to map value (from state estimator): %f, %f, %f",
          earth_map_tf.getOrigin().x(), earth_map_tf.getOrigin().y(), earth_map_tf.getOrigin().z());
      }
      // Convert orientation to euler
      Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = EKFFuseUtils::quaternionToEuler(q);

      // Set T_earth_to_map matrix
      T_earth_to_map_ = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          earth_map_tf.getOrigin().x(),
          earth_map_tf.getOrigin().y(),
          earth_map_tf.getOrigin().z()),
        Eigen::Vector3d(
          euler[0],
          euler[1],
          euler[2]),
        Eigen::Matrix4d::Identity());
      earth_to_map_set_ = true;
      return;
    }
    if (!can_update_) {
      // Skip this update to avoid updating too fast
      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Skipping update to avoid updating too fast");
      // }
      return;
    }
    // Prepare pose measurement
    ekf::PoseMeasurement pose_meas;
    Eigen::Quaterniond q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);
    q.normalize();
    auto euler = EKFFuseUtils::quaternionToEuler(q);
    Eigen::Vector3d pos_earth_base = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
    Eigen::Vector3d ori_earth_base = Eigen::Vector3d(
      euler[0],
      euler[1],
      euler[2]);

    // Transform from earth-base_link to map-base_link
    Eigen::Matrix4d T_map_base = ekf_wrapper_.get_T_b_c(
      pos_earth_base,
      ori_earth_base,
      T_earth_to_map_);

    Eigen::Vector<double, 7> pose_map_base = ekf::EKFWrapper::transform_to_pose(T_map_base);

    Eigen::Vector3d euler_map_base = EKFFuseUtils::quaternionToEuler(
      Eigen::Quaterniond(
        pose_map_base[6],
        pose_map_base[3],
        pose_map_base[4],
        pose_map_base[5]));

    // Get state
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();
    // Correct angles to be near the current state
    std::array<double, 3> state_euler_array = {
      state.data[ekf::State::ROLL],
      state.data[ekf::State::PITCH],
      state.data[ekf::State::YAW]
    };
    Eigen::Vector3d corrected_euler_map_base = EKFFuseUtils::correct_measured_euler(
      Eigen::Vector3d(
        state_euler_array[0],
        state_euler_array[1],
        state_euler_array[2]),
      euler_map_base);

    std::array<double, ekf::PoseMeasurement::size> pose_values = {
      pose_map_base[0],
      pose_map_base[1],
      pose_map_base[2],
      corrected_euler_map_base[0],   // roll
      corrected_euler_map_base[1],   // pitch
      corrected_euler_map_base[2]   // yaw
    };
    pose_meas.set(pose_values);

    // Prepare pose Covariance
    // TODO(rdasilva01): set covariance from message
    ekf::PoseMeasurementCovariance pose_cov;
    std::array<double, ekf::PoseMeasurementCovariance::size> pose_cov_values = {
      pose_meas_position_cov_[0], pose_meas_position_cov_[1], pose_meas_position_cov_[2],
      pose_meas_orientation_cov_[0], pose_meas_orientation_cov_[1], pose_meas_orientation_cov_[2]
    };
    pose_cov.set(pose_cov_values);

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE POSE: \n%s", pose_meas.to_string().c_str());
    // }
    // Update
    ekf_wrapper_.update_pose(
      pose_meas,
      pose_cov
    );

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE");
    //   state = ekf_wrapper_.get_state();
    //   covariance = ekf_wrapper_.get_state_covariance();
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current state update: \n%s",
    //      state.to_string().c_str());
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current covariance update: \n%s",
    //              covariance.to_string_diagonal().c_str());
    // }
    // can_update_ = false;
    can_predict_ = true;
  }


  void update_pose_with_covariance_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (verbose_) {
      // Print stamp difference
      rclcpp::Time current_time = node_ptr_->now();
      rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
      double time_diff = (current_time - msg_time).seconds();
      RCLCPP_INFO_THROTTLE(
        node_ptr_->get_logger(),
        *node_ptr_->get_clock(),
        2000,
        "Pose with covariance time difference: %f seconds [%f - %f]",
        time_diff,
        current_time.seconds(),
        msg_time.seconds());
    }
    // ------------------------------------------------------------
    // Initialization, set earth to map if needed
    // ------------------------------------------------------------
    if (!earth_to_map_set_ && pose_set_earth_map_) {
      // As earth to map is not set, we assume that the first pose received is in the earth
      geometry_msgs::msg::PoseStamped pose_stamped_msg;
      pose_stamped_msg.header = msg->header;
      pose_stamped_msg.pose = msg->pose.pose;
      generate_map_frame_from_ground_truth_pose(pose_stamped_msg);
      tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
      if (verbose_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map from ground truth");
        RCLCPP_INFO(
          node_ptr_->get_logger(), "Earth to map value (from state estimator): %f, %f, %f",
          earth_map_tf.getOrigin().x(), earth_map_tf.getOrigin().y(), earth_map_tf.getOrigin().z());
      }
      // Convert orientation to euler
      Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = EKFFuseUtils::quaternionToEuler(q);

      // Set T_earth_to_map matrix
      T_earth_to_map_ = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          earth_map_tf.getOrigin().x(),
          earth_map_tf.getOrigin().y(),
          earth_map_tf.getOrigin().z()),
        Eigen::Vector3d(
          euler[0],
          euler[1],
          euler[2]),
        Eigen::Matrix4d::Identity());
      earth_to_map_set_ = true;
      return;
    }

    // ------------------------------------------------------------
    // Check if can update
    // ------------------------------------------------------------
    if (!can_update_) {
      // Skip this update to avoid updating too fast
      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Skipping update to avoid updating too fast");
      // }
      return;
    }

    // ------------------------------------------------------------
    // Prepare pose measurement
    // ------------------------------------------------------------
    auto result = pose_updater_.initialPreparePose(msg);
    Eigen::Matrix4d corr_T_map_oldBase = result.first;
    std::array<double, 6> pose_covariance_diagonal = result.second;

    // ------------------------------------------------------------
    // Get state and state covariance
    // ------------------------------------------------------------
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();

    // ------------------------------------------------------------
    // Get T_map_base considering image delay if needed
    // ------------------------------------------------------------
    Eigen::Matrix4d T_map_base;
    if (take_into_account_image_delay_) {
      Eigen::Matrix4d T_oldBase_lastBase = pose_updater_.getMovementIncrementFromImageDelay(
        msg->header,
        state);
      // Update T_map_base with the increment
      T_map_base = corr_T_map_oldBase * T_oldBase_lastBase;
    } else {
      // No delay, so T_map_base is the corrected one
      T_map_base = corr_T_map_oldBase;
    }

    // ------------------------------------------------------------
    // Prepare PoseMeasurement from T_map_base to update
    // ------------------------------------------------------------
    ekf::PoseMeasurement pose_meas = pose_updater_.preparePoseMeasurement(
      T_map_base,
      state,
      roll_pitch_fixed_);

    // ------------------------------------------------------------
    // Prepare PoseMeasurementCovariance to update
    // ------------------------------------------------------------
    ekf::PoseMeasurementCovariance pose_cov;
    std::array<double, ekf::PoseMeasurementCovariance::size> pose_cov_values = {
      pose_covariance_diagonal[0], pose_covariance_diagonal[1], pose_covariance_diagonal[2],
      pose_covariance_diagonal[3], pose_covariance_diagonal[4], pose_covariance_diagonal[5]
    };
    pose_cov.set(pose_cov_values);

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE POSE: \n%s", pose_meas.to_string().c_str());
    // RCLCPP_INFO(node_ptr_->get_logger(), "With covariance: \n%s", pose_cov.to_string().c_str());
    // }

    // ------------------------------------------------------------
    // Update logic with accumulation if needed
    // ------------------------------------------------------------
    auto [update_with_accumulated, pose_meas_response, pose_cov_response] =
      pose_updater_.processPoseMeasurement(pose_meas, pose_cov, msg->header, state);
    pose_meas = pose_meas_response;
    pose_cov = pose_cov_response;


    // ------------------------------------------------------------
    // Update EKF
    // ------------------------------------------------------------
    if (update_with_accumulated) {
      if (first_distance_check_) {
        // Compute distance between current state and map
        ekf::State state = ekf_wrapper_.get_state();
        double x = state.data[ekf::State::X];
        double y = state.data[ekf::State::Y];
        double z = state.data[ekf::State::Z];
        double distance_to_map = std::sqrt(x * x + y * y + z * z);
        if (verbose_) {
          // RCLCPP_WARN(
          //   node_ptr_->get_logger(),
          //   "First distance to map check: %f m (distance_to_origin in params: %f m)",
          //   distance_to_map,
          //   distance_to_origin_);
          RCLCPP_WARN_THROTTLE(
            node_ptr_->get_logger(),
            *node_ptr_->get_clock(),
            2000,
            "First distance to map check: %f m (distance_to_origin in params: %f m)",
            distance_to_map,
            distance_to_origin_);
        }
        if (distance_to_map > distance_to_origin_) {
          RCLCPP_WARN(
            node_ptr_->get_logger(),
            "Distance to map (%f m) is greater than distance_to_origin (%f m). "
            "Now updating EKF with poses.",
            distance_to_map,
            distance_to_origin_);
          first_distance_check_ = false;
        }
      } else {
        ekf_wrapper_.update_pose(
          pose_meas,
          pose_cov
        );

        pose_updater_.clearAccumulatedPoses();

        // Publish debug correction message
        if (debug_ekf_real_corrections_pub_) {
          Eigen::Matrix4d map_to_pose_eigen =
            ekf_wrapper_.get_T_b_c(
            Eigen::Vector3d(pose_meas.data[0], pose_meas.data[1], pose_meas.data[2]),
            Eigen::Vector3d(pose_meas.data[3], pose_meas.data[4], pose_meas.data[5]),
            Eigen::Matrix4d::Identity());
          Eigen::Vector<double, 7> map_to_pose_values =
            ekf::EKFWrapper::transform_to_pose(map_to_pose_eigen);
          geometry_msgs::msg::PoseWithCovarianceStamped correction_msg;
          correction_msg.header = msg->header;
          correction_msg.header.stamp = node_ptr_->now();
          correction_msg.header.frame_id = "drone0/map";
          correction_msg.pose.pose.position.x = map_to_pose_values[0];
          correction_msg.pose.pose.position.y = map_to_pose_values[1];
          correction_msg.pose.pose.position.z = map_to_pose_values[2];
          correction_msg.pose.pose.orientation.x = map_to_pose_values[3];
          correction_msg.pose.pose.orientation.y = map_to_pose_values[4];
          correction_msg.pose.pose.orientation.z = map_to_pose_values[5];
          correction_msg.pose.pose.orientation.w = map_to_pose_values[6];
          for (size_t i = 0; i < 36; i++) {
            correction_msg.pose.covariance[i] = 0.0;
          }
          for (size_t i = 0; i < 6; i++) {
            correction_msg.pose.covariance[i * 6 + i] = pose_covariance_diagonal[i];
          }
          debug_ekf_real_corrections_pub_->publish(correction_msg);
        }
      }
    }

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE");
    //   state = ekf_wrapper_.get_state();
    //   covariance = ekf_wrapper_.get_state_covariance();
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current state update: \n%s",
    //    state.to_string().c_str());
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current covariance update: \n%s",
    //              covariance.to_string_diagonal().c_str());
    // }
    // can_update_ = false;
    // can_predict_ = true;
  }


  void update_pose_array_callback(
    const as2_msgs::msg::PoseWithCovarianceStampedArray::SharedPtr msg)
  {
    if (verbose_) {
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Received pose array with %zu poses.",
        msg->poses.size());
    }
    for (size_t i = 0; i < msg->poses.size(); i++) {
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(msg->poses[i]);
      update_pose_with_covariance_callback(pose_msg);
    }
  }


  void update_pose_velocity_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Odometry pose
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg =
      std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose = msg->pose.pose;
    std::array<double, 6> pose_covariance_diagonal;
    if (odom_use_message_measurement_covariances_) {
      for (size_t i = 0; i < 6; i++) {
        pose_covariance_diagonal[i] = msg->pose.covariance[i * 6 + i];
      }
    } else {
      pose_covariance_diagonal = {
        odom_meas_position_cov_[0], odom_meas_position_cov_[1], odom_meas_position_cov_[2],
        odom_meas_orientation_cov_[0], odom_meas_orientation_cov_[1], odom_meas_orientation_cov_[2]
      };
    }
    // Odometry twist
    geometry_msgs::msg::TwistStamped::SharedPtr twist_msg =
      std::make_shared<geometry_msgs::msg::TwistStamped>();
    twist_msg->header = msg->header;
    twist_msg->twist = msg->twist.twist;
    std::array<double, 6> twist_covariance_diagonal;
    if (odom_use_message_measurement_covariances_) {
      for (size_t i = 0; i < 6; i++) {
        twist_covariance_diagonal[i] = msg->twist.covariance[i * 6 + i];
      }
    } else {
      twist_covariance_diagonal = {
        odom_meas_velocity_cov_[0], odom_meas_velocity_cov_[1], odom_meas_velocity_cov_[2],
        0.0, 0.0, 0.0
      };
    }
    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Received odometry message with:");
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Pose:");
    //   RCLCPP_INFO(node_ptr_->get_logger(), " Position: %f, %f, %f",
    //               pose_msg->pose.position.x, pose_msg->pose.position.y,
    //               pose_msg->pose.position.z);
    //   RCLCPP_INFO(node_ptr_->get_logger(), " Orientation (quaternion): %f, %f, %f, %f",
    //               pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
    //               pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Twist:");
    //   RCLCPP_INFO(node_ptr_->get_logger(), " Linear: %f, %f, %f",
    //               twist_msg->twist.linear.x, twist_msg->twist.linear.y,
    //               twist_msg->twist.linear.z);
    //   RCLCPP_INFO(node_ptr_->get_logger(), " Angular: %f, %f, %f",
    //               twist_msg->twist.angular.x, twist_msg->twist.angular.y,
    //               twist_msg->twist.angular.z);
    // }
    // return;
    // Set earth-map if not already set
    if (!earth_to_map_set_ && odom_set_earth_map_) {
      // As earth to map is not set, we assume that the first pose received is in the earth
      // As it is odometry, we assume that the first pose is at 0,0,0 with 0,0,0 euler
      geometry_msgs::msg::PoseStamped::SharedPtr zero_pose =
        std::make_shared<geometry_msgs::msg::PoseStamped>();
      zero_pose->header = pose_msg->header;
      if (use_gazebo_) {
        zero_pose->pose = pose_msg->pose;
      } else {
        zero_pose->pose.position.x = 0.0;
        zero_pose->pose.position.y = 0.0;
        zero_pose->pose.position.z = 0.0;
        zero_pose->pose.orientation.w = 1.0;
        zero_pose->pose.orientation.x = 0.0;
        zero_pose->pose.orientation.y = 0.0;
        zero_pose->pose.orientation.z = 0.0;
      }
      generate_map_frame_from_ground_truth_pose(*zero_pose);
      tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
      if (verbose_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Setting earth to map from ground truth");
        RCLCPP_INFO(
          node_ptr_->get_logger(), "Earth to map value (from state estimator): %f, %f, %f",
          earth_map_tf.getOrigin().x(), earth_map_tf.getOrigin().y(), earth_map_tf.getOrigin().z());
      }
      // Convert orientation to euler
      Eigen::Quaterniond q(
        pose_msg->pose.orientation.w,
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = EKFFuseUtils::quaternionToEuler(q);

      // Set T_earth_to_map matrix
      T_earth_to_map_ = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          earth_map_tf.getOrigin().x(),
          earth_map_tf.getOrigin().y(),
          earth_map_tf.getOrigin().z()),
        Eigen::Vector3d(
          euler[0],
          euler[1],
          euler[2]),
        Eigen::Matrix4d::Identity());
      earth_to_map_set_ = true;
      return;
    }

    if (!can_update_) {
      // Skip this update to avoid updating too fast
      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Skipping update to avoid updating too fast");
      // }
      return;
    }

    // If gazebo is used, the odometry is in earth frame, so we need to convert it to odom frame
    if (use_gazebo_) {
      // Get map to odom transform
      Eigen::Matrix4d T_map_odom = ekf_wrapper_.get_map_to_odom();
      // Compute earth to odom
      // Eigen::Matrix4d T_earth_odom = T_map_odom * T_earth_to_map_;
      Eigen::Matrix4d T_earth_odom = T_earth_to_map_ * T_map_odom;
      // Transform pose from earth to odom
      Eigen::Matrix4d T_odom_base = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          pose_msg->pose.position.x,
          pose_msg->pose.position.y,
          pose_msg->pose.position.z),
        EKFFuseUtils::quaternionToEuler(
          Eigen::Quaterniond(
            pose_msg->pose.orientation.w,
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z)),
        T_earth_odom);
      Eigen::Vector<double, 7> pose_odom_base = ekf::EKFWrapper::transform_to_pose(T_odom_base);
      // Update pose_msg with the new values
      pose_msg->pose.position.x = pose_odom_base[0];
      pose_msg->pose.position.y = pose_odom_base[1];
      pose_msg->pose.position.z = pose_odom_base[2];
      Eigen::Quaterniond q_odom(
        pose_odom_base[6],
        pose_odom_base[3],
        pose_odom_base[4],
        pose_odom_base[5]);
      q_odom.normalize();
      pose_msg->pose.orientation.w = q_odom.w();
      pose_msg->pose.orientation.x = q_odom.x();
      pose_msg->pose.orientation.y = q_odom.y();
      pose_msg->pose.orientation.z = q_odom.z();

      // if (verbose_) {
      //   std::ostringstream oss1, oss2;
      //   oss1 << T_earth_odom;
      //   oss2 << T_map_odom;
      //   RCLCPP_INFO(node_ptr_->get_logger(), "T_earth_odom: \n%s", oss1.str().c_str());
      //   RCLCPP_INFO(node_ptr_->get_logger(), "T_map_odom: \n%s", oss2.str().c_str());
      // }
    }

    // TODO(rdasilva01): NOW DO NOT USE VELOCITY
    // // Prepare velocity measurement
    // ekf::VelocityMeasurement velocity_meas;

    // Get pose and orientation
    Eigen::Quaterniond q(
      pose_msg->pose.orientation.w,
      pose_msg->pose.orientation.x,
      pose_msg->pose.orientation.y,
      pose_msg->pose.orientation.z);
    q.normalize();
    auto euler_odom_base = EKFFuseUtils::quaternionToEuler(q);
    Eigen::Vector3d pos_odom_base = Eigen::Vector3d(
      pose_msg->pose.position.x,
      pose_msg->pose.position.y,
      pose_msg->pose.position.z);
    Eigen::Vector3d ori_odom_base = Eigen::Vector3d(
      euler_odom_base[0],
      euler_odom_base[1],
      euler_odom_base[2]);

    // Get map to odom transform
    Eigen::Matrix4d T_map_odom = ekf_wrapper_.get_map_to_odom();

    // TODO(rdasilva01): FIX
    // Transform from odom-base_link to map-base_link
    Eigen::Matrix4d T_map_base = ekf_wrapper_.get_T_a_c(
      pos_odom_base,
      ori_odom_base,
      T_map_odom);
    // // Transform from earth-base_link to map-base_link
    // Eigen::Matrix4d T_map_base = ekf_wrapper_.get_T_b_c(
    //   pos_odom_base,
    //   ori_odom_base,
    //   T_earth_to_map_);

    Eigen::Vector<double, 7> pose_map_base = ekf::EKFWrapper::transform_to_pose(T_map_base);

    Eigen::Vector3d euler_map_base = EKFFuseUtils::quaternionToEuler(
      Eigen::Quaterniond(
        pose_map_base[6],
        pose_map_base[3],
        pose_map_base[4],
        pose_map_base[5]));

    // Get state
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();
    // Correct angles to be near the current state
    std::array<double, 3> state_euler_array = {
      state.data[ekf::State::ROLL],
      state.data[ekf::State::PITCH],
      state.data[ekf::State::YAW]
    };
    Eigen::Vector3d corrected_euler_map_base = EKFFuseUtils::correct_measured_euler(
      Eigen::Vector3d(
        state_euler_array[0],
        state_euler_array[1],
        state_euler_array[2]),
      euler_map_base);

    if (apply_multiplicative_factors_) {
      if (odom_use_message_measurement_covariances_) {
        // Rotate covariance from odom to base_link
        Eigen::Matrix3d R_base_odom = ekf_wrapper_.get_T_a_c(
          pos_odom_base,
          ori_odom_base,
          Eigen::Matrix4d::Identity()).block<3, 3>(0, 0).transpose();
        // Extract the full 3x3 position covariance submatrix from the 6x6 pose covariance
        Eigen::Matrix3d pos_cov_odom = Eigen::Matrix3d::Zero();
        pos_cov_odom(0, 0) = pose_covariance_diagonal[0];
        pos_cov_odom(1, 1) = pose_covariance_diagonal[1];
        pos_cov_odom(2, 2) = pose_covariance_diagonal[2];
        // Transform: Σ_base = R_base_odom * Σ_odom * R_base_odom^T
        Eigen::Matrix3d pos_cov_base = R_base_odom * pos_cov_odom * R_base_odom.transpose();
        // Extract the full 3x3 orientation covariance submatrix
        Eigen::Matrix3d ori_cov_odom = Eigen::Matrix3d::Zero();
        ori_cov_odom(0, 0) = pose_covariance_diagonal[3];
        ori_cov_odom(1, 1) = pose_covariance_diagonal[4];
        ori_cov_odom(2, 2) = pose_covariance_diagonal[5];
        // Transform orientation covariance
        Eigen::Matrix3d ori_cov_base = R_base_odom * ori_cov_odom * R_base_odom.transpose();
        // Update pose_covariance_diagonal with transformed values
        pose_covariance_diagonal[0] = pos_cov_base(0, 0);
        pose_covariance_diagonal[1] = pos_cov_base(1, 1);
        pose_covariance_diagonal[2] = pos_cov_base(2, 2);
        pose_covariance_diagonal[3] = ori_cov_base(0, 0);
        pose_covariance_diagonal[4] = ori_cov_base(1, 1);
        pose_covariance_diagonal[5] = ori_cov_base(2, 2);
      }

      // Apply multiplier to covariance in base_link
      pose_covariance_diagonal[0] = pose_covariance_diagonal[0] * odom_x_covariance_factor_;
      pose_covariance_diagonal[1] = pose_covariance_diagonal[1] * odom_y_covariance_factor_;
      pose_covariance_diagonal[2] = pose_covariance_diagonal[2] * odom_z_covariance_factor_;

      // if (!odom_use_message_measurement_covariances_) {
      // // If we set a fixed covariance, the covariance is now in base_link frame,
      // // so we need to transform it to odom frame

      // Now transform covariance from base_link to odom
      // Get the rotation of the odometry R_odom_base
      Eigen::Matrix3d R_odom_base = ekf_wrapper_.get_T_a_c(
        pos_odom_base,
        ori_odom_base,
        Eigen::Matrix4d::Identity()).block<3, 3>(0, 0);
      // Transform covariance from base_link to odom
      // Extract the full 3x3 position covariance submatrix from the 6x6 pose covariance
      Eigen::Matrix3d pos_cov_base = Eigen::Matrix3d::Zero();
      pos_cov_base(0, 0) = pose_covariance_diagonal[0];
      pos_cov_base(1, 1) = pose_covariance_diagonal[1];
      pos_cov_base(2, 2) = pose_covariance_diagonal[2];
      // Transform: Σ_odom = R_odom_base * Σ_base * R_odom_base^T
      Eigen::Matrix3d pos_cov_odom = R_odom_base * pos_cov_base * R_odom_base.transpose();
      // Extract the full 3x3 orientation covariance submatrix
      Eigen::Matrix3d ori_cov_base = Eigen::Matrix3d::Zero();
      ori_cov_base(0, 0) = pose_covariance_diagonal[3];
      ori_cov_base(1, 1) = pose_covariance_diagonal[4];
      ori_cov_base(2, 2) = pose_covariance_diagonal[5];
      // Transform orientation covariance
      Eigen::Matrix3d ori_cov_odom = R_odom_base * ori_cov_base * R_odom_base.transpose();
      // Update pose_covariance_diagonal with transformed values
      pose_covariance_diagonal[0] = pos_cov_odom(0, 0);
      pose_covariance_diagonal[1] = pos_cov_odom(1, 1);
      pose_covariance_diagonal[2] = pos_cov_odom(2, 2);
      pose_covariance_diagonal[3] = ori_cov_odom(0, 0);
      pose_covariance_diagonal[4] = ori_cov_odom(1, 1);
      pose_covariance_diagonal[5] = ori_cov_odom(2, 2);
      // }
    }

    if (rotate_covariance_from_odom_to_map_) {
      // Transform covariance from odom to map
      Eigen::Matrix3d R_map_odom = T_map_odom.block<3, 3>(0, 0);
      // Extract the full 3x3 position covariance submatrix from the 6x6 pose covariance
      Eigen::Matrix3d pos_cov_odom = Eigen::Matrix3d::Zero();
      pos_cov_odom(0, 0) = pose_covariance_diagonal[0];
      pos_cov_odom(1, 1) = pose_covariance_diagonal[1];
      pos_cov_odom(2, 2) = pose_covariance_diagonal[2];

      // Transform: Σ_map = R_map_odom * Σ_odom * R_map_odom^T
      Eigen::Matrix3d pos_cov_map = R_map_odom * pos_cov_odom * R_map_odom.transpose();

      // Extract the full 3x3 orientation covariance submatrix
      Eigen::Matrix3d ori_cov_odom = Eigen::Matrix3d::Zero();
      ori_cov_odom(0, 0) = pose_covariance_diagonal[3];
      ori_cov_odom(1, 1) = pose_covariance_diagonal[4];
      ori_cov_odom(2, 2) = pose_covariance_diagonal[5];

      // Transform orientation covariance
      Eigen::Matrix3d ori_cov_map = R_map_odom * ori_cov_odom * R_map_odom.transpose();

      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Covariance before transformation:");
      //   RCLCPP_INFO(node_ptr_->get_logger(), " Position covariance diagonal: %f, %f, %f",
      //               pose_covariance_diagonal[0], pose_covariance_diagonal[1],
      //               pose_covariance_diagonal[2]);
      //   RCLCPP_INFO(node_ptr_->get_logger(), " Orientation covariance diagonal: %f, %f, %f",
      //               pose_covariance_diagonal[3], pose_covariance_diagonal[4],
      //               pose_covariance_diagonal[5]);

      //   RCLCPP_INFO(node_ptr_->get_logger(), "Rotation from map to odom frame:");
      //   RCLCPP_INFO(node_ptr_->get_logger(), "\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f",
      //               R_map_odom(0,0), R_map_odom(0,1), R_map_odom(0,2),
      //               R_map_odom(1,0), R_map_odom(1,1), R_map_odom(1,2),
      //               R_map_odom(2,0), R_map_odom(2,1), R_map_odom(2,2));
      // }

      // Update pose_covariance_diagonal with diagonal elements only (if you need them)
      pose_covariance_diagonal[0] = pos_cov_map(0, 0);
      pose_covariance_diagonal[1] = pos_cov_map(1, 1);
      pose_covariance_diagonal[2] = pos_cov_map(2, 2);
      pose_covariance_diagonal[3] = ori_cov_map(0, 0);
      pose_covariance_diagonal[4] = ori_cov_map(1, 1);
      pose_covariance_diagonal[5] = ori_cov_map(2, 2);

      // if (verbose_) {
      //   RCLCPP_INFO(node_ptr_->get_logger(), "Transformed pose covariance to map frame:");
      //   RCLCPP_INFO(node_ptr_->get_logger(), " Position covariance diagonal: %f, %f, %f",
      //               pose_covariance_diagonal[0], pose_covariance_diagonal[1],
      //               pose_covariance_diagonal[2]);
      //   RCLCPP_INFO(node_ptr_->get_logger(), " Orientation covariance diagonal: %f, %f, %f",
      //               pose_covariance_diagonal[3], pose_covariance_diagonal[4],
      //               pose_covariance_diagonal[5]);
      // }
    }

    // IGNORE VELOCITY OF ODOM FOR NOW
    // // Correct velocity to be in map frame
    // // Rotate from base_link to map frame
    // Eigen::Matrix3d R_map_base = T_map_base.block<3, 3>(0, 0);
    // Eigen::Vector3d vel_base = Eigen::Vector3d(
    //   twist_msg->twist.linear.x,
    //   twist_msg->twist.linear.y,
    //   twist_msg->twist.linear.z);
    // Eigen::Vector3d vel_map = R_map_base * vel_base;

    // std::array<double, ekf::VelocityMeasurement::size> velocity_values = {
    //   vel_map[0],   // vx
    //   vel_map[1],   // vy
    //   vel_map[2]   // vz
    // };
    // velocity_meas.set(velocity_values);

    // // Prepare velocity Covariance
    // ekf::VelocityMeasurementCovariance velocity_cov;
    // std::array<double, ekf::VelocityMeasurementCovariance::size> velocity_cov_values;
    // velocity_cov_values[0] = twist_covariance_diagonal[0];   // vx
    // velocity_cov_values[1] = twist_covariance_diagonal[1];   // vy
    // velocity_cov_values[2] = twist_covariance_diagonal[2];   // vz
    // velocity_cov.set(velocity_cov_values);

    // if (verbose_) {
    //   std::ostringstream oss;
    //   oss << pose_map_base;
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE POSE VELOCITY: \n%s",
    //   velocity_meas.to_string().c_str());
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Pose: \n%s", oss.str().c_str());
    // }

    // ekf_wrapper_.update_velocity(
    //   velocity_meas,
    //   velocity_cov
    // );    // Update velocity

    // Try not using the velocity for the update
    ekf::PoseMeasurement z;
    z.data[0] = pose_map_base[0];
    z.data[1] = pose_map_base[1];
    z.data[2] = pose_map_base[2];
    z.data[3] = corrected_euler_map_base[0];
    z.data[4] = corrected_euler_map_base[1];
    z.data[5] = corrected_euler_map_base[2];
    ekf::PoseMeasurementCovariance measurement_noise_covariance;
    measurement_noise_covariance.data[0] = pose_covariance_diagonal[0];
    measurement_noise_covariance.data[1] = pose_covariance_diagonal[1];
    measurement_noise_covariance.data[2] = pose_covariance_diagonal[2];
    measurement_noise_covariance.data[3] = pose_covariance_diagonal[3];
    measurement_noise_covariance.data[4] = pose_covariance_diagonal[4];
    measurement_noise_covariance.data[5] = pose_covariance_diagonal[5];
    ekf_wrapper_.update_pose_odom(z, measurement_noise_covariance);

    if (debug_ekf_input_odom_pub_) {
      nav_msgs::msg::Odometry debug_odom_msg;
      debug_odom_msg.header = msg->header;
      debug_odom_msg.child_frame_id = "drone0/base_link";
      debug_odom_msg.pose.pose = geometry_msgs::msg::Pose();
      Eigen::Matrix4d T_map_odom = ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(
          z.data[0],
          z.data[1],
          z.data[2]),
        Eigen::Vector3d(
          z.data[3],
          z.data[4],
          z.data[5]),
        Eigen::Matrix4d::Identity());
      Eigen::Vector<double, 7> pose_map_odom = ekf::EKFWrapper::transform_to_pose(T_map_odom);
      debug_odom_msg.pose.pose.position.x = pose_map_odom[0];
      debug_odom_msg.pose.pose.position.y = pose_map_odom[1];
      debug_odom_msg.pose.pose.position.z = pose_map_odom[2];
      debug_odom_msg.pose.pose.orientation.x = pose_map_odom[3];
      debug_odom_msg.pose.pose.orientation.y = pose_map_odom[4];
      debug_odom_msg.pose.pose.orientation.z = pose_map_odom[5];
      debug_odom_msg.pose.pose.orientation.w = pose_map_odom[6];
      for (size_t i = 0; i < 36; i++) {
        debug_odom_msg.pose.covariance[i] = 0.0;
      }
      for (size_t i = 0; i < 6; i++) {
        debug_odom_msg.pose.covariance[i * 6 + i] = pose_covariance_diagonal[i];
      }
      debug_ekf_input_odom_pub_->publish(debug_odom_msg);
    }

    // // Publish real correction pose for debugging
    // if (debug_ekf_real_corrections_pub_) {
    //   Eigen::Matrix4d map_to_pose_eigen =
    //   ekf_wrapper_.get_T_b_c(
    //     Eigen::Vector3d(z.data[0], z.data[1], z.data[2]),
    //     Eigen::Vector3d(z.data[3], z.data[4], z.data[5]),
    //     Eigen::Matrix4d::Identity());
    //   Eigen::Vector<double, 7> map_to_pose_values =
    //     ekf::EKFWrapper::transform_to_pose(map_to_pose_eigen);
    //   geometry_msgs::msg::PoseWithCovarianceStamped correction_msg;
    //   correction_msg.header = msg->header;
    //   correction_msg.header.stamp = node_ptr_->now();
    //   correction_msg.header.frame_id = "drone0/map";
    //   correction_msg.pose.pose.position.x = map_to_pose_values[0];
    //   correction_msg.pose.pose.position.y = map_to_pose_values[1];
    //   correction_msg.pose.pose.position.z = map_to_pose_values[2];
    //   correction_msg.pose.pose.orientation.x = map_to_pose_values[3];
    //   correction_msg.pose.pose.orientation.y = map_to_pose_values[4];
    //   correction_msg.pose.pose.orientation.z = map_to_pose_values[5];
    //   correction_msg.pose.pose.orientation.w = map_to_pose_values[6];
    //   for (size_t i = 0; i < 36; i++) {
    //     correction_msg.pose.covariance[i] = 0.0;
    //   }
    //   for (size_t i = 0; i < 6; i++) {
    //     correction_msg.pose.covariance[i * 6 + i] = pose_covariance_diagonal[i];
    //   }
    //   debug_ekf_real_corrections_pub_->publish(correction_msg);
    // }

    // if (verbose_) {
    //   RCLCPP_INFO(node_ptr_->get_logger(), "UPDATE");
    //   state = ekf_wrapper_.get_state();
    //   covariance = ekf_wrapper_.get_state_covariance();
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current state update: \n%s",
    //               state.to_string().c_str());
    //   RCLCPP_INFO(node_ptr_->get_logger(), "Current covariance update: \n%s",
    //              covariance.to_string_diagonal().c_str());
    // }
    // can_update_ = false;
    can_predict_ = true;
  }


  void timer_callback()
  {
    // Publish earth map
    if (!earth_to_map_set_) {
      if (!tf_initialized_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_initialized_ = true;
        pose_updater_.setTFBuffer(tf_buffer_);
      }
      if (!odom_set_earth_map_ && !pose_set_earth_map_) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Neither <ekf_fuse.pose_params.set_earth_map> nor <ekf_fuse.odom_params.set_earth_map> "
          "are true, the earth to map transform will be set with fixed parameters");
        T_earth_to_map_ = ekf_wrapper_.get_T_b_c(
          Eigen::Vector3d(
            fixed_earth_map_x_,
            fixed_earth_map_y_,
            fixed_earth_map_z_),
          Eigen::Vector3d(
            fixed_earth_map_roll_,
            fixed_earth_map_pitch_,
            fixed_earth_map_yaw_),
          Eigen::Matrix4d::Identity());
        Eigen::Vector<double,
          7> pose_earth_map = ekf::EKFWrapper::transform_to_pose(T_earth_to_map_);
        geometry_msgs::msg::PoseStamped earth_to_map_msg = geometry_msgs::msg::PoseStamped();
        earth_to_map_msg.header.frame_id = "earth";
        earth_to_map_msg.header.stamp = node_ptr_->now();
        earth_to_map_msg.pose.position.x = pose_earth_map[0];
        earth_to_map_msg.pose.position.y = pose_earth_map[1];
        earth_to_map_msg.pose.position.z = pose_earth_map[2];
        earth_to_map_msg.pose.orientation.x = pose_earth_map[3];
        earth_to_map_msg.pose.orientation.y = pose_earth_map[4];
        earth_to_map_msg.pose.orientation.z = pose_earth_map[5];
        earth_to_map_msg.pose.orientation.w = pose_earth_map[6];
        generate_map_frame_from_ground_truth_pose(earth_to_map_msg);
        pose_updater_.setT_earth_to_map(T_earth_to_map_);
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Setting earth to map with fixed parameters: "
          "x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
          fixed_earth_map_x_, fixed_earth_map_y_, fixed_earth_map_z_,
          fixed_earth_map_roll_, fixed_earth_map_pitch_, fixed_earth_map_yaw_);
        earth_to_map_set_ = true;
      } else {
        try {
          tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
          state_estimator_interface_->setEarthToMap(earth_map_tf, node_ptr_->now(), true);
        } catch (const std::exception & e) {
          RCLCPP_INFO(
            node_ptr_->get_logger(), "Failed to set earth to map transform: %s",
            e.what());
        }
      }
      return;
    }

    // Print EKF state for debugging
    if (verbose_) {
      ekf::State state = ekf_wrapper_.get_state();
      ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();
      RCLCPP_INFO_THROTTLE(
        node_ptr_->get_logger(),
        *node_ptr_->get_clock(),
        2000,
        "EKF State: \n%s", state.to_string().c_str());
      RCLCPP_INFO_THROTTLE(
        node_ptr_->get_logger(),
        *node_ptr_->get_clock(),
        2000,
        "EKF Covariance (diagonal): \n%s", covariance.to_string_diagonal().c_str());
    }

    if (!offboard_activated_) {
      // Generate 0 pose measurement to initialize EKF
      ekf::PoseMeasurement pose_meas;
      pose_meas.data[0] = 0.0;
      pose_meas.data[1] = 0.0;
      pose_meas.data[2] = 0.0;
      pose_meas.data[3] = 0.0;
      pose_meas.data[4] = 0.0;
      pose_meas.data[5] = 0.0;
      ekf::PoseMeasurementCovariance pose_cov;
      pose_cov.data[0] = 1e-3;
      pose_cov.data[1] = 1e-3;
      pose_cov.data[2] = 1e-3;
      pose_cov.data[3] = 1e-3;
      pose_cov.data[4] = 1e-3;
      pose_cov.data[5] = 1e-3;
      ekf_wrapper_.update_pose(
        pose_meas,
        pose_cov
      );
      RCLCPP_INFO_THROTTLE(
        node_ptr_->get_logger(),
        *node_ptr_->get_clock(),
        2000,
        "Offboard not activated yet, initializing EKF with zero pose measurement, "
        "offboard_activated_: %d", offboard_activated_);
    }

    // Get current state and covariance
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();

    if (debug_ekf_state_pose_pub_) {
      Eigen::Matrix4d map_to_baselink_eigen =
        ekf_wrapper_.get_T_b_c(
        Eigen::Vector3d(state.get_position().data()),
        Eigen::Vector3d(state.get_orientation().data()),
        Eigen::Matrix4d::Identity());
      Eigen::Vector<double, 7> map_to_baselink_values =
        ekf::EKFWrapper::transform_to_pose(map_to_baselink_eigen);
      geometry_msgs::msg::PoseWithCovarianceStamped map_to_baselink_msg;
      map_to_baselink_msg.header.stamp = node_ptr_->now();
      map_to_baselink_msg.header.frame_id = "drone0/map";
      map_to_baselink_msg.pose.pose.position.x = map_to_baselink_values[0];
      map_to_baselink_msg.pose.pose.position.y = map_to_baselink_values[1];
      map_to_baselink_msg.pose.pose.position.z = map_to_baselink_values[2];
      map_to_baselink_msg.pose.pose.orientation.x = map_to_baselink_values[3];
      map_to_baselink_msg.pose.pose.orientation.y = map_to_baselink_values[4];
      map_to_baselink_msg.pose.pose.orientation.z = map_to_baselink_values[5];
      map_to_baselink_msg.pose.pose.orientation.w = map_to_baselink_values[6];
      for (size_t i = 0; i < 36; i++) {
        map_to_baselink_msg.pose.covariance[i] = 0.0;
      }
      map_to_baselink_msg.pose.covariance[0] = covariance.data[ekf::Covariance::X];   // x
      map_to_baselink_msg.pose.covariance[7] = covariance.data[ekf::Covariance::Y];   // y
      map_to_baselink_msg.pose.covariance[14] = covariance.data[ekf::Covariance::Z];   // z
      map_to_baselink_msg.pose.covariance[21] = covariance.data[ekf::Covariance::ROLL];   // roll
      map_to_baselink_msg.pose.covariance[28] = covariance.data[ekf::Covariance::PITCH];   // pitch
      map_to_baselink_msg.pose.covariance[35] = covariance.data[ekf::Covariance::YAW];   // yaw
      debug_ekf_state_pose_pub_->publish(map_to_baselink_msg);
    }

    // Get map to odom transform
    Eigen::Matrix4d map_to_odom_eigen = ekf_wrapper_.get_map_to_odom();

    // Get odom to base_link transform
    Eigen::Matrix4d odom_to_baselink_eigen =
      ekf_wrapper_.get_T_b_c(
      Eigen::Vector3d(state.get_position().data()),
      Eigen::Vector3d(state.get_orientation().data()),
      map_to_odom_eigen);

    // Transform from eigen to position + quaternion
    Eigen::Vector<double, 7> odom_to_baselink_values =
      ekf::EKFWrapper::transform_to_pose(odom_to_baselink_eigen);

    // Prepare pose to update the state estimator
    geometry_msgs::msg::PoseWithCovariance odom_to_baselink_pose;
    // Position and orientation
    odom_to_baselink_pose.pose.position.x = odom_to_baselink_values[0];
    odom_to_baselink_pose.pose.position.y = odom_to_baselink_values[1];
    odom_to_baselink_pose.pose.position.z = odom_to_baselink_values[2];
    odom_to_baselink_pose.pose.orientation.x = odom_to_baselink_values[3];
    odom_to_baselink_pose.pose.orientation.y = odom_to_baselink_values[4];
    odom_to_baselink_pose.pose.orientation.z = odom_to_baselink_values[5];
    odom_to_baselink_pose.pose.orientation.w = odom_to_baselink_values[6];
    // Covariance
    for (size_t i = 0; i < 36; i++) {
      odom_to_baselink_pose.covariance[i] = 0.0;
    }
    odom_to_baselink_pose.covariance[0] = covariance.data[ekf::Covariance::X];   // x
    odom_to_baselink_pose.covariance[7] = covariance.data[ekf::Covariance::Y];   // y
    odom_to_baselink_pose.covariance[14] = covariance.data[ekf::Covariance::Z];   // z
    odom_to_baselink_pose.covariance[21] = covariance.data[ekf::Covariance::ROLL];   // roll
    odom_to_baselink_pose.covariance[28] = covariance.data[ekf::Covariance::PITCH];   // pitch
    odom_to_baselink_pose.covariance[35] = covariance.data[ekf::Covariance::YAW];   // yaw

    // Update odom to base_link transform in the state estimator
    state_estimator_interface_->setOdomToBaseLinkPose(
      odom_to_baselink_pose,
      node_ptr_->now());


    // Print last_imu_t_ vs now_dt
    double now_dt = node_ptr_->now().seconds();
    double dt_diff = now_dt - last_imu_t_;
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Time comparison: last_imu_t_: %.9f, now_dt: %.9f, difference: %.9f s",
      last_imu_t_, now_dt, dt_diff);


    // Transform from eigen to position + quaternion
    Eigen::Vector<double, 7> map_to_odom_new =
      ekf::EKFWrapper::transform_to_pose(map_to_odom_eigen);


    if (!map_to_odom_initialized_) {
      global_map_to_odom_ = map_to_odom_new;
      map_to_odom_initialized_ = true;
      if (verbose_) {
        RCLCPP_INFO(
          node_ptr_->get_logger(),
          "Initialized global_map_to_odom_ to first EKF value: [%.3f, %.3f, %.3f]",
          map_to_odom_new[0], map_to_odom_new[1], map_to_odom_new[2]);
      }
    }

    Eigen::Vector<double, 7> map_to_odom_values;
    Eigen::Vector3d vel_map_odom_values;
    double blend_ratio = 1.0;  // Default to full change if smoothing is disabled

    // Get map to base_link transform
    Eigen::Matrix4d map_to_baselink_eigen =
      ekf_wrapper_.get_T_b_c(
      Eigen::Vector3d(state.get_position().data()),
      Eigen::Vector3d(state.get_orientation().data()),
      Eigen::Matrix4d::Identity());
    Eigen::Matrix3d R_map_base = map_to_baselink_eigen.block<3, 3>(0, 0);

    // Transform linear velocity from earth to base_link
    Eigen::Vector3d vel_map = Eigen::Vector3d(
      state.data[ekf::State::VX],
      state.data[ekf::State::VY],
      state.data[ekf::State::VZ]);
    Eigen::Vector3d vel_map_odom_new = ekf_wrapper_.get_map_to_odom_velocity();
    Eigen::Vector3d vel_odom = vel_map - vel_map_odom_new;

    // Check if using alpha or stepping and blend accordingly
    if (activate_step_smoothing_) {
      // RCLCPP_INFO(
      //   node_ptr_->get_logger(),
      //   "Applying step smoothing for map to odom update with max step: %f",
      //   max_step_);
      auto blend_result = EKFFuseUtils::blendPosesWithRatio(
        global_map_to_odom_, map_to_odom_new,
        max_step_);
      map_to_odom_values = blend_result.first;
      blend_ratio = blend_result.second;
      vel_map_odom_values = EKFFuseUtils::blendTwistsWithRatio(
        global_map_to_odom_velocity_, vel_map_odom_new, blend_ratio);
    } else {
      // RCLCPP_INFO(
      //   node_ptr_->get_logger(),
      //   "Applying alpha smoothing for map to odom update with alpha: %f",
      //   map_odom_alpha_);
      map_to_odom_values = EKFFuseUtils::blendPoses(
        global_map_to_odom_, map_to_odom_new, map_odom_alpha_);
      vel_map_odom_values = EKFFuseUtils::blendTwists(
        global_map_to_odom_velocity_, vel_map_odom_new, map_odom_alpha_);
    }

    // RCLCPP_INFO(
    //   node_ptr_->get_logger(),
    //   "Blend ratio for map to odom update: %f",
    //   blend_ratio);
    // RCLCPP_INFO(
    //   node_ptr_->get_logger(),
    //   "Map to odom update: from [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]"
    //   "resulting in [%.3f, %.3f, %.3f]",
    //   global_map_to_odom_[0], global_map_to_odom_[1], global_map_to_odom_[2],
    //   map_to_odom_new[0], map_to_odom_new[1], map_to_odom_new[2],
    //   map_to_odom_values[0], map_to_odom_values[1],  map_to_odom_values[2]);

    Eigen::Vector3d vel_map_base = vel_odom + vel_map_odom_values;
    Eigen::Vector3d vel_base = R_map_base.transpose() * vel_map_base;

    global_map_to_odom_ = map_to_odom_values;
    global_map_to_odom_velocity_ = vel_map_odom_values;

    // Prepare map to odom pose to update the state estimator
    geometry_msgs::msg::PoseWithCovariance map_to_odom_pose;
    // Position and orientation
    map_to_odom_pose.pose.position.x = map_to_odom_values[0];
    map_to_odom_pose.pose.position.y = map_to_odom_values[1];
    map_to_odom_pose.pose.position.z = map_to_odom_values[2];
    map_to_odom_pose.pose.orientation.x = map_to_odom_values[3];
    map_to_odom_pose.pose.orientation.y = map_to_odom_values[4];
    map_to_odom_pose.pose.orientation.z = map_to_odom_values[5];
    map_to_odom_pose.pose.orientation.w = map_to_odom_values[6];
    // Covariance
    for (size_t i = 0; i < 36; i++) {
      map_to_odom_pose.covariance[i] = 0.0;
    }
    map_to_odom_pose.covariance[0] = covariance.data[ekf::Covariance::X];     // x
    map_to_odom_pose.covariance[7] = covariance.data[ekf::Covariance::Y];     // y
    map_to_odom_pose.covariance[14] = covariance.data[ekf::Covariance::Z];     // z
    map_to_odom_pose.covariance[21] = covariance.data[ekf::Covariance::ROLL];     // roll
    map_to_odom_pose.covariance[28] = covariance.data[ekf::Covariance::PITCH];     // pitch
    map_to_odom_pose.covariance[35] = covariance.data[ekf::Covariance::YAW];     // yaw

    // Update map to odom transform in the state estimator
    state_estimator_interface_->setMapToOdomPose(
      map_to_odom_pose,
      node_ptr_->now());


    // if (verbose_) {
    //   RCLCPP_INFO(
    //     node_ptr_->get_logger(),
    //     "Velocities: vel_map_base: %f, %f, %f | vel_map: %f, %f, %f",
    //     vel_map_base[0], vel_map_base[1], vel_map_base[2],
    //     vel_map[0], vel_map[1], vel_map[2]);
    // }

    // Build transformation matrices directly from quaternions
    // map_to_odom_values and odom_to_baselink_values are [x, y, z, qx, qy, qz, qw]
    Eigen::Matrix4d map_to_odom_pose_eigen = Eigen::Matrix4d::Identity();
    map_to_odom_pose_eigen.block<3, 1>(0, 3) = Eigen::Vector3d(
      map_to_odom_values[0], map_to_odom_values[1], map_to_odom_values[2]);
    map_to_odom_pose_eigen.block<3, 3>(0, 0) = Eigen::Quaterniond(
      map_to_odom_values[6], map_to_odom_values[3],
      map_to_odom_values[4], map_to_odom_values[5]).toRotationMatrix();

    Eigen::Matrix4d odom_to_baselink_pose_eigen = Eigen::Matrix4d::Identity();
    odom_to_baselink_pose_eigen.block<3, 1>(0, 3) = Eigen::Vector3d(
      odom_to_baselink_values[0], odom_to_baselink_values[1], odom_to_baselink_values[2]);
    odom_to_baselink_pose_eigen.block<3, 3>(0, 0) = Eigen::Quaterniond(
      odom_to_baselink_values[6], odom_to_baselink_values[3],
      odom_to_baselink_values[4], odom_to_baselink_values[5]).toRotationMatrix();

    Eigen::Matrix4d map_to_baselink_pose_eigen =
      map_to_odom_pose_eigen * odom_to_baselink_pose_eigen;
    Eigen::Vector<double, 7> global_map_to_baselink_pose_values =
      ekf::EKFWrapper::transform_to_pose(map_to_baselink_pose_eigen);

    // Transform quat to roll pitch yaw
    auto current_rpy = EKFFuseUtils::quaternionToEuler(
      Eigen::Quaterniond(
        global_map_to_baselink_pose_values[6],
        global_map_to_baselink_pose_values[3],
        global_map_to_baselink_pose_values[4],
        global_map_to_baselink_pose_values[5]));

    // Publish debug information
    Eigen::Vector3d vel_base_debug = R_map_base.transpose() * vel_map;

    // Calculate orientation derivative with low-pass filtering
    // Use IMU timestamp (last_imu_t_) instead of current wall clock time (now())
    // This ensures we're using the same time reference as the orientation data
    // which was updated by the IMU prediction step, eliminating timestamp mismatch
    double current_time = last_imu_t_;
    double d_roll = 0.0, d_pitch = 0.0, d_yaw = 0.0;

    double current_roll = current_rpy[0];
    double current_pitch = current_rpy[1];
    double current_yaw = current_rpy[2];

    if (orientation_initialized_ && last_orientation_time_ > 0.0) {
      double dt_orientation = current_time - last_orientation_time_;

      // Only compute derivative if dt is large enough to avoid numerical instability
      // Small dt values (e.g., < 5ms) can cause huge spikes from tiny measurement noise
      if (dt_orientation > min_dt_for_derivative_) {
        // Calculate angle differences, handling wraparound at -pi/pi
        // Normalize differences to [-pi, pi] range
        auto normalize_angle_diff = [](double angle_diff) {
            // while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            // while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
            // return angle_diff;
            return std::atan2(std::sin(angle_diff), std::cos(angle_diff));
          };

        double delta_roll = normalize_angle_diff(current_roll - last_orientation_[0]);
        double delta_pitch = normalize_angle_diff(current_pitch - last_orientation_[1]);
        double delta_yaw = normalize_angle_diff(current_yaw - last_orientation_[2]);

        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Orientation deltas (rad): roll: %.6f, pitch: %.6f, yaw: %.6f over dt: %.6f s",
          delta_roll, delta_pitch, delta_yaw, dt_orientation);

        // Calculate raw derivatives using normalized angle differences
        d_roll = delta_roll / dt_orientation;
        d_pitch = delta_pitch / dt_orientation;
        d_yaw = delta_yaw / dt_orientation;

        // Detect spikes BEFORE applying filter
        bool is_spike = (std::abs(d_roll) > max_orientation_derivative_ ||
          std::abs(d_pitch) > max_orientation_derivative_ ||
          std::abs(d_yaw) > max_orientation_derivative_);

        if (is_spike) {
          RCLCPP_WARN(
            node_ptr_->get_logger(),
            "Detected spike in orientation derivative: d_roll: %.6f, d_pitch: %.6f, d_yaw: %.6f. "
            "Ignoring this measurement to maintain stability.",
            d_roll, d_pitch, d_yaw);
          // Don't update filtered values, but DO update the reference for next iteration
          // Otherwise dt keeps growing and we get stuck in a spike loop
        } else {
          // Apply exponential moving average filter to reduce noise
          // filtered = alpha * new_value + (1 - alpha) * old_filtered
          filtered_orientation_derivative_[0] = derivative_filter_alpha_ * d_roll +
            (1.0 - derivative_filter_alpha_) * filtered_orientation_derivative_[0];
          filtered_orientation_derivative_[1] = derivative_filter_alpha_ * d_pitch +
            (1.0 - derivative_filter_alpha_) * filtered_orientation_derivative_[1];
          filtered_orientation_derivative_[2] = derivative_filter_alpha_ * d_yaw +
            (1.0 - derivative_filter_alpha_) * filtered_orientation_derivative_[2];
        }

        // Always update last orientation for next iteration (even if spike)
        // This prevents dt from growing indefinitely
        last_orientation_[0] = current_roll;
        last_orientation_[1] = current_pitch;
        last_orientation_[2] = current_yaw;
        last_orientation_time_ = current_time;
      }
      // If dt is too small, we skip the derivative update and keep using the last filtered value
    } else {
      // First iteration - just initialize
      last_orientation_[0] = current_roll;
      last_orientation_[1] = current_pitch;
      last_orientation_[2] = current_yaw;
      last_orientation_time_ = current_time;
    }

    orientation_initialized_ = true;

    if (debug_ekf_state_velocity_pub_) {
      geometry_msgs::msg::TwistWithCovarianceStamped msg;
      msg.header.stamp = node_ptr_->now();
      msg.twist.twist.linear.x = vel_base_debug[0];
      msg.twist.twist.linear.y = vel_base_debug[1];
      msg.twist.twist.linear.z = vel_base_debug[2];
      msg.twist.twist.angular.x = filtered_orientation_derivative_[0];
      msg.twist.twist.angular.y = filtered_orientation_derivative_[1];
      msg.twist.twist.angular.z = filtered_orientation_derivative_[2];
      for (size_t i = 0; i < 36; i++) {
        msg.twist.covariance[i] = 0.0;
      }
      msg.twist.covariance[0] = covariance.data[ekf::Covariance::VX];   // vx
      msg.twist.covariance[7] = covariance.data[ekf::Covariance::VY];   // vy
      msg.twist.covariance[14] = covariance.data[ekf::Covariance::VZ];   // vz
      debug_ekf_state_velocity_pub_->publish(msg);
    }

    // Prepare twist to update the state estimator
    geometry_msgs::msg::TwistWithCovariance twist;
    // Linear and angular velocities
    twist.twist.linear.x = vel_base[0];
    twist.twist.linear.y = vel_base[1];
    twist.twist.linear.z = vel_base[2];
    if (roll_pitch_fixed_) {
      twist.twist.angular.x = last_imu_input_.data[ekf::Input::WX] - state.data[ekf::State::WBX];
      twist.twist.angular.y = last_imu_input_.data[ekf::Input::WY] - state.data[ekf::State::WBY];
      twist.twist.angular.z = last_imu_input_.data[ekf::Input::WZ] - state.data[ekf::State::WBZ];
    } else {
      twist.twist.angular.x = filtered_orientation_derivative_[0];
      twist.twist.angular.y = filtered_orientation_derivative_[1];
      twist.twist.angular.z = filtered_orientation_derivative_[2];
    }
    // Covariance
    for (size_t i = 0; i < 36; i++) {
      twist.covariance[i] = 0.0;
    }
    twist.covariance[0] = covariance.data[ekf::Covariance::VX];   // vx
    twist.covariance[7] = covariance.data[ekf::Covariance::VY];   // vy
    twist.covariance[14] = covariance.data[ekf::Covariance::VZ];   // vz
    twist.covariance[21] = 0.0;   // wx
    twist.covariance[28] = 0.0;   // wy
    twist.covariance[35] = 0.0;   // wz

    state_estimator_interface_->setTwistInBaseFrame(
      twist,
      node_ptr_->now());


    if (verbose_) {
      // If state contains NAN exit
      for (size_t i = 0; i < ekf::State::size; i++) {
        if (std::isnan(state.data[i])) {
          RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "State contains NAN, exiting");
          rclcpp::shutdown();
        }
      }
    }
  }
};         // class ekf_fuse
}       // namespace ekf_fuse
#endif  // EKF_FUSE_HPP_
