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
* @file ekf.hpp
*
* An state estimation plugin ekf for AeroStack2
*
* @authors Rodrigo da Silva
*/

#ifndef EKF_FUSE_HPP_
#define EKF_FUSE_HPP_

#include <array>
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
#include <iostream>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/utils/conversions.hpp"
#include "as2_core/names/topics.hpp"

#include <ekf/ekf_wrapper.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <vector>


namespace ekf_fuse
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  // geometry_msgs::msg::TwistStamped last_twist_msg_;

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> predict_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> update_pose_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> update_pose_velocity_subs_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_ptr_;

  // EKF
  ekf::EKFWrapper ekf_wrapper_;
  double last_dt_ = 0.0;
  std::array<double, 3> last_angular_velocity_ = {0.0, 0.0, 0.0};

  // Initial state and covariance
  ekf::State initial_state_;
  ekf::Covariance initial_covariance_;

  // Parameters
  bool use_measurement_covariances_for_odometry_ = false;
  double meas_position_cov_ = 1e-6;        // 1e-6
  double meas_velocity_cov_ = 1e-5;        // 1e-5
  double meas_orientation_cov_ = 1e-7;     // 1e-7
  bool verbose_ = false;
  bool can_update_ = true;

  bool use_gazebo_ = false;

  // TF related
  bool earth_to_map_set_ = false;
  Eigen::Matrix4d T_earth_to_map_ = Eigen::Matrix4d::Identity();
  tf2::Transform earth_to_baselink = tf2::Transform::getIdentity();
  tf2::Transform odom_to_baselink = tf2::Transform::getIdentity();

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}
  void onSetup() override
  {
    if (node_ptr_->has_parameter("ekf_fuse.verbose")) {
      verbose_ = node_ptr_->get_parameter("ekf_fuse.verbose").as_bool();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.verbose> not defined, using default (false)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.use_gazebo")) {
      use_gazebo_ = node_ptr_->get_parameter("ekf_fuse.use_gazebo").as_bool();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.use_gazebo> not defined, using default (false)");
    }
    // Set initial state and covariance
    std::array<double, ekf::State::size> initial_state_values = {
      0.0, 0.0, 0.0,   // position
      0.0, 0.0, 0.0,   // velocity
      0.0, 0.0, 0.0,   // orientation
      0.0, 0.0, 0.0,   // bias accelerometer
      0.0, 0.0, 0.0    // bias gyroscope
    };
    initial_state_.set(initial_state_values);
    double position_cov = 1e-3;      // 1e-3
    double velocity_cov = 1e-5;      // 1e-5
    double orientation_cov = 1e-6;   // 1e-6
    double bias_acc_cov = 1e-8;      // 1e-8
    double bias_gyro_cov = 1e-8;     // 1e-8
    if (node_ptr_->has_parameter("ekf_fuse.initial_covariances.position")) {
      position_cov = node_ptr_->get_parameter("ekf_fuse.initial_covariances.position").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Parameter <ekf_fuse.initial_covariances.position> not defined");
    }
    if (node_ptr_->has_parameter("ekf_fuse.initial_covariances.velocity")) {
      velocity_cov = node_ptr_->get_parameter("ekf_fuse.initial_covariances.velocity").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Parameter <ekf_fuse.initial_covariances.velocity> not defined");
    }
    if (node_ptr_->has_parameter("ekf_fuse.initial_covariances.orientation")) {
      orientation_cov =
        node_ptr_->get_parameter("ekf_fuse.initial_covariances.orientation").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.initial_covariances.orientation> not defined");
    }
    if (node_ptr_->has_parameter("ekf_fuse.initial_covariances.bias_accelerometer")) {
      bias_acc_cov =
        node_ptr_->get_parameter("ekf_fuse.initial_covariances.bias_accelerometer").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.initial_covariances.bias_accelerometer> not defined");
    }
    if (node_ptr_->has_parameter("ekf_fuse.initial_covariances.bias_gyroscope")) {
      bias_gyro_cov =
        node_ptr_->get_parameter("ekf_fuse.initial_covariances.bias_gyroscope").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.initial_covariances.bias_gyroscope> not defined");
    }
    std::array<double, ekf::Covariance::size> initial_covariance_values =
    {
      position_cov, position_cov, position_cov,
      velocity_cov, velocity_cov, velocity_cov,
      orientation_cov, orientation_cov, orientation_cov,
      bias_acc_cov, bias_acc_cov, bias_acc_cov,
      bias_gyro_cov, bias_gyro_cov, bias_gyro_cov
    };
    initial_covariance_.set(initial_covariance_values);
    ekf_wrapper_.reset(
      initial_state_,
      initial_covariance_
    );

    // Set gravity Vector
    ekf::Gravity gravity = ekf::Gravity(std::array<double, ekf::Gravity::size>({0.00, 0.0, 9.81}));
    ekf_wrapper_.set_gravity(gravity);

    // IMU noise parameters
    Eigen::Vector<double, 6> imu_noise;
    imu_noise << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // double accelerometer_noise_density = 0.0023535650744762504;
    // double gyroscope_noise_density = 0.00010733905605362857;
    // double accelerometer_random_walk = 0.00034741552232679093;
    // double gyroscope_random_walk = 1.0052151907131208e-05;
    double accelerometer_noise_density = 1e-3;
    double gyroscope_noise_density = 1e-4;
    double accelerometer_random_walk = 1e-4;
    double gyroscope_random_walk = 1e-5;
    if (node_ptr_->has_parameter("ekf_fuse.imu_params.accelerometer_noise_density")) {
      accelerometer_noise_density =
        node_ptr_->get_parameter("ekf_fuse.imu_params.accelerometer_noise_density").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.imu_params.accelerometer_noise_density> not defined, using default (1e-3)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.imu_params.gyroscope_noise_density")) {
      gyroscope_noise_density =
        node_ptr_->get_parameter("ekf_fuse.imu_params.gyroscope_noise_density").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.imu_params.gyroscope_noise_density> not defined, using default (1e-4)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.imu_params.accelerometer_random_walk")) {
      accelerometer_random_walk =
        node_ptr_->get_parameter("ekf_fuse.imu_params.accelerometer_random_walk").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.imu_params.accelerometer_random_walk> not defined, using default (1e-4)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.imu_params.gyroscope_random_walk")) {
      gyroscope_random_walk =
        node_ptr_->get_parameter("ekf_fuse.imu_params.gyroscope_random_walk").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.imu_params.gyroscope_random_walk> not defined, using default (1e-5)");
    }
    ekf_wrapper_.set_noise_parameters(
      imu_noise,
      accelerometer_noise_density,
      gyroscope_noise_density,
      accelerometer_random_walk,
      gyroscope_random_walk);

    // Measurement covariances
    if (node_ptr_->has_parameter("ekf_fuse.use_measurement_covariances_for_odometry")) {
      use_measurement_covariances_for_odometry_ =
        node_ptr_->get_parameter("ekf_fuse.use_measurement_covariances_for_odometry").as_bool();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.use_measurement_covariances_for_odometry> not defined, using default (false)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.measurement_covariances.position")) {
      meas_position_cov_ =
        node_ptr_->get_parameter("ekf_fuse.measurement_covariances.position").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.measurement_covariances.position> not defined, using default (1e-6)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.measurement_covariances.velocity")) {
      meas_velocity_cov_ =
        node_ptr_->get_parameter("ekf_fuse.measurement_covariances.velocity").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.measurement_covariances.velocity> not defined, using default (1e-5)");
    }
    if (node_ptr_->has_parameter("ekf_fuse.measurement_covariances.orientation")) {
      meas_orientation_cov_ =
        node_ptr_->get_parameter("ekf_fuse.measurement_covariances.orientation").as_double();
    } else {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter <ekf_fuse.measurement_covariances.orientation> not defined, using default (1e-7)");
    }

    // Predict topics
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
    for (const auto & topic_name : update_pose_topic_names_) {
      RCLCPP_INFO(node_ptr_->get_logger(), " - %s", topic_name.c_str());
      auto update_sub = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, as2_names::topics::self_localization::qos,
        std::bind(&Plugin::update_pose_callback, this, std::placeholders::_1));
      update_pose_subs_.push_back(update_sub);
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
    }

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


  // Function to convert euler angles to a version out of [-pi, pi] nearest to state
  inline Eigen::Vector3d correct_measured_euler(
    const Eigen::Vector3d & state_euler,
    const Eigen::Vector3d & measured_euler)
  {
    Eigen::Vector3d corrected_euler = measured_euler;
    for (size_t i = 0; i < 3; i++) {
      double diff = std::abs(state_euler[i] - measured_euler[i]);
      double diff_2 = 0.0;
      if (measured_euler[i] >= 0.0) {
        diff_2 = state_euler[i] - (measured_euler[i] - 2.0 * M_PI);
      } else {
        diff_2 = state_euler[i] - (measured_euler[i] + 2.0 * M_PI);
      }
      diff_2 = std::abs(diff_2);
      if (diff_2 < diff) {
        if (measured_euler[i] >= 0.0) {
          corrected_euler[i] = measured_euler[i] - 2.0 * M_PI;
        } else {
          corrected_euler[i] = measured_euler[i] + 2.0 * M_PI;
        }
      }
    }
    return corrected_euler;
  }


  /* Convert a quaternion to Euler angles (roll, pitch, yaw), XYZ order (radians).
   * Input is Eigen::Quaterniond with components [w, x, y, z].
   */
  inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond & q_in)
  {
    // Normalize to ensure a valid rotation
    Eigen::Quaterniond q = q_in.normalized();

    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();

    // Roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const double sinp = 2.0 * (qw * qy - qz * qx);
    const double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

    // Yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(roll, pitch, yaw);
  }


  void predict_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // if (!earth_to_map_set_) {
    //   // As earth to map is not set, we cannot predict
    //   std::cout << "Earth to map not set, cannot predict" << std::endl;
    //   return;
    // }
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
    imu_input.set(imu_values);

    // Compute time difference
    double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt = 0.0;
    if (last_dt_ > 0.0) {
      dt = msg_time - last_dt_;
    } else {
      dt = 0.1;   // default value for the first message
    }
    last_dt_ = msg_time;


    // Predict
    ekf_wrapper_.predict(imu_input, dt);

    // Store last angular velocity
    last_angular_velocity_[0] = imu_input.data[ekf::Input::WX];
    last_angular_velocity_[1] = imu_input.data[ekf::Input::WY];
    last_angular_velocity_[2] = imu_input.data[ekf::Input::WZ];

    if (verbose_) {
      std::cout << "PREDICT. DT = " << dt << std::endl;

      state = ekf_wrapper_.get_state();
      covariance = ekf_wrapper_.get_state_covariance();
      std::cout << "Current state prediction: \n" << state.to_string() << std::endl;
      std::cout << "Current covariance prediction: \n" << covariance.to_string_diagonal() <<
        std::endl;
    }
    can_update_ = true;
  }


  void update_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!earth_to_map_set_) {
      // As earth to map is not set, we assume that the first pose received is in the earth
      generate_map_frame_from_ground_truth_pose(*msg);
      tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
      if (verbose_) {
        std::cout << "Setting earth to map from ground truth" << std::endl;
        std::cout << "Earth to map value (from state estimator): \n"
                  << earth_map_tf.getOrigin().x() << ", "
                  << earth_map_tf.getOrigin().y() << ", "
                  << earth_map_tf.getOrigin().z() << std::endl;
      }
      // Convert orientation to euler
      Eigen::Quaterniond q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = quaternionToEuler(q);

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
      if (verbose_) {
        std::cout << "Skipping update to avoid updating too fast" << std::endl;
      }
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
    auto euler = quaternionToEuler(q);
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

    Eigen::Vector3d euler_map_base = quaternionToEuler(
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
    Eigen::Vector3d corrected_euler_map_base = correct_measured_euler(
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
    // TODO: set covariance from message
    ekf::PoseMeasurementCovariance pose_cov;
    std::array<double, ekf::PoseMeasurementCovariance::size> pose_cov_values = {
      meas_position_cov_, meas_position_cov_, meas_position_cov_,
      meas_orientation_cov_, meas_orientation_cov_, meas_orientation_cov_
    };
    pose_cov.set(pose_cov_values);

    if (verbose_) {
      std::cout << "UPDATE POSE: \n" << pose_meas.to_string() << std::endl;
    }
    // Update
    ekf_wrapper_.update_pose(
      pose_meas,
      pose_cov
    );

    if (verbose_) {
      std::cout << "UPDATE" << std::endl;
      state = ekf_wrapper_.get_state();
      covariance = ekf_wrapper_.get_state_covariance();
      std::cout << "Current state update: \n" << state.to_string() << std::endl;
      std::cout << "Current covariance update: \n" << covariance.to_string_diagonal() <<
        std::endl;
    }
    can_update_ = false;
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
    if (use_measurement_covariances_for_odometry_) {
      for (size_t i = 0; i < 6; i++) {
        pose_covariance_diagonal[i] = msg->pose.covariance[i * 6 + i];
      }
    } else {
      pose_covariance_diagonal = {
        meas_position_cov_, meas_position_cov_, meas_position_cov_,
        meas_orientation_cov_, meas_orientation_cov_, meas_orientation_cov_
      };
    }
    // Odometry twist
    geometry_msgs::msg::TwistStamped::SharedPtr twist_msg =
      std::make_shared<geometry_msgs::msg::TwistStamped>();
    twist_msg->header = msg->header;
    twist_msg->twist = msg->twist.twist;
    std::array<double, 6> twist_covariance_diagonal;
    if (use_measurement_covariances_for_odometry_) {
      for (size_t i = 0; i < 6; i++) {
        twist_covariance_diagonal[i] = msg->twist.covariance[i * 6 + i];
      }
    } else {
      twist_covariance_diagonal = {
        meas_velocity_cov_, meas_velocity_cov_, meas_velocity_cov_,
        0.0, 0.0, 0.0
      };
    }
    if (verbose_) {
      std::cout << "Received odometry message with: " << std::endl;
      std::cout << "Pose:" << std::endl;
      std::cout << " Position: "
                << pose_msg->pose.position.x << ", "
                << pose_msg->pose.position.y << ", "
                << pose_msg->pose.position.z << std::endl;
      std::cout << " Orientation (quaternion): "
                << pose_msg->pose.orientation.w << ", "
                << pose_msg->pose.orientation.x << ", "
                << pose_msg->pose.orientation.y << ", "
                << pose_msg->pose.orientation.z << std::endl;
      std::cout << "Twist:" << std::endl;
      std::cout << " Linear: "
                << twist_msg->twist.linear.x << ", "
                << twist_msg->twist.linear.y << ", "
                << twist_msg->twist.linear.z << std::endl;
      std::cout << " Angular: "
                << twist_msg->twist.angular.x << ", "
                << twist_msg->twist.angular.y << ", "
                << twist_msg->twist.angular.z << std::endl;
    }
    // return;
    // Set earth-map if not already set
    if (!earth_to_map_set_) {
      // As earth to map is not set, we assume that the first pose received is in the earth
      // As it is odometry, we assume that the first pose is at 0,0,0 with 0,0,0 euler
      geometry_msgs::msg::PoseStamped::SharedPtr zero_pose =
        std::make_shared<geometry_msgs::msg::PoseStamped>();
      zero_pose->header = pose_msg->header;
      zero_pose->pose.position.x = 0.0;
      zero_pose->pose.position.y = 0.0;
      zero_pose->pose.position.z = 0.0;
      zero_pose->pose.orientation.w = 1.0;
      zero_pose->pose.orientation.x = 0.0;
      zero_pose->pose.orientation.y = 0.0;
      zero_pose->pose.orientation.z = 0.0;
      generate_map_frame_from_ground_truth_pose(*zero_pose);
      tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
      if (verbose_) {
        std::cout << "Setting earth to map from ground truth" << std::endl;
        std::cout << "Earth to map value (from state estimator): \n"
                  << earth_map_tf.getOrigin().x() << ", "
                  << earth_map_tf.getOrigin().y() << ", "
                  << earth_map_tf.getOrigin().z() << std::endl;
      }
      // Convert orientation to euler
      Eigen::Quaterniond q(
        pose_msg->pose.orientation.w,
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = quaternionToEuler(q);

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
      if (verbose_) {
        std::cout << "Skipping update to avoid updating too fast" << std::endl;
      }
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
        quaternionToEuler(
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

      if (verbose_) {
        std::cout << "T_earth_odom: \n" << T_earth_odom << std::endl;
        std::cout << "T_map_odom: \n" << T_map_odom << std::endl;
      }
    }
    // Prepare velocity measurement
    ekf::VelocityMeasurement velocity_meas;
    Eigen::Quaterniond q(
      pose_msg->pose.orientation.w,
      pose_msg->pose.orientation.x,
      pose_msg->pose.orientation.y,
      pose_msg->pose.orientation.z);
    q.normalize();
    auto euler_odom_base = quaternionToEuler(q);
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

    // Transform from odom-base_link to map-base_link
    Eigen::Matrix4d T_map_base = ekf_wrapper_.get_T_a_c(
      pos_odom_base,
      ori_odom_base,
      T_map_odom);

    Eigen::Vector<double, 7> pose_map_base = ekf::EKFWrapper::transform_to_pose(T_map_base);

    Eigen::Vector3d euler_map_base = quaternionToEuler(
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
    Eigen::Vector3d corrected_euler_map_base = correct_measured_euler(
      Eigen::Vector3d(
        state_euler_array[0],
        state_euler_array[1],
        state_euler_array[2]),
      euler_map_base);

    // Correct velocity to be in map frame
    // Rotate from base_link to map frame
    Eigen::Matrix3d R_map_base = T_map_base.block<3, 3>(0, 0);
    Eigen::Vector3d vel_base = Eigen::Vector3d(
      twist_msg->twist.linear.x,
      twist_msg->twist.linear.y,
      twist_msg->twist.linear.z);
    Eigen::Vector3d vel_map_base = R_map_base * vel_base;

    std::array<double, ekf::VelocityMeasurement::size> velocity_values = {
      vel_map_base[0],   // vx
      vel_map_base[1],   // vy
      vel_map_base[2]   // vz
    };
    velocity_meas.set(velocity_values);

    // Prepare pose Covariance
    ekf::VelocityMeasurementCovariance velocity_cov;
    std::array<double, ekf::VelocityMeasurementCovariance::size> velocity_cov_values;
    velocity_cov_values[0] = twist_covariance_diagonal[0];   // vx
    velocity_cov_values[1] = twist_covariance_diagonal[1];   // vy
    velocity_cov_values[2] = twist_covariance_diagonal[2];   // vz
    velocity_cov.set(velocity_cov_values);

    if (verbose_) {
      std::cout << "UPDATE POSE VELOCITY: \n" << velocity_meas.to_string() << std::endl;
      std::cout << "Pose: \n" << pose_map_base << std::endl;
    }

    ekf_wrapper_.update_velocity(
      velocity_meas,
      velocity_cov
    );    // Update

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
    ekf_wrapper_.update_pose(z, measurement_noise_covariance);

    if (verbose_) {
      std::cout << "UPDATE" << std::endl;
      state = ekf_wrapper_.get_state();
      covariance = ekf_wrapper_.get_state_covariance();
      std::cout << "Current state update: \n" << state.to_string() << std::endl;
      std::cout << "Current covariance update: \n" << covariance.to_string_diagonal() <<
        std::endl;
    }
    can_update_ = false;
  }


  void timer_callback()
  {
    // Publish earth map
    // if (!earth_to_map_set_) {
    //   return;
    // } else {
    tf2::Transform earth_map_tf = state_estimator_interface_->getEarthToMapTransform();
    state_estimator_interface_->setEarthToMap(earth_map_tf, node_ptr_->now(), true);
    // }

    // Get current state and covariance
    ekf::State state = ekf_wrapper_.get_state();
    ekf::Covariance covariance = ekf_wrapper_.get_state_covariance();

    // Get odom to base_link transform
    Eigen::Matrix4d odom_to_baselink_eigen =
      ekf_wrapper_.get_T_b_c(
      Eigen::Vector3d(state.get_position().data()),
      Eigen::Vector3d(state.get_orientation().data()),
      ekf_wrapper_.get_map_to_odom());

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

    // Get map to odom transform
    Eigen::Matrix4d map_to_odom_eigen = ekf_wrapper_.get_map_to_odom();

    // Transform from eigen to position + quaternion
    Eigen::Vector<double, 7> map_to_odom_values =
      ekf::EKFWrapper::transform_to_pose(map_to_odom_eigen);

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

    // Prepare twist to update the state estimator
    geometry_msgs::msg::TwistWithCovariance twist;
    // Get map to base_link transform
    Eigen::Matrix4d map_to_baselink_eigen =
      ekf_wrapper_.get_T_b_c(
      Eigen::Vector3d(state.get_position().data()),
      Eigen::Vector3d(state.get_orientation().data()),
      Eigen::Matrix4d::Identity());
    // Transform linear velocity from earth to base_link
    Eigen::Vector3d vel_map = Eigen::Vector3d(
      state.data[ekf::State::VX],
      state.data[ekf::State::VY],
      state.data[ekf::State::VZ]);
    Eigen::Matrix3d R_map_base = map_to_baselink_eigen.block<3, 3>(0, 0);
    Eigen::Vector3d vel_base = R_map_base.transpose() * vel_map;

    // Linear and angular velocities
    twist.twist.linear.x = vel_base[0];
    twist.twist.linear.y = vel_base[1];
    twist.twist.linear.z = vel_base[2];
    twist.twist.angular.x = last_angular_velocity_[0];
    twist.twist.angular.y = last_angular_velocity_[1];
    twist.twist.angular.z = last_angular_velocity_[2];
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
    // TODO: imu covariance for angular speed? for twist in base_link frame

    // Update twist in base_link frame in the state estimator
    // state_estimator_interface_->setTwistInBaseFrame(
    //   twist,
    //   msg->header.stamp);

    // geometry_msgs::msg::TwistWithCovariance last_twist;
    // last_twist.twist = last_twist_msg_.twist;
    // last_twist.covariance = twist.covariance;   // use the same covariance as computed
    // last_twist.twist.linear = twist.twist.linear;   // use the same linear velocity as computed
    // last_twist.twist.angular = twist.twist.angular;   // use the same angular velocity as computed
    state_estimator_interface_->setTwistInBaseFrame(
      twist,
      node_ptr_->now());

    // std::cout << "Current state timer: \n" << state.to_string() << std::endl;
    // std::cout << "Current covariance timer: \n" << covariance.to_string_diagonal() << std::endl;

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

  // void twist_callback(
  //   const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  // {
  //   last_twist_msg_ = *msg;
  // }
};        // class ekf_fuse
}       // namespace ekf_fuse
#endif  // EKF_FUSE_HPP_
