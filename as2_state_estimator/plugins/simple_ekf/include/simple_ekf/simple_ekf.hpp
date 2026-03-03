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
* @file simple_ekf.hpp
*
* An state estimation plugin simple_ekf for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef SIMPLE_EKF_HPP_
#define SIMPLE_EKF_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/utils/conversions.hpp"
#include "as2_core/names/topics.hpp"

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>

#include "simple_ekf/simple_ekf_utils.hpp"


namespace simple_ekf
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  bool verbose_ = false;
  bool debug_verbose_ = false;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr predict_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> update_pose_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
  update_pose_cov_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> update_odom_subs_;
  std::vector<rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr> update_mocap_subs_;

  // Pose topic configurations
  std::vector<PoseTopicConfig> update_pose_configs_;

  bool set_earth_map_manually_ = false;
  bool set_earth_map_from_topic_ = false;
  bool earth_to_map_set_ = false;
  bool map_to_odom_set_ = false;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_baselink_ = tf2::Transform::getIdentity();
  geometry_msgs::msg::TwistWithCovariance twist_in_base_ =
    geometry_msgs::msg::TwistWithCovariance();

  // Last imu message
  sensor_msgs::msg::Imu last_imu_msg_;

  // Last mocap pose per rigid body name — used for isSamePose duplicate detection
  std::map<std::string, tf2::Vector3> last_mocap_pose_;

  // EKF wrapper
  ekf::EKFWrapper ekf_wrapper_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase()
  {}

  /**
   * @brief Setup the ground truth plugin
   *
   * Configures the plugin by reading parameters and creating subscriptions
   * to pose/mocap and twist topics based on the configuration.
   */
  void onSetup() override;

  /**
   * @brief Get the list of transformation types provided by this plugin
   *
   * @return std::vector<as2_state_estimator::TransformInformatonType> List of available transformations
   */
  std::vector<as2_state_estimator::TransformInformatonType> getTransformationTypesAvailable() const
  override;

private:
  /**
   * @brief Initialize the EKF wrapper with initial covariance, gravity and IMU noise parameters
   *
   * Reads all required parameters from node_ptr_ and calls reset(),
   * set_gravity() and set_noise_parameters() on the EKF wrapper.
   * Can also be called externally to re-initialize the filter.
   */
  void setupWrapper();

  /**
   * @brief Setup the TF tree by initializing earth-to-map and map-to-odom transforms
   */
  void setupTfTree();

  /**
   * @brief Publish the current transforms and twist
   */
  void publishState();

  /**
   * @brief Update all transforms and twist from the current EKF wrapper state
   */
  void updateStateFromEkf();

  /**
   * @brief Process IMU data for prediction step
   */
  void processImu(const sensor_msgs::msg::Imu & msg);

  /**
   * @brief Process a pose
   *
   * @param msg Pose message to process
   * @param is_odom Whether this pose comes from an odometry topic
   */
  void processPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg, bool is_odom = false);

  /**
   * @brief Callback for IMU topic subscription
   *
   * @param msg IMU message from topic
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Callback for pose topic subscription
   *
   * @param msg Pose message from topic
   * @param config Configuration for this pose topic
   */
  void poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const PoseTopicConfig & config);

  /**
   * @brief Callback for pose with covariance topic subscription
   *
   * @param msg Pose with covariance message from topic
   * @param config Configuration for this pose topic
   */
  void poseWithCovarianceCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    const PoseTopicConfig & config);

  /**
   * @brief Callback for odometry topic subscription
   *
   * Extracts the pose with covariance from the odometry message and processes it,
   * ignoring the twist part of the odometry.
   *
   * @param msg Odometry message from topic
   * @param config Configuration for this pose topic
   */
  void odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const PoseTopicConfig & config);

  /**
   * @brief Callback for mocap rigid bodies topic subscription
   *
   * Finds the rigid body matching config.rigid_body_name, converts its pose to a
   * PoseStamped in the earth frame and forwards it to the standard pose pipeline.
   *
   * @param msg Rigid bodies message from mocap system
   * @param config Configuration for this pose topic (includes rigid_body_name)
   */
  void mocapCallback(
    const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
    const PoseTopicConfig & config);
};      // class SIMPLE_EKF
}       // namespace simple_ekf
#endif  // SIMPLE_EKF_HPP_
