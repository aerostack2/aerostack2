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
* @file ground_truth.hpp
*
* An state estimation plugin ground truth for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef GROUND_TRUTH_HPP_
#define GROUND_TRUTH_HPP_

#include <string>
#include <vector>
#include <geographic_msgs/msg/geo_point.hpp>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/gps_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include <as2_msgs/srv/set_origin.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/utils/conversions.hpp"
#include "as2_core/names/topics.hpp"


namespace ground_truth
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_sub_;

  bool integrate_pose_for_twist_ = false;
  double twist_smooth_filter_cte_ = 0.5;

  std::string rigid_body_name_ = "";

  // bool using_gazebo_tf_ = false;
  bool set_earth_map_manually_ = false;
  bool earth_to_map_set_ = false;
  bool map_to_odom_set_ = false;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  tf2::Transform earth_to_baselink_ = tf2::Transform::getIdentity();
  tf2::Transform odom_to_baselink_ = tf2::Transform::getIdentity();

  geometry_msgs::msg::TwistWithCovariance twist_with_covariance_msg_;

  // State variables for twist computation
  tf2::Vector3 last_pose_{0, 0, 0};
  tf2::Vector3 last_vel_{0, 0, 0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool first_pose_received_ = false;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

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
   * @brief Setup the TF tree by initializing earth-to-map and map-to-odom transforms
   */
  void setupTfTree();

  /**
   * @brief Generate the initial odom frame as an identity transform
   *
   * @return geometry_msgs::msg::PoseWithCovariance Identity pose for map-to-odom transform
   */
  geometry_msgs::msg::PoseWithCovariance generateInitialOdomFrame();

  /**
   * @brief Compute twist from pose by numerical differentiation
   *
   * Differentiates position in earth frame and transforms the resulting velocity
   * to base frame using the current orientation. Applies low-pass filtering.
   *
   * @param pose Current pose measurement
   * @return const geometry_msgs::msg::TwistWithCovariance& Computed twist in base frame
   */
  const geometry_msgs::msg::TwistWithCovariance & computeTwistFromPose(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Check if two poses are the same within a threshold
   *
   * @param pose1 First pose to compare
   * @param pose2 Second pose to compare
   * @param position_threshold Distance threshold for considering poses equal (default: 1e-6)
   * @return true if poses are within threshold, false otherwise
   */
  bool isSamePose(
    const tf2::Vector3 & pose1, const tf2::Vector3 & pose2,
    double position_threshold = 1e-6);

  /**
   * @brief Process a pose message and update state estimation
   *
   * Converts earth-to-baselink transform to odom-to-baselink and publishes it.
   * Optionally computes and publishes twist if integration is enabled.
   *
   * @param msg Pose message in earth frame
   */
  void processPose(const geometry_msgs::msg::PoseStamped & msg);

  /**
   * @brief Callback for pose topic subscription
   *
   * @param msg Pose message from topic
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback for mocap rigid bodies topic subscription
   *
   * Extracts the pose of the specified rigid body and processes it.
   *
   * @param msg Rigid bodies message from mocap system
   */
  void mocapCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);

  /**
   * @brief Callback for twist topic subscription
   *
   * @param msg Twist message from topic
   */
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};      // class GroundTruth
}       // namespace ground_truth
#endif  // GROUND_TRUTH_HPP_
