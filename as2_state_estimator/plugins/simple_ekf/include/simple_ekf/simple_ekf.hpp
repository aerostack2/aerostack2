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
* The filtering itself is simple_ekf_core::Filter, which knows nothing about ROS. This is
* the node's side of it: parameters, subscriptions, the clock, and publishing the tree the
* filter produces.
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef SIMPLE_EKF__SIMPLE_EKF_HPP_
#define SIMPLE_EKF__SIMPLE_EKF_HPP_

#include <array>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <as2_msgs/msg/platform_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <simple_ekf_core/filter.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "simple_ekf/ros_conversions.hpp"
#include "simple_ekf/topic_config.hpp"

namespace simple_ekf
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
public:
  /**
   * @brief Read the parameters, build the filter and subscribe to every configured topic.
   */
  void onSetup() override;

  std::vector<as2_state_estimator::TransformInformatonType> getTransformationTypesAvailable() const
  override;

private:
  using SourceId = simple_ekf_core::SourceId;

  std::unique_ptr<simple_ekf_core::Filter> filter_;

  bool verbose_ = false;
  bool debug_verbose_ = false;
  bool use_arm_ = false;
  bool set_earth_map_from_topic_ = false;
  bool earth_to_map_static_tf_ = true;

  // The state estimator's own frame ids, which every measurement's frame id is matched against
  FrameIds frame_ids_;
  // The (topic, frame id) pairs whose frame had to be guessed, each reported once
  std::set<std::pair<std::string, std::string>> guessed_frames_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> update_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Raw, pre-smoothing state, published only when internal_ekf_debug_topics is not empty
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr internal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr internal_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr internal_map_to_odom_pub_;

  simple_ekf_core::Config readFilterConfig();
  TopicConfig readTopicConfig(const std::string & topic_id);

  /**
   * @brief Read a flag whose default depends on the topic's message type, and log where
   *        its value came from.
   */
  bool readTypeDefaultedFlag(
    const std::string & topic_id, const std::string & key,
    bool default_value);

  /**
   * @brief Read a three-value list parameter, falling back to the default on a wrong length.
   */
  std::array<double, 3> readTriple(
    const std::string & name, const std::array<double, 3> & default_value);

  void subscribe(const TopicConfig & config, SourceId source);
  simple_ekf_core::LogSink makeLogSink() const;
  simple_ekf_core::Nanoseconds nowNanoseconds() const;

  /**
   * @brief Publish earth->map and an identity map->odom, and let the filter start.
   *
   * Does nothing once the filter has started.
   */
  void startEstimation();

  void publishState();

  /**
   * @param stamp Shared with the external state, so both line up exactly when plotted
   */
  void publishInternalDebugState(const builtin_interfaces::msg::Time & stamp);

  void logFilterState(const std::string & context) const;

  /**
   * @brief Set earth->map from the first pose of a topic that provides it.
   *
   * Only a pose in the earth frame or in the map frame can: see
   * simple_ekf_core::Filter::setEarthToMapFromFirstPose.
   *
   * @return true if earth->map was set
   */
  bool setEarthToMapFromFirstPose(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id);

  /**
   * @brief Which of the filter's frames a measurement's frame id names.
   *
   * The state estimator's frame of that exact name. An id that is none of them is guessed from
   * the words in it, as every id was before, with a warning the first time a topic uses it.
   *
   * @param guess guessPoseSourceFrame or guessTwistSourceFrame
   */
  simple_ekf_core::SourceFrame resolveFrame(
    const std::string & frame_id, const std::string & topic,
    simple_ekf_core::SourceFrame (* guess)(const std::string &));

  /**
   * @brief Fuse a pose, or use it to set earth->map if that is still unknown.
   *
   * @param msg The pose, with the covariance its topic is believed with already applied
   */
  void fusePose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    const TopicConfig & config, SourceId source);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  /**
   * @brief Republish a dynamic earth->map and advance the filter's tick.
   */
  void timerCallback();

  void poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const TopicConfig & config, SourceId source);
  void poseWithCovarianceCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    const TopicConfig & config, SourceId source);

  /**
   * @brief Fuse the pose of an odometry message. Its twist is ignored.
   */
  void odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const TopicConfig & config, SourceId source);

  /**
   * @brief Fuse the pose of the configured rigid body, which mocap reports in the earth frame.
   */
  void mocapCallback(
    const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg,
    const TopicConfig & config, SourceId source);

  void twistWithCovarianceCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
    const TopicConfig & config, SourceId source);
};

}  // namespace simple_ekf

#endif  // SIMPLE_EKF__SIMPLE_EKF_HPP_
