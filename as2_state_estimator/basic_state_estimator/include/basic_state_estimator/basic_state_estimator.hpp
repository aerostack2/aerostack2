/*!*******************************************************************************************
 *  \file       basic_state_estimator.hpp
 *  \brief      A basic state estimator for AeroStack2
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef BASIC_STATE_ESTIMATOR_HPP_
#define BASIC_STATE_ESTIMATOR_HPP_

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define FRAME_RECTIFIED_TOPIC "rectified_localization/pose"

class BasicStateEstimator : public as2::Node {
public:
  BasicStateEstimator();

  void setupNode();
  void cleanupNode();
  void setupTfTree();
  void run();
  void getStartingPose(const std::string &_earth_frame, const std::string &_map);
  void updateOdomTfDrift(const geometry_msgs::msg::TransformStamped &_odom2baselink,
                         const geometry_msgs::msg::TransformStamped &_map2baselink);
  void updateRefTfRectification(const geometry_msgs::msg::Transform &_frame_rectified_tf_,
                                const geometry_msgs::msg::Transform &_ref2frame_rectified_tf_);
  geometry_msgs::msg::TransformStamped calculateLocalization();
  void publishTfs();
  void publishStaticTfs();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gt_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rectified_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimated_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_estimated_pub_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void gtTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg);
  void rectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);

  std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
  geometry_msgs::msg::TransformStamped map2odom_tf_;
  geometry_msgs::msg::TransformStamped odom2baselink_tf_;
  geometry_msgs::msg::TransformStamped frame_rectified_tf_;
  geometry_msgs::msg::TransformStamped ref2ref_rectified_tf_;
  geometry_msgs::msg::TwistStamped odom_twist_;
  geometry_msgs::msg::PoseStamped gt_pose_stamped_;
  geometry_msgs::msg::TwistStamped gt_twist_stamped_;
  geometry_msgs::msg::PoseStamped rectified_pose_;
  geometry_msgs::msg::PoseStamped global_ref_pose_;
  geometry_msgs::msg::TwistStamped global_ref_twist_;  // TODO:Review

  bool odom_only_;
  bool ground_truth_;
  bool sensor_fusion_;
  bool rectified_localization_;
  bool start_run_;

  void getGlobalRefState();

  std::string global_ref_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string baselink_frame_;

  std::string frame_rectified_;
  std::string frame_global_pose_;
  std::string frame_global_twist_;

  void publishStateEstimation();
  geometry_msgs::msg::PoseStamped generatePoseStampedMsg(const rclcpp::Time &_timestamp);
  geometry_msgs::msg::TwistStamped generateTwistStampedMsg(const rclcpp::Time &_timestamp);

  rclcpp::Time last_info_time_;
  rclcpp::Time tf_publish_time_;

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};

#endif  // BASIC_STATE_ESTIMATOR_HPP_
