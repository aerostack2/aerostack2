/*!*******************************************************************************************
 *  \file       ground_truth.hpp
 *  \brief      An state estimation plugin ground truth for AeroStack2
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

#ifndef __GROUND_TRUTH_H__
#define __GROUND_TRUTH_H__

#include <as2_core/utils/tf_utils.hpp>
#include "as2_state_estimator/plugin_base.hpp"
namespace ground_truth {

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  bool using_ignition_tf_ = false;

public:
  Plugin() : as2_state_estimator_plugin_base::StateEstimatorBase(){};
  void on_setup() override {
    pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
    twist_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos,
        std::bind(&Plugin::twist_callback, this, std::placeholders::_1));

    // publish static transform from earth to map and map to odom
    geometry_msgs::msg::TransformStamped earth_to_map =
        as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    geometry_msgs::msg::TransformStamped map_to_odom =
        as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);

    // TODO: CHECK IF WE NEED TO PUBLISH THIS PERIODICALLY
    if (node_ptr_->has_parameter("use_ignition_tf")) {
      node_ptr_->get_parameter("use_ignition_tf", using_ignition_tf_);
      if (using_ignition_tf_) RCLCPP_INFO(node_ptr_->get_logger(), "Using ignition tfs");
    }
    publish_static_transform(earth_to_map);
    publish_static_transform(map_to_odom);
  };

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // since it's ground_truth we consider that the frame obtained is the world frame (earth)
    // so we need to publish the transform from world to base_link, with map and odom as the
    // identity transform in order to keep the continuity of the tf tree we will modify the
    // odom->base_link transform

    if (msg->header.frame_id != get_earth_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Received pose in frame %s, expected %s",
                   msg->header.frame_id.c_str(), get_earth_frame().c_str());
      return;
    }

    auto transform            = geometry_msgs::msg::TransformStamped();
    transform.header.stamp    = msg->header.stamp;
    transform.header.frame_id = get_odom_frame();
    if (using_ignition_tf_) {
      transform.child_frame_id = as2::tf::generateTfName("", node_ptr_->get_namespace());
    } else {
      transform.child_frame_id = get_base_frame();
    }
    // transform.child_frame_id          = get_base_frame();
    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;

    transform.transform.rotation.x = msg->pose.orientation.x;
    transform.transform.rotation.y = msg->pose.orientation.y;
    transform.transform.rotation.z = msg->pose.orientation.z;
    transform.transform.rotation.w = msg->pose.orientation.w;

    publish_transform(transform);

    // Pose should be published in the earth frame (world)
    // so we dont need to modify the frame_id
    publish_pose(*msg);
  };

  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (msg->header.frame_id != get_base_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Received twist in frame %s, expected %s",
                   msg->header.frame_id.c_str(), get_base_frame().c_str());
      // TODO: convert it to the base_link frame if needed
      return;
    }
    publish_twist(*msg);
  };
};

};      // namespace ground_truth
#endif  // __GROUND_TRUTH_HPP__
