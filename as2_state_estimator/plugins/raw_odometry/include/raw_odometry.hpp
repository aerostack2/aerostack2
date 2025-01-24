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
* @file raw_odometry.hpp
*
* An state estimation plugin external odom for AeroStack2
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef RAW_ODOMETRY_HPP_
#define RAW_ODOMETRY_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_core/names/topics.hpp"

namespace raw_odometry
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void onSetup() override
  {
    std::string odom_topic = as2_names::topics::sensor_measurements::odom;
    node_ptr_->get_parameter("odom_topic", odom_topic);
    odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, as2_names::topics::sensor_measurements::qos,
      std::bind(&Plugin::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::UniquePtr msg)
  {
    // odom should have frame_id = odom and child_frame_id = base_link
    // since we only have this message for generating the tf tree we will publish the transform
    // from odom to base_link directly and the transform from earth to map and map to odom  will
    // be the identity transform
    if (msg->header.frame_id != state_estimator_interface_->getOdomFrame()) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(), "Received odom in frame %s, expected %s. "
        "frame_id changed to expected one",
        msg->header.frame_id.c_str(), state_estimator_interface_->getOdomFrame().c_str());
      return;
    }
    if (msg->child_frame_id != state_estimator_interface_->getBaseFrame()) {
      RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Received odom child_frame_id  in frame %s, expected %s. "
        "child_frame_id changed to expected one",
        msg->child_frame_id.c_str(), state_estimator_interface_->getBaseFrame().c_str());
      return;
    }

    auto pose_cov = geometry_msgs::msg::PoseWithCovariance();
    auto twist_cov = geometry_msgs::msg::TwistWithCovariance();

    pose_cov.pose = msg->pose.pose;
    pose_cov.covariance = msg->pose.covariance;
    state_estimator_interface_->setOdomToBaseLinkPose(pose_cov, msg->header.stamp);
    twist_cov.twist = msg->twist.twist;
    twist_cov.covariance = msg->twist.covariance;
    state_estimator_interface_->setTwistInBaseFrame(twist_cov, msg->header.stamp);
  }
};

}  // namespace raw_odometry

#endif  // RAW_ODOMETRY_HPP_
