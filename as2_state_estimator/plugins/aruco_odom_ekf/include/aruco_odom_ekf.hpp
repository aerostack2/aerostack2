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
* @file aruco_odom_ekf.hpp
*
* An state estimation plugin based on an EKF that fuses ArUco markers and odometry
*
* @authors Pedro Arias Pérez
*/

#ifndef ARUCO_ODOM_EKF_HPP_
#define ARUCO_ODOM_EKF_HPP_

#include <utility>
#include <regex>
#include <memory>

#include <as2_core/names/services.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_state_estimator/plugin_base.hpp"

namespace aruco_odom_ekf
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  bool earth_to_map_set_ = false;
  geometry_msgs::msg::TransformStamped earth_to_map_;

  bool using_gazebo_tf_ = false;

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void on_setup() override;

private:
  void generate_map_frame_from_ground_truth_pose(const geometry_msgs::msg::PoseStamped & pose);

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

};      // class Plugin
}       // namespace aruco_odom_ekf
#endif  // ARUCO_ODOM_EKF_HPP_
