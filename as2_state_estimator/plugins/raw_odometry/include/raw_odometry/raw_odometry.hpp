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
* An state estimation plugin raw odometry for AeroStack2
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
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <as2_core/utils/tf_utils.hpp>

#include "as2_state_estimator/plugin_base.hpp"
#include "as2_core/names/topics.hpp"

namespace raw_odometry
{

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  bool set_earth_map_manually_ = false;
  bool earth_to_map_set_ = false;
  bool map_to_odom_set_ = false;
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  /**
   * @brief Setup the raw odometry plugin
   *
   * Configures the plugin by reading parameters and creating subscription
   * to the odometry topic.
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
   * @brief Callback for odometry topic subscription
   *
   * Processes incoming odometry messages by extracting pose and twist information
   * and publishing them to the state estimator interface.
   *
   * @param msg Odometry message from topic
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};      // class Plugin
}       // namespace raw_odometry
#endif  // RAW_ODOMETRY_HPP_
