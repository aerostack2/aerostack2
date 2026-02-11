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
* @file plugin_wrapper.hpp
*
* A wrapper for the plugins in the state estimation server for AeroStack2, focused in easing the
* implementation of metacontrol layers
*
* @authors Miguel Fernández Cortizas
*/

#ifndef AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_
#define AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_

#include <tf2/LinearMath/Transform.h>

#include <optional>
#include <string>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>

#include "as2_state_estimator/robot_state.hpp"
#include "as2_state_estimator/plugin_base.hpp"

using StateComponent = std::variant<
  geometry_msgs::msg::PoseWithCovariance,
  geometry_msgs::msg::TwistWithCovariance>;

namespace as2_state_estimator
{

class PluginWrapper
{
public:
  using SharedPtr = std::shared_ptr<PluginWrapper>;
  std::string plugin_name_;
  std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase> plugin_ptr;

  static std::optional<PluginWrapper::SharedPtr> create(
    const std::string & plugin_name,
    std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>> loader);

  bool filterTransformRule(const geometry_msgs::msg::TransformStamped & transform);

  friend class PluginWrapperInterface;

  void advertiseUpdate(TransformInformatonType type);
  PluginWrapper() {}
  ~PluginWrapper()
  {
    // assert is the only reference to the plugin before deleting it
    assert(plugin_ptr.use_count() == 1);
    plugin_ptr.reset();
  }

  RobotState robot_state_;

private:
  std::shared_ptr<StateEstimatorInterface> interface;
  std::shared_ptr<as2::tf::TfHandler> tf_handler;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr plugin_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr plugin_twist_pub;

};


}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_
