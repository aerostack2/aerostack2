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
* @file plugin_wrapper.cpp
*
* Implementation of the PluginWrapper class
*
* @authors Miguel Fernández Cortizas
*/

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <iostream>

#include "as2_state_estimator/plugin_wrapper.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_state_estimator/as2_state_estimator.hpp"
#include "as2_state_estimator/plugin_wrapper_interface.hpp"

namespace as2_state_estimator
{

bool PluginWrapper::filterTransformRule(const geometry_msgs::msg::TransformStamped & transform)
{
  if (transform.header.frame_id == StateEstimator::getEarthFrame() &&
    transform.child_frame_id == StateEstimator::getMapFrame())
  {
    return false;
  }
  if (transform.header.frame_id == StateEstimator::getMapFrame() &&
    transform.child_frame_id == StateEstimator::getOdomFrame())
  {
    return false;
  }
  if (transform.header.frame_id == StateEstimator::getOdomFrame() &&
    transform.child_frame_id == StateEstimator::getBaseFrame())
  {
    return false;
  }
  return true;
}


std::optional<PluginWrapper::SharedPtr> PluginWrapper::create(
  const std::string & plugin_name,
  std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>> loader)
{
  PluginWrapper::SharedPtr wrapper = std::make_shared<PluginWrapper>();
  wrapper->plugin_name_ = plugin_name;
  std::string plugin_name_complete = plugin_name + "::Plugin";
  auto state_estimator = StateEstimator::getInstance();

  try {
    wrapper->plugin_ptr = loader->createSharedInstance(plugin_name_complete);
    wrapper->interface = std::make_shared<PluginWrapperInterface>(
      wrapper.get());

    std::vector<std::function<bool(const geometry_msgs::msg::TransformStamped &)>> filter_rules;
    filter_rules.push_back(
      std::bind(
        &PluginWrapper::filterTransformRule, wrapper.get(),
        std::placeholders::_1));
    wrapper->tf_handler = std::make_shared<as2::tf::TfHandler>(
      state_estimator.get(),
      filter_rules);
    wrapper->plugin_ptr->setup(
      state_estimator.get(), wrapper->tf_handler, wrapper->interface);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_FATAL(state_estimator->get_logger(), "Failed to load plugin: %s", e.what());
    return std::nullopt;
  }
  wrapper->plugin_pose_pub =
    state_estimator->create_publisher<geometry_msgs::msg::PoseStamped>(
    std::string("state_estimation/") + plugin_name + "/pose", 10);
  wrapper->plugin_twist_pub =
    state_estimator->create_publisher<geometry_msgs::msg::TwistStamped>(
    std::string("state_estimation/") + plugin_name + "/twist", 10);

  RCLCPP_INFO(state_estimator->get_logger(), "Loaded plugin %s", plugin_name.c_str());


  return wrapper;
}


void PluginWrapper::advertiseUpdate(TransformInformatonType type)
{
  if (type == TransformInformatonType::TWIST_IN_BASE) {
    plugin_twist_pub->publish(robot_state_.getTwistStampedInBase());
  } else {
    plugin_pose_pub->publish(robot_state_.getPoseStampedEarthToBase());
  }
  StateEstimator::getInstance()->receiveStateUpdate(plugin_name_, type);

}


}  // namespace as2_state_estimator
