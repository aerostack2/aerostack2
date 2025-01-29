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
* @file as2_state_estimator.cpp
*
* An state estimation server for AeroStack2 implementation
*
* @authors Miguel Fernández Cortizas
*          David Pérez Saura
*          Rafael Pérez Seguí
*          Pedro Arias Pérez
*          Javier Melero Deza
*/

#include <functional>
#include <memory>
#include <vector>
#include <string>

#include "as2_state_estimator/as2_state_estimator.hpp"
#include "as2_state_estimator/state_estimator_metacontroller.hpp"

namespace as2_state_estimator
{

StateEstimator::StateEstimator(const rclcpp::NodeOptions & options)
: as2::Node("state_estimator", get_modified_options(options))
{
  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>(
    "as2_state_estimator", "as2_state_estimator_plugin_base::StateEstimatorBase");
  declareRosInterfaces();
  readParameters();
  std::vector<std::string> plugin_names;
  // the plugin_names parameter, can be a single string or a list of strings
  // if it is a single string, we add it to the list, otherwise we get the list

  try {
    std::string plugin_name;
    this->get_parameter<std::string>("plugin_name", plugin_name);
    plugin_names.push_back(plugin_name);
  } catch (std::exception & e) {
    RCLCPP_INFO(
      this->get_logger(), "Parameter <plugin_name> not defined or malformed: %s",
      e.what());
    try {
      std::vector<std::string> plugin_names_list;
      this->get_parameter<std::vector<std::string>>("plugin_name", plugin_names_list);
      for (const auto & plugin_name : plugin_names_list) {
        RCLCPP_INFO(this->get_logger(), "Loading plugin %s", plugin_name.c_str());
        plugin_names.push_back(plugin_name);
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
        e.what());
      this->~StateEstimator();
    }
  }

  // assert there is at least one plugin
  if (plugin_names.empty()) {
    RCLCPP_FATAL(
      this->get_logger(),
      "No plugins to load, check that the parameter <plugin_name> is set, as a string or a list of strings");
    this->~StateEstimator();
  }

  for (const auto & plugin_name : plugin_names) {
    bool out = loadPlugin(plugin_name);
    if (!out) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load plugin %s", plugin_name.c_str());
      this->~StateEstimator();
    }
  }
  publish_initial_transforms();
}

bool StateEstimator::loadPlugin(const std::string & _plugin_name)
{
  std::string plugin_name = _plugin_name + "::Plugin";
  as2_state_estimator_plugin_base::StateEstimatorBase::SharedPtr plugin_ptr;
  std::vector<std::function<bool(const geometry_msgs::msg::TransformStamped &)>> filter_rules;
  filter_rules.push_back(
    std::bind(
      &StateEstimator::filterTransformRule, this,
      std::placeholders::_1));

  if (plugins_.find(plugin_name) != plugins_.end()) {
    RCLCPP_WARN(this->get_logger(), "Plugin %s already loaded", plugin_name.c_str());
    return true;
  }
  try {
    plugin_ptr = loader_->createSharedInstance(plugin_name);
    state_estimator_interfaces_.insert(
      std::make_pair(
        plugin_name,
        std::make_shared<MetacontrollerInterface>(
          this,
          plugin_name)));
    tf_handlers_.insert(
      std::make_pair(
        plugin_name,
        std::make_shared<as2::tf::TfHandler>(
          this,
          filter_rules)));

    plugin_ptr->setup(this, tf_handlers_[plugin_name], state_estimator_interfaces_[plugin_name]);
    plugins_[plugin_name] = plugin_ptr;
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
    return false;
  }
  registerPlugin(plugin_name);
  RCLCPP_INFO(this->get_logger(), "Loaded plugin %s", plugin_name.c_str());
  return true;
}

void StateEstimator::publish_initial_transforms()
{
  // Publish the initial transforms
  for (int type = 0; type < 3; type++) {
    auto [parent_frame,
      child_frame] = getFramesFromType(static_cast<TransformInformatonType>(type));
    publishTransform(
      transforms_[type], parent_frame, child_frame, this->now(), true);
  }
}

rclcpp::NodeOptions StateEstimator::get_modified_options(const rclcpp::NodeOptions & options)
{
  // Create a copy of the options and modify it
  rclcpp::NodeOptions modified_options = options;
  modified_options.allow_undeclared_parameters(true);
  modified_options.automatically_declare_parameters_from_overrides(true);
  return modified_options;
}

void StateEstimator::processPose(
  const std::string & authority,
  const geometry_msgs::msg::PoseWithCovariance & msg,
  const as2_state_estimator::TransformInformatonType & type,
  const builtin_interfaces::msg::Time & stamp,
  bool is_static)
{
  auto[parent_frame, child_frame] = getFramesFromType(type);
  publishTransform(msg, parent_frame, child_frame, stamp, is_static);
  // update the transform
  tf2::Transform & transform = transforms_[static_cast<int>(type)];
  tf2::fromMsg(msg.pose, transform);
}
void StateEstimator::processTwist(
  const std::string & authority,
  const geometry_msgs::msg::TwistWithCovariance & msg,
  const builtin_interfaces::msg::Time & stamp)
{
  // RCLCPP_INFO(this->get_logger(), "Processing Twist, authority: %s", authority.c_str());
  publishTwist(msg, stamp);
}

void StateEstimator::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const builtin_interfaces::msg::Time & stamp,
  bool is_static)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;
  tf_msg.transform = tf2::toMsg(transform);
  if (is_static) {
    tf_msg.header.stamp = builtin_interfaces::msg::Time();
    tfstatic_broadcaster_->sendTransform(tf_msg);
    return;
  }
  tf_broadcaster_->sendTransform(tf_msg);
}

void StateEstimator::publishTransform(
  const geometry_msgs::msg::PoseWithCovariance & pose, const std::string & parent_frame,
  const std::string & child_frame, const builtin_interfaces::msg::Time & stamp, bool is_static)
{
  tf2::Transform transform;
  tf2::fromMsg(pose.pose, transform);
  publishTransform(transform, parent_frame, child_frame, stamp, is_static);
}


}  // namespace as2_state_estimator
