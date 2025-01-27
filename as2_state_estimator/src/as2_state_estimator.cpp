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
  std::vector<std::string> plugin_names;
  // the plugin_names parameter, can be a single string or a list of strings
  // if it is a single string, we add it to the list, otherwise we get the list

  try {
    std::string plugin_name;
    this->get_parameter<std::string>("plugin_name", plugin_name);
    plugin_names.push_back(plugin_name);
  } catch (const rclcpp::ParameterTypeException & e) {
    try {
      std::vector<std::string> plugin_names_list;
      this->get_parameter<std::vector<std::string>>("plugin_name", plugin_names_list);
      for (const auto & plugin_name : plugin_names_list) {
        RCLCPP_INFO(this->get_logger(), "Loading plugin %s", plugin_name.c_str());
        plugin_names.push_back(plugin_name);
      }
    } catch (const rclcpp::ParameterTypeException & e) {
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
}

bool StateEstimator::loadPlugin(const std::string & _plugin_name)
{
  std::string plugin_name = _plugin_name + "::Plugin";
  as2_state_estimator_plugin_base::StateEstimatorBase::SharedPtr plugin_ptr;
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
    plugin_ptr->setup(this, tf_handler_, state_estimator_interfaces_[plugin_name]);
    plugins_[plugin_name] = plugin_ptr;
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
    return false;
  }
  return true;
}

rclcpp::NodeOptions StateEstimator::get_modified_options(const rclcpp::NodeOptions & options)
{
  // Create a copy of the options and modify it
  rclcpp::NodeOptions modified_options = options;
  modified_options.allow_undeclared_parameters(true);
  modified_options.automatically_declare_parameters_from_overrides(true);
  return modified_options;
}

void StateEstimator::processEarthToMap(
  const std::string & authority,
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  bool is_static)
{
  RCLCPP_INFO(this->get_logger(), "Processing Earth to Map, authority: %s", authority.c_str());
}
void StateEstimator::processMapToOdom(
  const std::string & authority,
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  bool is_static)
{
  RCLCPP_INFO(this->get_logger(), "Processing Map to Odom, authority: %s", authority.c_str());
}
void StateEstimator::processOdomToBase(
  const std::string & authority,
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  bool is_static)
{
  RCLCPP_INFO(this->get_logger(), "Processing Odom to Base, authority: %s", authority.c_str());
}
void StateEstimator::processTwist(
  const std::string & authority,
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg,
  bool is_static)
{
  RCLCPP_INFO(this->get_logger(), "Processing Twist, authority: %s", authority.c_str());
}


}  // namespace as2_state_estimator
