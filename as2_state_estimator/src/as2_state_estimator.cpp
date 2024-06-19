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
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#include "as2_state_estimator.hpp"
namespace as2_state_estimator
{

StateEstimator::StateEstimator(const rclcpp::NodeOptions & options)
: as2::Node("state_estimator", get_modified_options(options))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  try {
    this->get_parameter("plugin_name", plugin_name_);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~StateEstimator();
  }
  plugin_name_ += "::Plugin";
  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>(
    "as2_state_estimator", "as2_state_estimator_plugin_base::StateEstimatorBase");
  try {
    plugin_ptr_ = loader_->createSharedInstance(plugin_name_);
    plugin_ptr_->setup(this, tf_buffer_, tf_broadcaster_, tfstatic_broadcaster_);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
    this->~StateEstimator();
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

}  // namespace as2_state_estimator
