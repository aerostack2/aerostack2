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
 * @file detect_behavior.cpp
 *
 * Detect behavior file
 *
 * @authors Guillermo GP-Lenza
 */

#include <string>
#include <memory>
#include "detect_behavior/detect_behavior.hpp"

DetectBehavior::DetectBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::Detect>(as2_names::actions::behaviors::detect,
    options)
{
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  try {
    this->declare_parameter<std::string>("camera_image_topic");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <camera_image_topic> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  try {
    this->declare_parameter<std::string>("camera_info_topic");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <camera_info_topic> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  try {
    this->declare_parameter<std::string>("persistent");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <persistent> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    detect_plugin_ = loader_->createSharedInstance(plugin_name);
  } catch {
    (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
        ex.what());
      this->~DetectBehavior();
    }
  }

  loader_ = std::make_shared<pluginlib::ClassLoader<detect_base::DetectBase>>(
    "as2_behaviors_perception",
    "detect_base::DetectBase");

  persistent = this->get_parameter("persistent").as_bool();

  camera_sub = this->create_subscription<sensor_msgs::msg::Image>(
    this->get_parameter("camera_image_topic").as_string(),
    as2_names::topics::sensors::qos,
    std::bind(&DetectBehavior::image_callback, this, std::placeholders::_1));

  cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    this->get_parameter("camera_info_topic").as_string(),
    as2_names::topics::sensors::qos,
    std::bind(&DetectBehavior::camera_info_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "Detect Behavior ready!");
}

bool DetectBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
{
  if (!detect_plugin_->on_activate(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate detect plugin");
    return false;
  }
  // Additional activation logic can be added here if needed
  RCLCPP_INFO(this->get_logger(), "Detect behavior activated successfully");
  return true;
}

bool DetectBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
{
  return detect_plugin_->on_modify(goal);
}

bool DetectBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_deactivate(message);
}

bool DetectBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_pause(message);
}

bool DetectBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus DetectBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Detect::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Detect::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Detect::Result> & result_msg)
{
  return detect_plugin_->on_run(goal, feedback_msg, result_msg);
}

void DetectBehavior::image_callback(const sensor_msgs::msg::Image::SharedPtr & image_msg)
{
  detect_plugin_->image_callback(image_msg);
}

void DetectBehavior::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr & cam_info_msg)
{
  detect_plugin_->camera_info_callback(cam_info_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DetectBehavior)
