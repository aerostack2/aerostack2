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
 * @file as2_behaviors_detection.cpp
 *
 * Detect behavior file
 *
 * @authors Guillermo GP-Lenza
 */

#include "as2_behaviors_detection/as2_behaviors_detection.hpp"
#include <string>
#include <memory>

namespace as2_behaviors_detection
{

DetectBehavior::DetectBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::Detect>("DetectBehavior",
    options)
{
  std::string ns = this->get_namespace();
  try {
    std::string plugin_name_ = this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }
  RCLCPP_INFO(this->get_logger(), "Declared plugin_name");

  try {
    camera_image_topic_ = this->declare_parameter<std::string>("camera_image_topic");
    camera_image_topic_ = ns + camera_image_topic_;
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <camera_image_topic> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }
  RCLCPP_INFO(this->get_logger(), "Declared camera_image_topic");

  try {
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic");
    camera_info_topic_ = ns + camera_info_topic_;
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <camera_info_topic> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  RCLCPP_INFO(this->get_logger(), "Declared camera_info_topic");

  try {
    persistent_ = this->declare_parameter<bool>("persistent");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <persistent> not defined or malformed: %s",
      e.what());
    this->~DetectBehavior();
  }

  RCLCPP_INFO(this->get_logger(), "Declared persistent topic");

  detect_loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_behaviors_detection_plugin_base::DetectBase>>(
    "as2_behaviors_detection",
    "as2_behaviors_detection_plugin_base::DetectBase");

  RCLCPP_INFO(this->get_logger(), "Created plugin loader");
  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    detect_plugin_ = detect_loader_->createSharedInstance(plugin_name);
    detect_plugin_->initialize(this);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
      ex.what());
    this->~DetectBehavior();
  }

  RCLCPP_INFO(this->get_logger(), "Loaded plugin");

  persistent_ = this->get_parameter("persistent").as_bool();

  RCLCPP_INFO(this->get_logger(), "Camera topic read");
  // auto img = sensor_msgs::msg::Image();

  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    camera_image_topic_,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&DetectBehavior::image_callback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "Created image_sub");

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&DetectBehavior::camera_info_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Created caminfo sub");

  RCLCPP_DEBUG(this->get_logger(), "Detect Behavior ready!");
}


bool DetectBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
{
  if (!detect_plugin_->on_activate(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate detect plugin");
    return false;
  }
  // Additional activation logic can be added here if neededinit
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
  auto status = detect_plugin_->on_run(goal, feedback_msg, result_msg);
  if (persistent_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  } else {
    return status;
  }
}

void DetectBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
}

void DetectBehavior::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  if (!detect_plugin_) {
    RCLCPP_ERROR(this->get_logger(), "Detect plugin not initialized");
    return;
  }
  detect_plugin_->image_callback(image_msg);
}

void DetectBehavior::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  detect_plugin_->camera_info_callback(cam_info_msg);
}

}  // namespace as2_behaviors_detection
