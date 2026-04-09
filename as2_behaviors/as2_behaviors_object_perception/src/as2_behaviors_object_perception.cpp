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
 * @file as2_behaviors_object_perception.cpp
 *
 * Perception behavior manager — orchestrates image preprocessing and the
 * detection plugin stage.
 *
 * @authors Alba López del Águila
 */

#include "as2_behaviors_object_perception/as2_behaviors_object_perception.hpp"
#include <memory>
#include <string>

namespace as2_behaviors_object_perception
{

PerceptionBehavior::PerceptionBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::DetectObjects>(
    "PerceptionBehavior", options),
  detection_loader_(
    "as2_behaviors_object_perception",
    "detection_plugin_base::DetectionBase"),
  preprocessor_(this->get_logger())
{
  const std::string ns = this->get_namespace();

  try {
    plugin_name_ = this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s", e.what());
    throw;
  }

  try {
    camera_image_topic_ = ns + this->declare_parameter<std::string>("camera_image_topic");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <camera_image_topic> not defined or malformed: %s", e.what());
    throw;
  }

  try {
    camera_info_topic_ = ns + this->declare_parameter<std::string>("camera_info_topic");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <camera_info_topic> not defined or malformed: %s", e.what());
    throw;
  }

  try {
    persistent_ = this->declare_parameter<bool>("persistent");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <persistent> not defined or malformed: %s", e.what());
    throw;
  }

  bool enable_rectification =
    this->declare_parameter<bool>("enable_rectification", false);
  preprocessor_.setRectificationEnabled(enable_rectification);

  // Load detection plugin
  try {
    const std::string full_plugin_name = plugin_name_ + "::Plugin";
    detection_plugin_ = detection_loader_.createSharedInstance(full_plugin_name);
    detection_plugin_->initialize(this);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to load detection plugin '%s': %s",
      plugin_name_.c_str(), ex.what());
    throw;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded detection plugin: %s", plugin_name_.c_str());

  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    camera_image_topic_,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&PerceptionBehavior::image_callback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&PerceptionBehavior::camera_info_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "PerceptionBehavior ready.");
}

bool PerceptionBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  if (!detection_plugin_->on_activate(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate detection plugin");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "PerceptionBehavior activated");
  return true;
}

bool PerceptionBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  return detection_plugin_->on_modify(goal);
}

bool PerceptionBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return detection_plugin_->on_deactivate(message);
}

bool PerceptionBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return detection_plugin_->on_pause(message);
}

bool PerceptionBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return detection_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus PerceptionBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> & /*goal*/,
  std::shared_ptr<as2_msgs::action::DetectObjects::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::DetectObjects::Result> & result_msg)
{
  auto status = detection_plugin_->on_run();
  auto detections = detection_plugin_->getDetections();
  feedback_msg->perceptions = detections;
  result_msg->perceptions = detections;
  if (persistent_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  return status;
}

void PerceptionBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  detection_plugin_->on_execution_end(state);
}

void PerceptionBehavior::image_callback(
  const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  if (!detection_plugin_) {
    RCLCPP_ERROR(this->get_logger(), "Detection plugin not initialized");
    return;
  }
  cv::Mat frame;
  if (!preprocessor_.preprocessCompressedImage(*image_msg, frame)) {
    return;
  }
  detection_plugin_->image_callback(frame, image_msg->header);
}

void PerceptionBehavior::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  preprocessor_.updateCameraInfo(*cam_info_msg);
  detection_plugin_->camera_info_callback(*cam_info_msg);
}

}  // namespace as2_behaviors_object_perception
