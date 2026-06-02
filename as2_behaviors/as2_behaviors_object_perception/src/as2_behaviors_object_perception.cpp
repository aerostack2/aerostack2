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
#include <stdexcept>
#include <string>
#include <vector>
#include "as2_behaviors_object_perception/common/common.hpp"

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
    const auto camera_image_topic_param =
      this->declare_parameter<std::string>("camera_image_topic", "");
    if (camera_image_topic_param.empty()) {
      enable_arducam = true;
      RCLCPP_INFO(this->get_logger(), "camera_image_topic is empty, using Arducam input");
    } else {
      camera_image_topic_ = as2_behaviors_object_perception::getNamespacedTopic(
        ns,
        camera_image_topic_param);
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <camera_image_topic> malformed: %s", e.what());
    throw;
  }

  try {
    const auto camera_info_topic_param =
      this->declare_parameter<std::string>("camera_info_topic", "");
    if (!camera_info_topic_param.empty()) {
      camera_info_topic_ = as2_behaviors_object_perception::getNamespacedTopic(
        ns,
        camera_info_topic_param);
    }
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

  loadPipeline();

  if (enable_arducam) {
    arducam_ = std::make_unique<as2_usb_camera_interface::ArducamInterface>(this);
    initializeArducamCameraInfo();
  } else {
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      camera_image_topic_,
      as2_names::topics::sensor_measurements::qos,
      std::bind(&PerceptionBehavior::image_callback, this, std::placeholders::_1));

    if (!camera_info_topic_.empty()) {
      cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&PerceptionBehavior::camera_info_callback, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "camera_info_topic is empty. Topic input will run without camera info updates");
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "PerceptionBehavior ready.");
}

void PerceptionBehavior::loadPipeline()
{
  const auto stage_names =
    this->declare_parameter<std::vector<std::string>>(
    "pipeline.stages", std::vector<std::string>{});

  if (stage_names.empty()) {
    loadSinglePluginPipeline();
    return;
  }

  pipeline_stages_.clear();
  pipeline_stages_.reserve(stage_names.size());
  for (const auto & stage_name : stage_names) {
    pipeline_stages_.push_back(loadStage(stage_name));
  }
}

void PerceptionBehavior::loadSinglePluginPipeline()
{
  try {
    plugin_name_ = this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s", e.what());
    throw;
  }

  PipelineStage stage;
  stage.name = plugin_name_;
  stage.plugin_name = plugin_name_;
  stage.input_source = "image";

  const std::string full_plugin_name = stage.plugin_name + "::Plugin";
  stage.plugin = detection_loader_.createSharedInstance(full_plugin_name);
  stage.plugin->initialize(this);
  pipeline_stages_.push_back(stage);

  RCLCPP_INFO(this->get_logger(), "Loaded perception plugin: %s", plugin_name_.c_str());
}

PerceptionBehavior::PipelineStage PerceptionBehavior::loadStage(const std::string & stage_name)
{
  const std::string prefix = "pipeline." + stage_name + ".";
  PipelineStage stage;
  stage.name = stage_name;
  stage.plugin_name = this->declare_parameter<std::string>(prefix + "plugin");
  stage.input_source = this->declare_parameter<std::string>(prefix + "input_source", "internal");
  stage.input_stage = this->declare_parameter<std::string>(prefix + "input_stage", "");
  stage.input_topic = as2_behaviors_object_perception::getNamespacedTopic(
    this->get_namespace(), this->declare_parameter<std::string>(prefix + "input_topic", ""));
  stage.output_topic = as2_behaviors_object_perception::getNamespacedTopic(
    this->get_namespace(), this->declare_parameter<std::string>(prefix + "output_topic", ""));
  stage.publish_output = this->declare_parameter<bool>(prefix + "publish_output", false);
  stage.debug_poses_topic = as2_behaviors_object_perception::getNamespacedTopic(
    this->get_namespace(), this->declare_parameter<std::string>(prefix + "debug_poses_topic", ""));
  stage.debug_markers_topic = as2_behaviors_object_perception::getNamespacedTopic(
    this->get_namespace(),
    this->declare_parameter<std::string>(prefix + "debug_markers_topic", ""));
  stage.debug_marker_size = this->declare_parameter<double>(prefix + "debug_marker_size", 0.15);

  const std::string full_plugin_name = stage.plugin_name + "::Plugin";
  stage.plugin = detection_loader_.createSharedInstance(full_plugin_name);
  stage.plugin->initialize(this);

  if (stage.publish_output && !stage.output_topic.empty()) {
    stage.output_pub =
      this->create_publisher<as2_msgs::msg::ObjectPerceptionArray>(
      stage.output_topic, as2_names::topics::sensor_measurements::qos);
  }

  // Debug: publishes the valid 3D poses as a PoseArray for visualization (e.g. RViz).
  if (!stage.debug_poses_topic.empty()) {
    stage.debug_poses_pub =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
      stage.debug_poses_topic, as2_names::topics::sensor_measurements::qos);
  }

  // Debug: publishes each detection as a flat square (CUBE) MarkerArray for RViz.
  if (!stage.debug_markers_topic.empty()) {
    stage.debug_markers_pub =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      stage.debug_markers_topic, as2_names::topics::sensor_measurements::qos);
  }

  if (stage.input_source == "external") {
    if (stage.input_topic.empty()) {
      throw std::runtime_error(
              "Pipeline stage '" + stage.name +
              "' has empty external input_topic");
    }
    stage.input_sub =
      this->create_subscription<as2_msgs::msg::ObjectPerceptionArray>(
      stage.input_topic,
      as2_names::topics::sensor_measurements::qos,
      [this, stage_name](const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg) {
        external_input_callback(stage_name, msg);
      });
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded pipeline stage '%s' with plugin '%s'",
    stage.name.c_str(), stage.plugin_name.c_str());
  return stage;
}

PerceptionBehavior::PipelineStage * PerceptionBehavior::findStage(const std::string & stage_name)
{
  for (auto & stage : pipeline_stages_) {
    if (stage.name == stage_name) {
      return &stage;
    }
  }
  return nullptr;
}

void PerceptionBehavior::publishStageOutput(const PipelineStage & stage)
{
  if (!stage.output_pub && !stage.debug_poses_pub && !stage.debug_markers_pub) {
    return;
  }

  const auto detections = stage.plugin->getDetections();

  if (stage.output_pub) {
    stage.output_pub->publish(detections);
  }

  if (stage.debug_poses_pub) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = detections.header;
    for (const auto & perception : detections.perceptions) {
      if (perception.pose_valid) {
        pose_array.poses.push_back(perception.hypothesis.pose.pose);
      }
    }
    stage.debug_poses_pub->publish(pose_array);
  }

  if (stage.debug_markers_pub) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous markers before drawing the current detections.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 0;
    for (const auto & perception : detections.perceptions) {
      if (!perception.pose_valid) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header = detections.header;
      marker.ns = stage.name;
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = perception.hypothesis.pose.pose;
      marker.scale.x = stage.debug_marker_size;
      marker.scale.y = stage.debug_marker_size;
      marker.scale.z = 0.005;  // flat square
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.7f;
      marker_array.markers.push_back(marker);
    }
    stage.debug_markers_pub->publish(marker_array);
  }
}

void PerceptionBehavior::external_input_callback(
  const std::string & stage_name,
  const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg)
{
  auto * stage = findStage(stage_name);
  if (!stage) {
    return;
  }
  stage->external_input = *msg;
  stage->has_external_input = true;
}

void PerceptionBehavior::handleImageFrame(
  const cv::Mat & frame, const std_msgs::msg::Header & header)
{
  if (pipeline_stages_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Perception pipeline not initialized");
    return;
  }

  for (auto & stage : pipeline_stages_) {
    if (stage.input_source == "image") {
      stage.plugin->image_callback(frame, header);
    }
  }
}

void PerceptionBehavior::initializeArducamCameraInfo()
{
  if (!arducam_ || arducam_camera_info_initialized_) {
    return;
  }

  const auto camera_info = arducam_->getCameraInfoMessage();
  preprocessor_.updateCameraInfo(camera_info);
  for (auto & stage : pipeline_stages_) {
    stage.plugin->camera_info_callback(camera_info);
  }
  arducam_camera_info_initialized_ = true;
}

void PerceptionBehavior::drainArducamQueue()
{
  if (!arducam_) {
    return;
  }

  initializeArducamCameraInfo();

  as2_usb_camera_interface::ArducamFrame frame;
  as2_usb_camera_interface::ArducamFrame latest_frame;
  bool has_frame = false;
  auto & output_queue = arducam_->getOutputQueue();
  while (output_queue.tryPop(frame)) {
    latest_frame = std::move(frame);
    has_frame = true;
  }

  if (has_frame) {
    handleImageFrame(latest_frame.image, latest_frame.header);
  }
}

bool PerceptionBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  for (auto & stage : pipeline_stages_) {
    if (!stage.plugin->on_activate(goal)) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to activate pipeline stage '%s'", stage.name.c_str());
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "PerceptionBehavior activated");
  return true;
}

bool PerceptionBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  for (auto & stage : pipeline_stages_) {
    if (!stage.plugin->on_modify(goal)) {
      return false;
    }
  }
  return true;
}

bool PerceptionBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_deactivate(message) && success;
  }
  return success;
}

bool PerceptionBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_pause(message) && success;
  }
  return success;
}

bool PerceptionBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_resume(message) && success;
  }
  return success;
}

as2_behavior::ExecutionStatus PerceptionBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> & /*goal*/,
  std::shared_ptr<as2_msgs::action::DetectObjects::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::DetectObjects::Result> & result_msg)
{
  if (enable_arducam) {
    drainArducamQueue();
  }

  auto status = as2_behavior::ExecutionStatus::RUNNING;
  as2_msgs::msg::ObjectPerceptionArray previous_output;
  bool has_previous_output = false;

  for (auto & stage : pipeline_stages_) {
    if (stage.input_source == "internal") {
      if (!stage.input_stage.empty()) {
        auto * input_stage = findStage(stage.input_stage);
        if (input_stage) {
          stage.plugin->setInputDetections(input_stage->plugin->getDetections());
        }
      } else if (has_previous_output) {
        stage.plugin->setInputDetections(previous_output);
      }
    } else if (stage.input_source == "external") {
      if (!stage.has_external_input) {
        continue;
      }
      stage.plugin->setInputDetections(stage.external_input);
      stage.has_external_input = false;
    }

    status = stage.plugin->on_run();
    previous_output = stage.plugin->getDetections();
    has_previous_output = true;
    publishStageOutput(stage);
  }

  latest_pipeline_output_ = previous_output;
  feedback_msg->perceptions = latest_pipeline_output_;
  result_msg->perceptions = latest_pipeline_output_;
  if (persistent_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  return status;
}

void PerceptionBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  for (auto & stage : pipeline_stages_) {
    stage.plugin->on_execution_end(state);
  }
}

void PerceptionBehavior::image_callback(
  const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  cv::Mat frame;
  if (!preprocessor_.preprocessCompressedImage(*image_msg, frame)) {
    return;
  }
  handleImageFrame(frame, image_msg->header);
}

void PerceptionBehavior::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  preprocessor_.updateCameraInfo(*cam_info_msg);
  for (auto & stage : pipeline_stages_) {
    stage.plugin->camera_info_callback(*cam_info_msg);
  }
}

}  // namespace as2_behaviors_object_perception
