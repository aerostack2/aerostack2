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
#include <utility>
#include <sensor_msgs/image_encodings.hpp>
#include "as2_core/custom/cv_bridge.hpp"
#include "as2_behaviors_object_perception/common/common.hpp"

namespace as2_behaviors_object_perception
{

ObjectPerceptionBehavior::ObjectPerceptionBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::DetectObjects>(
    "ObjectPerceptionBehavior", options),
  detection_loader_(
    "as2_behaviors_object_perception",
    "detection_plugin_base::DetectionBase"),
  preprocessor_(this->get_logger())
{
  const std::string ns = this->get_namespace();

  try {
    use_embedded_camera = this->declare_parameter<bool>("use_embedded_camera");
    if (use_embedded_camera) {
      RCLCPP_INFO(
        this->get_logger(),
        "Images will be captured directly from the camera device by this node "
        "(use_embedded_camera is true), not read from a topic");
    } else {
      const auto camera_image_topic_param =
        this->declare_parameter<std::string>("camera_image_topic");
      camera_image_topic_ = as2_behaviors_object_perception::getNamespacedTopic(
        ns, camera_image_topic_param);
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

  // Rectification changes the effective intrinsics, so downstream PnP/pose
  // estimation needs the new K. Only makes sense when rectifying, and it is
  // opt-in: an empty topic (the default) disables it.
  const auto rectified_camera_info_topic_param =
    this->declare_parameter<std::string>("rectified_camera_info_topic", "");
  if (enable_rectification && !rectified_camera_info_topic_param.empty()) {
    rectified_camera_info_topic_ = as2_behaviors_object_perception::getNamespacedTopic(
      ns, rectified_camera_info_topic_param);
    // Latched: published once, late subscribers still get it.
    rectified_cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      rectified_camera_info_topic_, rclcpp::QoS(1).transient_local());
    RCLCPP_INFO(
      this->get_logger(), "Publishing rectified camera info on '%s'",
      rectified_camera_info_topic_.c_str());
  }

  loadPipeline();

  if (use_embedded_camera) {
    camera_driver_ = std::make_unique<usb_camera_interface::UsbCameraInterface>(this);
    initializeCameraInfo();
  } else {
    // The transport is picked from the topic name, as image_transport does: a
    // "/compressed" suffix means CompressedImage, anything else raw Image.
    if (isCompressedTopic(camera_image_topic_)) {
      image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        camera_image_topic_,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&ObjectPerceptionBehavior::image_callback, this, std::placeholders::_1));
    } else {
      raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_image_topic_,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&ObjectPerceptionBehavior::raw_image_callback, this, std::placeholders::_1));
    }
    RCLCPP_INFO(
      this->get_logger(), "Subscribed to image topic '%s' (%s)",
      camera_image_topic_.c_str(),
      isCompressedTopic(camera_image_topic_) ? "CompressedImage" : "Image");

    if (!camera_info_topic_.empty()) {
      cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        as2_names::topics::sensor_measurements::qos,
        std::bind(&ObjectPerceptionBehavior::camera_info_callback, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to camera info topic '%s'",
        camera_info_topic_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "camera_info_topic is empty. Topic input will run without camera info updates");
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "ObjectPerceptionBehavior ready, waiting for a DetectObjects goal. "
    "The pipeline does not run until the behavior is activated.");
}

void ObjectPerceptionBehavior::loadPipeline()
{
  const auto stage_names =
    this->declare_parameter<std::vector<std::string>>(
    "pipeline.stages", std::vector<std::string>{});

  if (stage_names.empty()) {
    RCLCPP_FATAL(this->get_logger(), "Launch argument <pipeline.stages> is empty");
    throw std::runtime_error("Perception pipeline has no stages");
  }

  pipeline_stages_.clear();
  pipeline_stages_.reserve(stage_names.size());
  for (const auto & stage_name : stage_names) {
    pipeline_stages_.push_back(loadStage(stage_name));
  }
}

ObjectPerceptionBehavior::PipelineStage ObjectPerceptionBehavior::loadStage(
  const std::string & stage_name)
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
  stage.enable_debug = this->declare_parameter<bool>(prefix + "enable_debug", false);
  stage.debug_poses_topic = as2_behaviors_object_perception::getNamespacedTopic(
    this->get_namespace(), this->declare_parameter<std::string>(prefix + "debug_poses_topic", ""));

  const std::string full_plugin_name = stage.plugin_name + "::Plugin";
  stage.plugin = detection_loader_.createSharedInstance(full_plugin_name);
  stage.plugin->initialize(this);

  RCLCPP_INFO(
    this->get_logger(), "Loaded pipeline stage '%s' with plugin '%s'",
    stage.name.c_str(), stage.plugin_name.c_str());

  if (stage.publish_output && !stage.output_topic.empty()) {
    stage.output_pub =
      this->create_publisher<as2_msgs::msg::ObjectPerceptionArray>(
      stage.output_topic, as2_names::topics::sensor_measurements::qos);
    RCLCPP_INFO(
      this->get_logger(), "Stage '%s' publishes detections on '%s'",
      stage.name.c_str(), stage.output_topic.c_str());
  }

  // Debug: publishes the valid 3D poses as a PoseArray for visualization (e.g. RViz).
  if (stage.enable_debug && !stage.debug_poses_topic.empty()) {
    stage.debug_poses_pub =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
      stage.debug_poses_topic, as2_names::topics::sensor_measurements::qos);
    RCLCPP_INFO(
      this->get_logger(), "Stage '%s' publishes debug poses on '%s'",
      stage.name.c_str(), stage.debug_poses_topic.c_str());
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
    RCLCPP_INFO(
      this->get_logger(), "Stage '%s' takes its input from '%s'",
      stage.name.c_str(), stage.input_topic.c_str());
  }

  return stage;
}

ObjectPerceptionBehavior::PipelineStage * ObjectPerceptionBehavior::findStage(
  const std::string & stage_name)
{
  for (auto & stage : pipeline_stages_) {
    if (stage.name == stage_name) {
      return &stage;
    }
  }
  return nullptr;
}

void ObjectPerceptionBehavior::publishStageOutput(const PipelineStage & stage)
{
  const auto detections = stage.plugin->getDetections();

  if (stage.output_pub) {
    stage.output_pub->publish(detections);
  }

  if (stage.enable_debug) {
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
    stage.plugin->publishDebug(detections);
  }
}

void ObjectPerceptionBehavior::external_input_callback(
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

void ObjectPerceptionBehavior::handleImageFrame(
  const cv::Mat & frame, const std_msgs::msg::Header & header)
{
  if (pipeline_stages_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Perception pipeline not initialized");
    return;
  }

  if (!images_received_) {
    RCLCPP_INFO(
      this->get_logger(), "First image received (%dx%d)", frame.cols, frame.rows);
    images_received_ = true;
  }

  // When the stream is rectified, the effective intrinsics change (a new K is
  // estimated, distortion is zeroed). Keypoints produced by the detector are in
  // rectified pixel coordinates, so downstream stages doing PnP need the
  // rectified K, not the raw camera_info. Forward it once it is available.
  if (preprocessor_.rectificationReady() && !rectified_info_propagated_) {
    const auto rectified_info = preprocessor_.getCameraInfo();
    for (auto & stage : pipeline_stages_) {
      stage.plugin->camera_info_callback(rectified_info);
    }
    // The publisher is latched, so a single publish also reaches late subscribers.
    if (rectified_cam_info_pub_) {
      rectified_cam_info_pub_->publish(rectified_info);
    }
    rectified_info_propagated_ = true;
    RCLCPP_INFO(this->get_logger(), "Propagated rectified camera info to pipeline stages");
  }

  for (auto & stage : pipeline_stages_) {
    if (stage.input_source == "image") {
      stage.plugin->image_callback(frame, header);
    }
  }
}

void ObjectPerceptionBehavior::initializeCameraInfo()
{
  if (!camera_driver_ || camera_info_initialized_) {
    return;
  }

  const auto camera_info = camera_driver_->getCameraInfoMessage();
  preprocessor_.updateCameraInfo(camera_info);
  for (auto & stage : pipeline_stages_) {
    stage.plugin->camera_info_callback(camera_info);
  }
  camera_info_initialized_ = true;
}

void ObjectPerceptionBehavior::drainCameraQueue()
{
  if (!camera_driver_) {
    return;
  }

  initializeCameraInfo();

  usb_camera_interface::CameraFrame frame;
  usb_camera_interface::CameraFrame latest_frame;
  bool has_frame = false;
  auto & output_queue = camera_driver_->getOutputQueue();
  while (output_queue.tryPop(frame)) {
    latest_frame = std::move(frame);
    has_frame = true;
  }

  if (has_frame) {
    cv::Mat processed;
    if (!preprocessor_.preprocessImage(latest_frame.image, processed)) {
      return;
    }
    handleImageFrame(processed, latest_frame.header);
  }
}

bool ObjectPerceptionBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  for (auto & stage : pipeline_stages_) {
    if (!stage.plugin->on_activate(goal)) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to activate pipeline stage '%s'", stage.name.c_str());
      return false;
    }
    // "First" is relative to the goal: each new goal reports its first detection.
    stage.logged_first_detection = false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Detection started. Results are published on the output topic of each stage; "
    "the action feedback carries the detections of the last stage");
  return true;
}

bool ObjectPerceptionBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
{
  for (auto & stage : pipeline_stages_) {
    if (!stage.plugin->on_modify(goal)) {
      return false;
    }
  }
  return true;
}

bool ObjectPerceptionBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_deactivate(message) && success;
  }
  return success;
}

bool ObjectPerceptionBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_pause(message) && success;
  }
  return success;
}

bool ObjectPerceptionBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  bool success = true;
  for (auto & stage : pipeline_stages_) {
    success = stage.plugin->on_resume(message) && success;
  }
  return success;
}

as2_behavior::ExecutionStatus ObjectPerceptionBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> & /*goal*/,
  std::shared_ptr<as2_msgs::action::DetectObjects::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::DetectObjects::Result> & result_msg)
{
  if (use_embedded_camera) {
    drainCameraQueue();
  }

  if (!images_received_) {
    if (use_embedded_camera) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Detection is active but the camera device has not delivered any image yet. "
        "Check that the device in camera_interface_file exists and is not in use");
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Detection is active but no image has arrived on '%s' yet. Check that the topic "
        "is being published and that its type is %s",
        camera_image_topic_.c_str(),
        isCompressedTopic(camera_image_topic_) ?
        "sensor_msgs/CompressedImage (the topic name ends in /compressed)" :
        "sensor_msgs/Image (add a /compressed suffix to the topic name for CompressedImage)");
    }
  }

  as2_msgs::msg::ObjectPerceptionArray previous_output;
  bool has_previous_output = false;

  bool any_failure = false;
  bool any_running = false;

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

    stage.last_status = stage.plugin->on_run();

    previous_output = stage.plugin->getDetections();
    has_previous_output = true;
    if (stage.plugin->hasNewDetections()) {
      publishStageOutput(stage);
      // Only the first one: enough to tell the stage works, and the topics were
      // already logged at startup. A per-stage flag instead of RCLCPP_INFO_ONCE,
      // which would fire for a single stage of the pipeline.
      if (!stage.logged_first_detection) {
        stage.logged_first_detection = true;
        RCLCPP_INFO(
          this->get_logger(), "Stage '%s' first detection: %zu object(s)",
          stage.name.c_str(), previous_output.perceptions.size());
      }
    }

    if (stage.last_status == as2_behavior::ExecutionStatus::FAILURE ||
      stage.last_status == as2_behavior::ExecutionStatus::ABORTED)
    {
      any_failure = true;
      RCLCPP_ERROR(
        this->get_logger(), "Pipeline stage '%s' failed", stage.name.c_str());
    } else if (stage.last_status == as2_behavior::ExecutionStatus::RUNNING) {
      any_running = true;
    }
  }

  latest_pipeline_output_ = previous_output;
  feedback_msg->perceptions = latest_pipeline_output_;
  result_msg->perceptions = latest_pipeline_output_;

  if (any_failure) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (persistent_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  return any_running ?
         as2_behavior::ExecutionStatus::RUNNING :
         as2_behavior::ExecutionStatus::SUCCESS;
}

void ObjectPerceptionBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  for (auto & stage : pipeline_stages_) {
    stage.plugin->on_execution_end(state);
  }
}

void ObjectPerceptionBehavior::image_callback(
  const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  cv::Mat frame;
  if (!preprocessor_.preprocessCompressedImage(*image_msg, frame)) {
    return;
  }
  handleImageFrame(frame, image_msg->header);
}

void ObjectPerceptionBehavior::raw_image_callback(
  const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Could not convert image from '%s' to BGR8: %s",
      image_msg->encoding.c_str(), e.what());
    return;
  }

  cv::Mat frame;
  if (!preprocessor_.preprocessImage(cv_image->image, frame)) {
    return;
  }
  handleImageFrame(frame, image_msg->header);
}

void ObjectPerceptionBehavior::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  preprocessor_.updateCameraInfo(*cam_info_msg);
  for (auto & stage : pipeline_stages_) {
    stage.plugin->camera_info_callback(*cam_info_msg);
  }
  // New calibration invalidates the rectification maps, so the rectified
  // intrinsics have to be recomputed and propagated again.
  rectified_info_propagated_ = false;
}

}  // namespace as2_behaviors_object_perception
