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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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
 * @file detector_yolo_gates.cpp
 *
 * Gate detection plugin using YOLO inference backend.
 * Receives pre-processed images from PerceptionBehavior and outputs
 * ObjectPerceptionArray detections.
 *
 * @authors Alba López del Águila
 */

#include "detector_yolo_gates/detector_yolo_gates.hpp"
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace detector_yolo_gates
{

void Plugin::ownInit()
{
  model_path_ =
    node_ptr_->declare_parameter<std::string>("detector_yolo_gates.model_path");
  task_str_ =
    node_ptr_->declare_parameter<std::string>("detector_yolo_gates.task");
  input_size_ =
    node_ptr_->declare_parameter<int>("detector_yolo_gates.input_size");

  confidence_threshold_ =
    node_ptr_->declare_parameter<double>("detector_yolo_gates.confidence_threshold");
  nms_threshold_ =
    node_ptr_->declare_parameter<double>("detector_yolo_gates.nms_threshold");
  keypoint_threshold_ =
    node_ptr_->declare_parameter<double>("detector_yolo_gates.keypoint_threshold");

  max_detections_ = static_cast<size_t>(
    node_ptr_->declare_parameter<int>("detector_yolo_gates.max_detections"));

  inference_frequency_ =
    node_ptr_->declare_parameter<double>("detector_yolo_gates.inference_frequency");
  inference_period_ = rclcpp::Duration::from_seconds(1.0 / inference_frequency_);

  new_frame_ = false;

  // Resolve model path
  std::filesystem::path path(model_path_);
  if (path.is_relative()) {
    path = std::filesystem::absolute(path);
  }
  if (!std::filesystem::exists(path)) {
    RCLCPP_FATAL(
      node_ptr_->get_logger(), "Model path does not exist: %s", path.c_str());
    return;
  }

  // Initialize inference backend
  backend_ptr_ = yolo_inference::createInferenceBackend(path);
  if (!backend_ptr_ ||
    !backend_ptr_->initialize(
      path,
      yolo_inference::stringToTaskType(task_str_),
      input_size_))
  {
    RCLCPP_FATAL(node_ptr_->get_logger(), "Failed to initialize inference backend");
    return;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Initialized %s backend for '%s' task",
    backend_ptr_->getFormat() == yolo_inference::ModelFormat::TENSORRT ? "TensorRT" : "ONNX",
    task_str_.c_str());

  class_names_ = backend_ptr_->getClassNames();
  keypoint_names_ = backend_ptr_->getKeypointNames();

  if (class_names_.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No class names found in model");
  }
  if (keypoint_names_.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No keypoint names found in model");
  }

  node_ptr_->declare_parameter<bool>("detector_yolo_gates.enable_debug", false);
  node_ptr_->declare_parameter<std::string>(
    "detector_yolo_gates.debug.detections_data_topic", "");
  node_ptr_->declare_parameter<std::string>(
    "detector_yolo_gates.debug.debug_detections_image_topic", "");
}

bool Plugin::own_activate(as2_msgs::action::DetectObjects::Goal & goal)
{
  if (goal.threshold < 0.0f || goal.threshold > 1.0f) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be in [0, 1]");
    return false;
  }
  if (goal.threshold > 0.0f) {
    confidence_threshold_ = static_cast<double>(goal.threshold);
  }
  return true;
}

bool Plugin::own_modify(as2_msgs::action::DetectObjects::Goal & goal)
{
  if (goal.threshold < 0.0f || goal.threshold > 1.0f) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be in [0, 1]");
    return false;
  }
  if (goal.threshold > 0.0f) {
    confidence_threshold_ = static_cast<double>(goal.threshold);
  }
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Deactivating gate detector");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Pausing gate detector");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Resuming gate detector");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & /*state*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Gate detector execution ended");
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  const auto now = steady_clock_.now();

  // First iteration: record start time and wait for a frame
  if (last_inference_time_.nanoseconds() == 0) {
    last_inference_time_ = now;
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  if (!new_frame_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  if ((now - last_inference_time_) < inference_period_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // Grab latest frame under lock
  cv::Mat frame;
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame = latest_frame_.clone();
    header = latest_header_;
    new_frame_ = false;
  }

  last_inference_time_ = now;
  processImage(frame, header);

  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(
  const cv::Mat & image,
  const std_msgs::msg::Header & header)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  latest_frame_ = image;
  latest_header_ = header;
  new_frame_ = true;
}

void Plugin::camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info)
{
  // Delegate to base class for matrix extraction
  detection_plugin_base::DetectionBase::camera_info_callback(camera_info);
}

void Plugin::processImage(const cv::Mat & image, const std_msgs::msg::Header & header)
{
  const auto t0 = std::chrono::high_resolution_clock::now();

  yolo_inference::InferenceResult inference;
  processInference(image, inference);

  const auto t1 = std::chrono::high_resolution_clock::now();

  as2_msgs::msg::ObjectPerceptionArray perceptions;
  perceptions.header = header;
  if (!processDetection(inference, perceptions)) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "No detections in this frame");
  }

  const auto t2 = std::chrono::high_resolution_clock::now();

  // Make detections available to PerceptionBehavior
  latest_detections_ = perceptions;

  const auto t3 = std::chrono::high_resolution_clock::now();

  /*
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Inference: %ld ms  Detection: %ld ms  Output: %ld ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(),
    std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(),
    std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count());
  */
}

void Plugin::processInference(
  const cv::Mat & image,
  yolo_inference::InferenceResult & result)
{
  result = backend_ptr_->infer(image, confidence_threshold_, nms_threshold_, keypoint_threshold_);
}

bool Plugin::processDetection(
  const yolo_inference::InferenceResult & inference,
  as2_msgs::msg::ObjectPerceptionArray & perceptions)
{
  perceptions.perceptions.clear();

  if (inference.detections.empty()) {
    return false;
  }

  const size_t n = std::min(inference.detections.size(), max_detections_);
  perceptions.perceptions.reserve(n);

  for (size_t di = 0; di < n; ++di) {
    const auto & det = inference.detections[di];

    as2_msgs::msg::ObjectPerception p;
    p.id = std::to_string(di);
    p.class_name = (static_cast<size_t>(det.class_id) < class_names_.size()) ?
      static_cast<std::string>(class_names_[det.class_id]) :
      "unknown";
    p.confidence = det.confidence;
    p.pose_valid = false;

    // Bounding box (pixel coords, z=0)
    p.bbox_min.x = det.bbox.x;
    p.bbox_min.y = det.bbox.y;
    p.bbox_min.z = 0.0;
    p.bbox_max.x = det.bbox.x + det.bbox.width;
    p.bbox_max.y = det.bbox.y + det.bbox.height;
    p.bbox_max.z = 0.0;

    // Keypoints
    const size_t kp_count = std::min(det.keypoints.size(), keypoint_names_.size());
    p.keypoints.resize(kp_count);
    p.keypoint_names.resize(kp_count);
    p.keypoint_scores.resize(kp_count);

    for (size_t i = 0; i < kp_count; ++i) {
      const auto & kp_in = det.keypoints[i];
      p.keypoints[i].x = kp_in.x;
      p.keypoints[i].y = kp_in.y;
      p.keypoints[i].z = 0.0;
      p.keypoint_names[i] = static_cast<std::string>(keypoint_names_[i]);
      p.keypoint_scores[i] = kp_in.z;  // confidence stored in z
    }

    perceptions.perceptions.emplace_back(std::move(p));
  }

  return !perceptions.perceptions.empty();
}

}  // namespace detector_yolo_gates

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(detector_yolo_gates::Plugin, detection_plugin_base::DetectionBase)
