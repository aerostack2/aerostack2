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
 * @file gate_detection.hpp
 *
 * This file contains the implementation to a gate detector based on rgb values.
 *
 * @authors
 */

#include "gate_detection/gate_detection.hpp"
#include <memory>
#include <algorithm>
#include <utility>
#include <vector>
#include <string>


namespace gate_detection
{
void Plugin::ownInit()
{
  model_path_ =
    node_ptr_->declare_parameter<std::string>("gate_detection.model_path");
  task_str_ = node_ptr_->declare_parameter<std::string>("gate_detection.task");
  input_size_ = node_ptr_->declare_parameter<int>("gate_detection.input_size");

  confidence_threshold_ =
    node_ptr_->declare_parameter<double>("gate_detection.confidence_threshold");
  nms_threshold_ =
    node_ptr_->declare_parameter<double>("gate_detection.nms_threshold");
  keypoint_threshold_ =
    node_ptr_->declare_parameter<double>("gate_detection.keypoint_threshold");

  int max_detections =
    node_ptr_->declare_parameter<int>("gate_detection.max_detections");
  max_detections_ = static_cast<size_t>(max_detections);

  enable_rectification_ = node_ptr_->declare_parameter<bool>(
    "gate_detection.enable_rectification",
    false);
  bool enable_debug =
    node_ptr_->declare_parameter<bool>("gate_detection.enable_debug", false);

  inference_frequency_ =
    node_ptr_->declare_parameter<double>("gate_detection.inference_frequency");
  inference_period_ =
    rclcpp::Duration::from_seconds(1.0 / inference_frequency_);

  new_frame_ = false;
  camera_info_received_ = false;
  significant_distorsion_ = false;
  rect_maps_initialized_ = false;

  if (model_path_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "model_path is empty");
    return;
  }

  // Convert relative path to absolute
  std::filesystem::path path(model_path_);
  if (path.is_relative()) {
    path = std::filesystem::absolute(path);
  }

  // Check if path exists
  if (!std::filesystem::exists(path)) {
    RCLCPP_FATAL(
      node_ptr_->get_logger(), "Model path does not exist: %s", path.c_str());
    this->~Plugin();
  }

  // Initialize backend
  backend_ptr_ = yolo_inference::createInferenceBackend(path);
  if (!backend_ptr_ ||
    !backend_ptr_->initialize(
      path,
      yolo_inference::stringToTaskType(task_str_),
      input_size_))
  {
    RCLCPP_FATAL(
      node_ptr_->get_logger(), "Failed to initialize inference backend");
    return;
  }
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Initialized %s backend for %s task",
    backend_ptr_->getFormat() ==
    yolo_inference::ModelFormat::TENSORRT ? "TensorRT" : "ONNX",
    task_str_.c_str());

  // Detections initialization
  class_names_ = backend_ptr_->getClassNames();
  keypoint_names_ = backend_ptr_->getKeypointNames();
  if (class_names_.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No class names found");
  }
  if (keypoint_names_.empty()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No keypoint names found");
  }

  if (enable_debug) {
    std::string detections_data_topic =
      node_ptr_->declare_parameter<std::string>(
      "gate_detection.debug.detections_data_topic");

    if (!detections_data_topic.empty()) {
      detections_data_pub_ =
        node_ptr_->create_publisher
        <as2_gates_localization::msg::KeypointDetectionArray>(
        detections_data_topic, as2_names::topics::sensor_measurements::qos);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing detections to %s",
        detections_data_topic.c_str());
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing detections to %s",
        detections_data_topic.c_str());
    }

    std::string debug_detections_image_topic =
      node_ptr_->declare_parameter<std::string>(
      "gate_detection.debug.debug_detections_image_topic");
    if (!debug_detections_image_topic.empty()) {
      detections_image_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::CompressedImage>(
        debug_detections_image_topic + "/compressed",
        as2_names::topics::sensor_measurements::qos);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing detections image to %s",
        debug_detections_image_topic.c_str());

      // Camera info publisher for debug image
      camera_info_pub_ = node_ptr_->create_publisher
        <sensor_msgs::msg::CameraInfo>(
        debug_detections_image_topic + "/camera_info",
        as2_names::topics::sensor_measurements::qos);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing camera info to %s",
        (debug_detections_image_topic + "/camera_info").c_str());
    }
  }
}

bool Plugin::own_activate(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }
  confidence_threshold_ = goal.threshold;
  return true;
}

bool Plugin::own_modify(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }
  confidence_threshold_ = goal.threshold;
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Deactivating gate color detector");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Pausing gate color detector");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Resuming gate color detector");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Execution ended");
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Running gate detection");

  const auto now = steady_clock_.now();

  // first time of the loop
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

  processImage(input_data_, output_data_);
  last_inference_time_ = now;
  new_frame_ = false;

  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(
  const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  if (!image_msg) {
    return;
  }

  cv::Mat img;

  compressedImagePreprocessing(image_msg, img);
  input_data_.header = image_msg->header;
  input_data_.image = img;

  new_frame_ = true;
}

void Plugin::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  if (!cam_info_msg) {
    return;
  }

  camera_info_ = *cam_info_msg;
  // Extract camera matrix K (3x3)
  const auto & K = cam_info_msg->k;
  if (K.size() == 9) {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << K[0], K[1], K[2],
      K[3], K[4], K[5],
      K[6], K[7], K[8]);
  } else {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "GatesDetection: CameraInfo.K has size %zu (expected 9).", K.size());
  }

  // Extract distortion coefficients D
  const auto & D = cam_info_msg->d;
  if (!D.empty()) {
    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(D.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(D.size()); ++i) {
      dist_coeffs_.at<double>(0, i) = D[i];
    }
  } else {
    // No distortion provided -> assume zero distortion
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "GatesDetection: CameraInfo.D is empty. Assuming zero distortion.");
  }

  // Store distortion model
  distortion_model_ = cam_info_msg->distortion_model;

  // Check if we should enable rectification (only if there's actual distortion)
  significant_distorsion_ = !D.empty() && cv::norm(dist_coeffs_) > 1e-6;

  if (enable_rectification_) {
    initRectificationMaps(
      cv::Size(cam_info_msg->width, cam_info_msg->height));
  }

  camera_info_received_ = true;
}

void Plugin::compressedImagePreprocessing(
  const sensor_msgs::msg::CompressedImage::SharedPtr camera_image_msg,
  cv::Mat & image)
{
  const auto start = std::chrono::high_resolution_clock::now();
  buffer_ =
    cv::Mat(
    1, camera_image_msg->data.size(), CV_8UC1,
    const_cast<uint8_t *>(camera_image_msg->data.data()));
  cv::Mat decoded_image = cv::imdecode(buffer_, cv::IMREAD_COLOR);
  if (decoded_image.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to decode image");
    return;
  }
  const auto end = std::chrono::high_resolution_clock::now();
  const auto duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  // Apply image rectification if camera info is available and rectification
  // is enabled
  if (enable_rectification_ && significant_distorsion_ && !camera_matrix_.empty() &&
    !dist_coeffs_.empty())
  {
    try {
      cv::cuda::GpuMat d_src, d_dst;
      d_src.upload(decoded_image);

      cv::cuda::remap(
        d_src, d_dst, d_map1_, d_map2_,
        cv::INTER_LINEAR, cv::BORDER_CONSTANT);

      d_dst.download(image);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Failed to rectify image: %s. Using original image.", e.what());
      image = decoded_image;
    }
  } else {
    // No rectification needed or available
    image = decoded_image;
  }
}

bool Plugin::processImage(
  const GatesDetectionInputData & input_data,
  GatesDetectionOutputData & output_data)
{
  // Run inference
  const auto inference = std::chrono::high_resolution_clock::now();
  processInference(
    input_data.image, inference_);

  const auto detection = std::chrono::high_resolution_clock::now();
  // Process inference to get detections
  if (!processDetection(
      inference_, output_data.detections))
  {
    RCLCPP_WARN(node_ptr_->get_logger(), "No detections found");
    return false;
  }
  output_data.header = input_data.header;
  output_data.detections.header = input_data.header;

  // Publish debug information if enabled
  // TODO(RPS98): Move this to another thread
  const auto debug = std::chrono::high_resolution_clock::now();
  publishDebug(
    output_data.detections, input_data.image, input_data.header);

  const auto end = std::chrono::high_resolution_clock::now();
  const auto inference_duration =
    std::chrono::duration_cast
    <std::chrono::milliseconds>(detection - inference).count();
  const auto detection_duration =
    std::chrono::duration_cast
    <std::chrono::milliseconds>(debug - detection).count();
  const auto debug_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(end - debug).count();
  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Inference: %ld ms, Detection: %ld ms, Debug: %ld ms",
    inference_duration, detection_duration, debug_duration);

  // Add results to output queue
  output_queue_.push(output_data);

  return true;
}

void Plugin::processInference(
  const cv::Mat & image,
  yolo_inference::InferenceResult & result)
{
  result = backend_ptr_->infer(
    image,
    confidence_threshold_,
    nms_threshold_,
    keypoint_threshold_);
}

bool Plugin::processDetection(
  const yolo_inference::InferenceResult & inference,
  as2_gates_localization::msg::KeypointDetectionArray & detections_array)
{
  // Reserve space and clear previous detections
  detections_array.detections.clear();
  const std::size_t max_to_process =
    std::min(
    inference.detections.size(),
    static_cast<std::size_t>(max_detections_));
  detections_array.detections.reserve(max_to_process);

  if (inference.detections.empty()) {
    return false;
  }

  // Process each detection

  for (std::size_t di = 0; di < max_to_process; ++di) {
    const auto & detection = inference.detections[di];

    as2_gates_localization::msg::KeypointDetection detection_msg;

    // Basic detection info
    detection_msg.class_id = detection.class_id;
    detection_msg.label = static_cast<size_t>(detection.class_id) <
      class_names_.size() ? class_names_[detection.class_id] : "unknown";
    detection_msg.confidence = detection.confidence;

    // Bounding box - convert from cv::Rect2f to message format
    detection_msg.bounding_box.x1 = detection.bbox.x;
    detection_msg.bounding_box.y1 = detection.bbox.y;
    detection_msg.bounding_box.x2 = detection.bbox.x + detection.bbox.width;
    detection_msg.bounding_box.y2 = detection.bbox.y + detection.bbox.height;
    detection_msg.bounding_box.confidence = detection.confidence;

    // Keypoints
    const std::size_t kp_count =
      std::min(detection.keypoints.size(), keypoint_names_.size());

    // Resize once so we can assign by index with no growth checks.
    detection_msg.keypoints.resize(kp_count);
    for (std::size_t i = 0; i < kp_count; ++i) {
      const auto & keypoint_in = detection.keypoints[i];
      auto & keypoint_out = detection_msg.keypoints[i];

      keypoint_out.name = keypoint_names_[i];
      keypoint_out.x = keypoint_in.x;
      keypoint_out.y = keypoint_in.y;
      keypoint_out.confidence = keypoint_in.z;
      keypoint_out.visible = keypoint_in.z > keypoint_threshold_;
    }
    detections_array.detections.emplace_back(std::move(detection_msg));
  }
  if (detections_array.detections.empty()) {
    return false;
  }
  return true;
}

void Plugin::initRectificationMaps(const cv::Size & size)
{
  if (camera_matrix_.empty() || dist_coeffs_.empty()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Cannot initialize rectification maps:missing arguments");
    rect_maps_initialized_ = false;
    return;
  }

  rectified_size_ = size;

  // Matriz de rectificación y nueva K (puedes ajustar si quieres cambiar FOV)
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat P = camera_matrix_.clone();   // similar a tu new_camera_matrix

  if (distortion_model_ == "fisheye" || distortion_model_ == "equidistant") {
    // Modelo de ojo de pez - use estimateNewCameraMatrixForUndistortRectify
    // with balance 0.0
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      camera_matrix_, dist_coeffs_, rectified_size_, R, P, 0.0);

    cv::fisheye::initUndistortRectifyMap(
      camera_matrix_, dist_coeffs_,
      R, P,
      rectified_size_,
      CV_32FC1,     // map1 y map2 en float
      map1_, map2_);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Initialized fisheye rectification maps (CPU)");

    // Sustituit camera_matrix_ por P para usar la nueva cámara rectificada
    camera_matrix_ = P(cv::Rect(0, 0, 3, 3)).clone();
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
    camera_info_.k[0] = camera_matrix_.at<double>(0, 0);
    camera_info_.k[2] = camera_matrix_.at<double>(0, 2);
    camera_info_.k[4] = camera_matrix_.at<double>(1, 1);
    camera_info_.k[5] = camera_matrix_.at<double>(1, 2);
    camera_info_.p[0] = camera_matrix_.at<double>(0, 0);
    camera_info_.p[2] = camera_matrix_.at<double>(0, 2);
    camera_info_.p[5] = camera_matrix_.at<double>(1, 1);
    camera_info_.p[6] = camera_matrix_.at<double>(1, 2);

  } else if (distortion_model_ == "plumb_bob" || distortion_model_ == "radtan" ||
    distortion_model_.empty())
  {
    // Modelo pinhole estándar
    cv::initUndistortRectifyMap(
      camera_matrix_, dist_coeffs_,
      R, P,
      rectified_size_,
      CV_32FC1,
      map1_, map2_);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Initialized plumb_bob rectification maps (CPU)");
  } else {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Unknown distortion model '%s'. Skipping rectification maps init.",
      distortion_model_.c_str());
    rect_maps_initialized_ = false;
    return;
  }
  d_map1_.upload(map1_);
  d_map2_.upload(map2_);
  rect_maps_initialized_ = true;
}

void Plugin::publishDebug(
  const as2_gates_localization::msg::KeypointDetectionArray & detections,
  const cv::Mat & image,
  const std_msgs::msg::Header & header)
{
  if (!detections_image_pub_ && !detections_data_pub_) {
    return;
  }

  // Publish data detections if publisher exists
  if (detections_data_pub_) {
    detections_data_pub_->publish(detections);
  }

  // Create and publish visualization image if publisher exists
  if (detections_image_pub_) {
    // Static color vector to avoid recreating every time
    static const std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 255, 0),       // Green
      cv::Scalar(255, 0, 0),       // Blue
      cv::Scalar(0, 0, 255),       // Red
      cv::Scalar(255, 255, 0),     // Cyan
      cv::Scalar(255, 0, 255),     // Magenta
      cv::Scalar(0, 255, 255),     // Yellow
    };

    cv::Mat vis_image = image.clone();

    const size_t num_detections = detections.detections.size();
    for (size_t i = 0; i < num_detections; ++i) {
      const auto & detection = detections.detections[i];
      const cv::Scalar & color = colors[i % colors.size()];

      // Draw bounding box - use faster int conversion
      const int x1 = static_cast<int>(detection.bounding_box.x1 + 0.5f);
      const int y1 = static_cast<int>(detection.bounding_box.y1 + 0.5f);
      const int x2 = static_cast<int>(detection.bounding_box.x2 + 0.5f);
      const int y2 = static_cast<int>(detection.bounding_box.y2 + 0.5f);

      cv::rectangle(vis_image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

      // Pre-calculate confidence as integer to avoid repeated conversion
      const int confidence_pct =
        static_cast<int>(detection.confidence * 100 + 0.5f);

      // Use more efficient string building
      std::string label;
      label.reserve(detection.label.size() + 8);   // Reserve space
      label = detection.label;
      label += " ";
      label += std::to_string(confidence_pct);
      label += "%";

      // Draw label background and text in one go
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
      const cv::Point label_pos(x1, y1 - baseline);

      cv::rectangle(
        vis_image,
        cv::Point(x1, y1 - text_size.height - baseline),
        cv::Point(x1 + text_size.width, y1),
        color, -1);
      cv::putText(
        vis_image, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(255, 255, 255), 2);

      // Draw keypoints - minimize operations inside loop
      for (const auto & keypoint : detection.keypoints) {
        if (keypoint.visible) {
          const int kp_x = static_cast<int>(keypoint.x + 0.5f);
          const int kp_y = static_cast<int>(keypoint.y + 0.5f);
          const cv::Point kp_point(kp_x, kp_y);

          cv::circle(vis_image, kp_point, 4, color, -1);

          // Draw keypoint name with offset
          std::string keypoint_label = keypoint.name + " " +
            cv::format("%.4f%%", keypoint.confidence * 100.0);
          cv::putText(
            vis_image, keypoint_label, cv::Point(kp_x + 5, kp_y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
        }
      }
    }

    // Add detection count info - use pre-calculated string
    const std::string info_text =
      "Detections: " + std::to_string(num_detections);
    cv::putText(
      vis_image, info_text, cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    // Convert to image message using cv_bridge
    // Use cv_bridge to create normal image message
    sensor_msgs::msg::CompressedImage::SharedPtr image_msg =
      cv_bridge::CvImage(header, "bgr8", vis_image).toCompressedImageMsg();

    if (image_msg) {
      detections_image_pub_->publish(*image_msg);

      // Publish camera info if available
      if (camera_info_received_) {
        sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_;
        camera_info_msg.header.stamp = header.stamp;
        camera_info_pub_->publish(camera_info_msg);
      }
    } else {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Failed to convert image using cv_bridge");
    }

    // Commented out compressed image version for RViz compatibility
    // sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg =
    //   cv_bridge::CvImage(header, "bgr8", vis_image).toCompressedImageMsg();
    // if (compressed_msg) {
    //   detections_image_pub_->publish(*compressed_msg);
    // }
  }
}

}   // namespace gate_detection

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  gate_detection::Plugin,
  as2_behaviors_detection_plugin_base::DetectBase);
