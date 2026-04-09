// Copyright 2026 Universidad Politécnica de Madrid
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

/*!******************************************************************************
 *  \file       aruco.cpp
 *  \brief      aruco plugin file.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#include "aruco/aruco.hpp"

#include <filesystem>
#include <memory>
#include <algorithm>
#include <utility>
#include <vector>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace aruco
{

cv::aruco::PREDEFINED_DICTIONARY_NAME Plugin::dictFromString(const std::string & s)
{
  if (s == "4x4_50") {return cv::aruco::DICT_4X4_50;}
  if (s == "4x4_100") {return cv::aruco::DICT_4X4_100;}
  if (s == "4x4_250") {return cv::aruco::DICT_4X4_250;}
  if (s == "4x4_1000") {return cv::aruco::DICT_4X4_1000;}
  if (s == "5x5_50") {return cv::aruco::DICT_5X5_50;}
  if (s == "5x5_100") {return cv::aruco::DICT_5X5_100;}
  if (s == "5x5_250") {return cv::aruco::DICT_5X5_250;}
  if (s == "5x5_1000") {return cv::aruco::DICT_5X5_1000;}
  if (s == "6x6_50") {return cv::aruco::DICT_6X6_50;}
  if (s == "6x6_100") {return cv::aruco::DICT_6X6_100;}
  if (s == "6x6_250") {return cv::aruco::DICT_6X6_250;}
  if (s == "6x6_1000") {return cv::aruco::DICT_6X6_1000;}
  if (s == "7x7_50") {return cv::aruco::DICT_7X7_50;}
  if (s == "7x7_100") {return cv::aruco::DICT_7X7_100;}
  if (s == "7x7_250") {return cv::aruco::DICT_7X7_250;}
  if (s == "7x7_1000") {return cv::aruco::DICT_7X7_1000;}
  throw std::runtime_error("Unknown aruco dictionary: " + s);
}

void Plugin::ownInit()
{
  std::string ns = node_ptr_->get_namespace();

  const std::string dict_id =
    node_ptr_->declare_parameter<std::string>("aruco.tag_dict", "6x6_250");

  aruco_size_ =
    node_ptr_->declare_parameter<float>("aruco.size", 0.1f);

  camera_model_ =
    node_ptr_->declare_parameter<std::string>("aruco.camera_model", "pinhole");

  publish_debug_image_ =
    node_ptr_->declare_parameter<bool>("aruco.publish_debug_image", false);

  debug_image_topic_ =
    node_ptr_->declare_parameter<std::string>("aruco.debug_image_topic", "aruco/debug_image");

  debug_image_topic_ = ns + debug_image_topic_;

  aruco_pose_pub_ =
    node_ptr_->create_publisher<as2_msgs::msg::KeypointDetectionArray>(
    ns + "detections_data", as2_names::topics::sensor_measurements::qos);

  if (publish_debug_image_) {
    debug_image_pub_ =
      node_ptr_->create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_, as2_names::topics::sensor_measurements::qos);
  }

  aruco_dict_ = cv::aruco::getPredefinedDictionary(dictFromString(dict_id));

  detector_params_ = cv::aruco::DetectorParameters::create();
  detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params_->adaptiveThreshWinSizeMin = 3;
  detector_params_->adaptiveThreshWinSizeMax = 23;
  detector_params_->adaptiveThreshWinSizeStep = 10;
  detector_params_->minMarkerPerimeterRate = 0.03;
  detector_params_->maxMarkerPerimeterRate = 4.0;
  detector_params_->polygonalApproxAccuracyRate = 0.03;
  detector_params_->minCornerDistanceRate = 0.05;

  new_frame_ = false;
  camera_params_available_ = false;
  confidence_ = 0.0f;

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Aruco plugin initialized. dict=%s size=%.3f model=%s",
    dict_id.c_str(), aruco_size_, camera_model_.c_str());
}

bool Plugin::own_activate(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }

  conf_threshold_ = goal.threshold;
  target_ids_.clear();
  target_ids_.insert(target_ids_.end(), goal.target_ids.begin(), goal.target_ids.end());

  resetDetectionState();

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Aruco detector activated. Target IDs: %s",
    targetIdsToString(target_ids_).c_str());

  return true;
}

bool Plugin::own_modify(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }

  conf_threshold_ = goal.threshold;
  target_ids_.clear();
  target_ids_.insert(target_ids_.end(), goal.target_ids.begin(), goal.target_ids.end());

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "Aruco detector modified. Target IDs: %s",
    targetIdsToString(target_ids_).c_str());

  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> &)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Deactivating aruco detector");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> &)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Pausing aruco detector");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> &)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Resuming aruco detector");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus &)
{
  resetDetectionState();
  target_ids_.clear();
  RCLCPP_INFO(node_ptr_->get_logger(), "Aruco detector execution ended");
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  if (!new_frame_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  if (!camera_params_available_) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "No camera parameters available yet");
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  processImage(input_data_, output_data_);
  new_frame_ = false;

  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg, cv::Mat image)
{
  if (!image_msg || image.empty()) {
    return;
  }

  cv::Mat bgr;
  if (image.channels() == 1) {
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  } else if (image.channels() == 3) {
    if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = image.clone();
    }
  } else if (image.channels() == 4) {
    cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
  } else {
    RCLCPP_WARN(node_ptr_->get_logger(), "Unsupported image format");
    return;
  }

  input_data_.header = image_msg->header;
  input_data_.image = bgr;
  new_frame_ = true;
}

void Plugin::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
{
  if (!cam_info_msg) {
    return;
  }

  as2_behaviors_detection_plugin_base::DetectBase::camera_info_callback(cam_info_msg);
  setCameraParameters(*cam_info_msg);
}

void Plugin::setCameraParameters(const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (camera_info.k.size() != 9) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "CameraInfo.K must have size 9");
    return;
  }

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      camera_matrix_.at<double>(r, c) = camera_info.k[r * 3 + c];
    }
  }

  const int n_discoeff = static_cast<int>(camera_info.d.size());
  if (n_discoeff > 0) {
    dist_coeffs_ = cv::Mat(1, n_discoeff, CV_64F);
    for (int i = 0; i < n_discoeff; ++i) {
      dist_coeffs_.at<double>(0, i) = camera_info.d[i];
    }
  } else {
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  }

  distortion_model_ = camera_info.distortion_model;
  camera_params_available_ = true;

  if (camera_model_ == "fisheye") {
    RCLCPP_INFO(node_ptr_->get_logger(), "Using FISHEYE camera model");
    if (n_discoeff != 4) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "FISHEYE usually expects 4 distortion coefficients, received %d", n_discoeff);
    }
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Using PINHOLE camera model");
  }
}

bool Plugin::processImage(
  const ArucoDetectionInputData & input_data,
  ArucoDetectionOutputData & output_data)
{
  resetDetectionState();

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;

  cv::aruco::detectMarkers(
    input_data.image,
    aruco_dict_,
    marker_corners,
    marker_ids,
    detector_params_,
    rejected_candidates);

  if (!processDetection(marker_ids, marker_corners, output_data.detections)) {
    RCLCPP_WARN(node_ptr_->get_logger(), "No ArUco detections found");
    return false;
  }

  output_data.header = input_data.header;
  output_data.detections.header = input_data.header;

  const auto & first_detection = output_data.detections.detections.front();
  confidence_ = first_detection.confidence;

  corners_.clear();
  for (const auto & kp : first_detection.keypoints) {
    corners_.push_back(static_cast<int>(kp.x));
    corners_.push_back(static_cast<int>(kp.y));
  }

  return true;
}

bool Plugin::checkIdIsTarget(int id) const
{
  if (target_ids_.empty()) {
    return true;
  }

  return std::find(target_ids_.begin(), target_ids_.end(), static_cast<uint16_t>(id)) !=
         target_ids_.end();
}

std::string Plugin::targetIdsToString(const std::vector<uint16_t> & target_ids) const
{
  if (target_ids.empty()) {
    return "All";
  }

  std::string out;
  const int max_id_show = 10;

  if (static_cast<int>(target_ids.size()) < max_id_show) {
    for (size_t i = 0; i < target_ids.size(); ++i) {
      out += std::to_string(target_ids[i]);
      if (i + 1 < target_ids.size()) {
        out += ", ";
      }
    }
  } else {
    out = std::to_string(target_ids.front()) + ", ... , " + std::to_string(target_ids.back());
  }

  return out;
}

void Plugin::resetDetectionState()
{
  confidence_ = 0.0f;
  det_rvec_ = cv::Mat();
  det_tvec_ = cv::Mat();
  corners_.clear();
}

bool Plugin::processDetection(
  const std::vector<int> & marker_ids,
  const std::vector<std::vector<cv::Point2f>> & marker_corners,
  as2_gates_localization::msg::KeypointDetectionArray & detections_array)
{
  detections_array.detections.clear();
  detections_array.detections.reserve(marker_ids.size());

  if (marker_ids.empty() || marker_corners.empty()) {
    return false;
  }

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    const int id = marker_ids[i];

    if (!checkIdIsTarget(id)) {
      continue;
    }

    if (marker_corners[i].size() != 4) {
      continue;
    }

    const auto & corners = marker_corners[i];

    as2_gates_localization::msg::KeypointDetection detection_msg;

    detection_msg.class_id = id;
    detection_msg.label = "aruco_" + std::to_string(id);
    detection_msg.confidence = 1.0f;

    float min_x = corners[0].x;
    float min_y = corners[0].y;
    float max_x = corners[0].x;
    float max_y = corners[0].y;

    for (const auto & pt : corners) {
      min_x = std::min(min_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_x = std::max(max_x, pt.x);
      max_y = std::max(max_y, pt.y);
    }

    detection_msg.bounding_box.x1 = min_x;
    detection_msg.bounding_box.y1 = min_y;
    detection_msg.bounding_box.x2 = max_x;
    detection_msg.bounding_box.y2 = max_y;
    detection_msg.bounding_box.confidence = 1.0f;

    detection_msg.keypoints.resize(4);

    detection_msg.keypoints[0].name = "top_left";
    detection_msg.keypoints[0].x = corners[0].x;
    detection_msg.keypoints[0].y = corners[0].y;
    detection_msg.keypoints[0].confidence = 1.0f;
    detection_msg.keypoints[0].visible = true;

    detection_msg.keypoints[1].name = "top_right";
    detection_msg.keypoints[1].x = corners[1].x;
    detection_msg.keypoints[1].y = corners[1].y;
    detection_msg.keypoints[1].confidence = 1.0f;
    detection_msg.keypoints[1].visible = true;

    detection_msg.keypoints[2].name = "bottom_right";
    detection_msg.keypoints[2].x = corners[2].x;
    detection_msg.keypoints[2].y = corners[2].y;
    detection_msg.keypoints[2].confidence = 1.0f;
    detection_msg.keypoints[2].visible = true;

    detection_msg.keypoints[3].name = "bottom_left";
    detection_msg.keypoints[3].x = corners[3].x;
    detection_msg.keypoints[3].y = corners[3].y;
    detection_msg.keypoints[3].confidence = 1.0f;
    detection_msg.keypoints[3].visible = true;

    detections_array.detections.emplace_back(std::move(detection_msg));
  }

  return !detections_array.detections.empty();
}

}  // namespace aruco

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  aruco::Plugin,
  as2_behaviors_detection_plugin_base::DetectBase);
