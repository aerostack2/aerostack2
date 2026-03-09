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
 * @file aruco.cpp
 *
 * This file contains the implementation to a marker detector.
 *
 * @authors Alba López del Águila
 */

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "aruco/aruco.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace aruco
{

auto Plugin::dictFromString(const std::string & s)
{
  if (s == "4x4_50") {
    return cv::aruco::DICT_4X4_50;
  } else if (s == "4x4_100") {
    return cv::aruco::DICT_4X4_100;
  } else if (s == "4x4_250") {
    return cv::aruco::DICT_4X4_250;
  } else if (s == "4x4_1000") {
    return cv::aruco::DICT_4X4_1000;
  } else if (s == "5x5_50") {
    return cv::aruco::DICT_5X5_50;
  } else if (s == "5x5_100") {
    return cv::aruco::DICT_5X5_100;
  } else if (s == "5x5_250") {
    return cv::aruco::DICT_5X5_250;
  } else if (s == "5x5_1000") {
    return cv::aruco::DICT_5X5_1000;
  } else if (s == "6x6_50") {
    return cv::aruco::DICT_6X6_50;
  } else if (s == "6x6_100") {
    return cv::aruco::DICT_6X6_100;
  } else if (s == "6x6_250") {
    return cv::aruco::DICT_6X6_250;
  } else if (s == "6x6_1000") {
    return cv::aruco::DICT_6X6_1000;
  } else if (s == "7x7_50") {
    return cv::aruco::DICT_7X7_50;
  } else if (s == "7x7_100") {
    return cv::aruco::DICT_7X7_100;
  } else if (s == "7x7_250") {
    return cv::aruco::DICT_7X7_250;
  } else if (s == "7x7_1000") {
    return cv::aruco::DICT_7X7_1000;
  }
  throw std::runtime_error("Unknown aruco family: " + s);
}

void Plugin::ownInit()
{
  const std::string dict_id =
    node_ptr_->declare_parameter<std::string>("aruco.tag_dict");

  const bool enable_debug =
    node_ptr_->declare_parameter<bool>("aruco.enable_debug", false);

  new_frame_ = false;

  if (enable_debug) {
    const std::string detections_data_topic =
      node_ptr_->declare_parameter<std::string>(
      "aruco.debug.detections_data_topic", "");

    if (!detections_data_topic.empty()) {
      detections_data_pub_ =
        node_ptr_->create_publisher<
        as2_behaviors_marker_detection::msg::MarkerDetectionArrayWithID>(
        detections_data_topic, as2_names::topics::sensor_measurements::qos);

      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing detections to %s",
        detections_data_topic.c_str());
    }

    const std::string debug_detections_image_topic =
      node_ptr_->declare_parameter<std::string>(
      "aruco.debug.debug_detections_image_topic", "");

    if (!debug_detections_image_topic.empty()) {
      detections_image_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::CompressedImage>(
        debug_detections_image_topic + "/compressed",
        as2_names::topics::sensor_measurements::qos);

      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing detections image to %s",
        debug_detections_image_topic.c_str());

      camera_info_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        debug_detections_image_topic + "/camera_info",
        as2_names::topics::sensor_measurements::qos);

      RCLCPP_INFO(
        node_ptr_->get_logger(), "Publishing camera info to %s",
        (debug_detections_image_topic + "/camera_info").c_str());
    }
  }

  dictionary_ = cv::aruco::getPredefinedDictionary(dictFromString(dict_id));

  params_ = cv::aruco::DetectorParameters();
  params_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params_.adaptiveThreshWinSizeMin = 3;
  params_.adaptiveThreshWinSizeMax = 23;
  params_.adaptiveThreshWinSizeStep = 10;
  params_.minMarkerPerimeterRate = 0.03;
  params_.maxMarkerPerimeterRate = 4.0;
  params_.polygonalApproxAccuracyRate = 0.03;
  params_.minCornerDistanceRate = 0.05;

  detector_ = cv::aruco::ArucoDetector(dictionary_, params_);
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
  RCLCPP_INFO(node_ptr_->get_logger(), "Execution ended");
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  if (!new_frame_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  detector_.detectMarkers(input_data_.image, corners_, ids_, rejected_);
  new_frame_ = false;

  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
  if (!image_msg) {
    return;
  }

  cv::Mat img;
  compressedImagePreprocessing(image_msg, img);
  if (img.empty()) {
    return;
  }

  input_data_.header = image_msg->header;
  input_data_.image = img;
  new_frame_ = true;
}

void Plugin::compressedImagePreprocessing(
  const sensor_msgs::msg::CompressedImage::SharedPtr camera_image_msg,
  cv::Mat & image)
{
  if (!camera_image_msg) {
    image.release();
    return;
  }

  if (camera_image_msg->data.empty()) {
    image.release();
    return;
  }

  cv::Mat raw(1, static_cast<int>(camera_image_msg->data.size()), CV_8UC1,
    const_cast<unsigned char *>(camera_image_msg->data.data()));

  image = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (image.empty()) {
    return;
  }

  if (image.channels() == 3) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }
}

}  // namespace aruco

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  aruco::Plugin,
  as2_behaviors_marker_detection_plugin_base::MarkerDetectBase);
