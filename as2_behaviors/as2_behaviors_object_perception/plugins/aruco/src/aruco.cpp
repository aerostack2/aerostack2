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

/**
 * @file aruco.cpp
 *
 * ArUco marker detection + pose estimation plugin.
 *
 * Receives the pre-processed (BGR) frame from PerceptionBehavior and emits one
 * ObjectPerception per detected marker: the four corners are reported as
 * keypoints, an axis-aligned bounding box is computed from them, and — when the
 * camera intrinsics are known — the marker's 6-DoF pose in the camera frame is
 * estimated with cv::aruco::estimatePoseSingleMarkers.
 *
 * @authors Alba López del Águila
 */

#include "aruco/aruco.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace aruco
{

cv::aruco::PredefinedDictionaryType Plugin::dictFromString(const std::string & s)
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
  const std::string dict_id =
    node_ptr_->declare_parameter<std::string>("aruco.tag_dict", "6x6_250");
  marker_size_ =
    node_ptr_->declare_parameter<double>("aruco.marker_size", 0.1);
  estimate_pose_ =
    node_ptr_->declare_parameter<bool>("aruco.estimate_pose", true);

  // Shared with PerceptionBehavior: if the image is already rectified, the
  // distortion is removed and pose estimation must use zero coefficients.
  node_ptr_->get_parameter_or("enable_rectification", enable_rectification_, false);

  const cv::aruco::Dictionary dictionary =
    cv::aruco::getPredefinedDictionary(dictFromString(dict_id));

  cv::aruco::DetectorParameters detector_params;
  detector_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params.adaptiveThreshWinSizeMin = 3;
  detector_params.adaptiveThreshWinSizeMax = 23;
  detector_params.adaptiveThreshWinSizeStep = 10;
  detector_params.minMarkerPerimeterRate = 0.03;
  detector_params.maxMarkerPerimeterRate = 4.0;
  detector_params.polygonalApproxAccuracyRate = 0.03;
  detector_params.minCornerDistanceRate = 0.05;

  detector_ = cv::aruco::ArucoDetector(dictionary, detector_params);

  new_frame_ = false;

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "aruco initialised. dict=%s marker_size=%.3f m estimate_pose=%s rectification=%s",
    dict_id.c_str(), marker_size_, estimate_pose_ ? "true" : "false",
    enable_rectification_ ? "true" : "false");
}


bool Plugin::own_activate(as2_msgs::action::DetectObjects::Goal & goal)
{
  if (goal.threshold < 0.0f || goal.threshold > 1.0f) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be in [0, 1]");
    return false;
  }

  target_classes_ = goal.target_classes;

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    new_frame_ = false;
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "aruco activated. target markers: %s",
    target_classes_.empty() ? "all" : std::to_string(target_classes_.size()).c_str());
  return true;
}

bool Plugin::own_modify(as2_msgs::action::DetectObjects::Goal & goal)
{
  if (goal.threshold < 0.0f || goal.threshold > 1.0f) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be in [0, 1]");
    return false;
  }
  target_classes_ = goal.target_classes;
  RCLCPP_INFO(node_ptr_->get_logger(), "aruco modified");
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Deactivating aruco detector");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Pausing aruco detector");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Resuming aruco detector");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & /*state*/)
{
  target_classes_.clear();
  RCLCPP_INFO(node_ptr_->get_logger(), "aruco execution ended");
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  cv::Mat frame;
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!new_frame_) {
      return as2_behavior::ExecutionStatus::RUNNING;
    }
    frame = latest_frame_.clone();
    header = latest_header_;
    new_frame_ = false;
  }

  processImage(frame, header);
  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(const cv::Mat & image, const std_msgs::msg::Header & header)
{
  if (image.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(frame_mutex_);
  latest_frame_ = image;
  latest_header_ = header;
  new_frame_ = true;
}

void Plugin::camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info)
{
  // Delegate to base class for camera_matrix_ / dist_coeffs_ extraction.
  detection_plugin_base::DetectionBase::camera_info_callback(camera_info);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Detection + pose estimation
// ─────────────────────────────────────────────────────────────────────────────

bool Plugin::isTargetClass(int marker_id) const
{
  if (target_classes_.empty()) {
    return true;
  }
  return std::find(
    target_classes_.begin(), target_classes_.end(),
    std::to_string(marker_id)) != target_classes_.end();
}

void Plugin::processImage(const cv::Mat & image, const std_msgs::msg::Header & header)
{
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "aruco: unsupported image format (%d channels)", image.channels());
    return;
  }

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected;
  detector_.detectMarkers(gray, marker_corners, marker_ids, rejected);

  as2_msgs::msg::ObjectPerceptionArray perceptions;
  perceptions.header = header;

  // Pose estimation needs the camera intrinsics. camera_matrix_ is populated by
  // the base-class camera_info_callback.
  const bool can_estimate_pose = estimate_pose_ && !camera_matrix_.empty();
  cv::Mat distortion = dist_coeffs_;
  if (distortion.empty() || enable_rectification_) {
    distortion = cv::Mat::zeros(1, 5, CV_64F);
  }

  // Marker object points in its own frame (centre at origin, Z out of the plane).
  // Ordering matches ArUco's corner order: top-left, top-right, bottom-right,
  // bottom-left. cv::aruco::estimatePoseSingleMarkers was removed in OpenCV 4.7,
  // so the pose is recovered with cv::solvePnP.
  const float half = static_cast<float>(marker_size_) / 2.0f;
  const std::vector<cv::Point3f> marker_object_points = {
    {-half, half, 0.0f},
    {half, half, 0.0f},
    {half, -half, 0.0f},
    {-half, -half, 0.0f}};

  // ArUco returns corners clockwise starting from the top-left corner.
  static const std::array<const char *, 4> kCornerNames =
  {"top_left", "top_right", "bottom_right", "bottom_left"};

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    const int id = marker_ids[i];
    if (!isTargetClass(id)) {
      continue;
    }
    const auto & corners = marker_corners[i];
    if (corners.size() != 4) {
      continue;
    }

    as2_msgs::msg::ObjectPerception p;
    p.header = header;
    p.hypothesis.hypothesis.class_id = std::to_string(id);
    p.hypothesis.hypothesis.score = 1.0;
    p.pose_valid = false;

    // Axis-aligned bounding box from the four corners.
    float min_x = corners[0].x, min_y = corners[0].y;
    float max_x = corners[0].x, max_y = corners[0].y;
    for (const auto & c : corners) {
      min_x = std::min(min_x, c.x);
      min_y = std::min(min_y, c.y);
      max_x = std::max(max_x, c.x);
      max_y = std::max(max_y, c.y);
    }
    p.bbox.center.position.x = (min_x + max_x) / 2.0;
    p.bbox.center.position.y = (min_y + max_y) / 2.0;
    p.bbox.size_x = max_x - min_x;
    p.bbox.size_y = max_y - min_y;

    // Corners as keypoints.
    p.keypoints.resize(4);
    p.keypoint_names.resize(4);
    p.keypoint_scores.resize(4);
    for (size_t k = 0; k < 4; ++k) {
      p.keypoints[k].x = corners[k].x;
      p.keypoints[k].y = corners[k].y;
      p.keypoint_names[k] = kCornerNames[k];
      p.keypoint_scores[k] = 1.0f;
    }

    // Marker pose in the camera frame.
    if (can_estimate_pose) {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      if (!cv::solvePnP(
          marker_object_points, corners, camera_matrix_, distortion,
          rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
      {
        perceptions.perceptions.emplace_back(std::move(p));
        continue;
      }

      cv::Mat rotation;
      cv::Rodrigues(rvec, rotation);
      tf2::Matrix3x3 tf_rotation(
        rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
      tf2::Quaternion q;
      tf_rotation.getRotation(q);
      q.normalize();

      p.hypothesis.pose.pose.position.x = tvec[0];
      p.hypothesis.pose.pose.position.y = tvec[1];
      p.hypothesis.pose.pose.position.z = tvec[2];
      p.hypothesis.pose.pose.orientation.x = q.x();
      p.hypothesis.pose.pose.orientation.y = q.y();
      p.hypothesis.pose.pose.orientation.z = q.z();
      p.hypothesis.pose.pose.orientation.w = q.w();
      p.pose_valid = true;
    }

    perceptions.perceptions.emplace_back(std::move(p));
  }

  latest_detections_ = std::move(perceptions);
}

}  // namespace aruco

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aruco::Plugin, detection_plugin_base::DetectionBase)
