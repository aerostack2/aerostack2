// Copyright 2026 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names of its
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

#include "pnp_pose/pnp_pose.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <memory>

#include <opencv2/calib3d.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pnp_pose
{

void Plugin::ownInit()
{
  use_only_inner_corners_ = node_ptr_->declare_parameter<bool>(
    "pnp_pose.use_only_inner_corners", false);

  node_ptr_->get_parameter_or("enable_rectification", enable_rectification_, false);
  use_ransac_ = node_ptr_->declare_parameter<bool>("pnp_pose.use_ransac", true);
  apply_axis_transform_ = node_ptr_->declare_parameter<bool>("pnp_pose.apply_axis_transform", true);
  keypoint_score_threshold_ = node_ptr_->declare_parameter<double>(
    "pnp_pose.keypoint_score_threshold", 0.1);
  min_keypoints_ = node_ptr_->declare_parameter<int>("pnp_pose.min_keypoints", 4);
  ransac_iters_ = node_ptr_->declare_parameter<int>("pnp_pose.ransac_iters", 100);
  reproj_thresh_px_ = node_ptr_->declare_parameter<double>("pnp_pose.reproj_thresh_px", 8.0);
  ransac_conf_ = node_ptr_->declare_parameter<double>("pnp_pose.ransac_conf", 0.99);

  const double gate_size_exterior = node_ptr_->declare_parameter<double>(
    "pnp_pose.gate_size_exterior", 1.90);
  const double gate_size_interior = node_ptr_->declare_parameter<double>(
    "pnp_pose.gate_size_interior", 1.35);

  if (!use_only_inner_corners_) {
    object_points_map_["bottom_right_outer"] =
      cv::Point3d(gate_size_exterior / 2.0, gate_size_exterior / 2.0, 0.0);
    object_points_map_["bottom_left_outer"] =
      cv::Point3d(-gate_size_exterior / 2.0, gate_size_exterior / 2.0, 0.0);
    object_points_map_["top_right_outer"] =
      cv::Point3d(gate_size_exterior / 2.0, -gate_size_exterior / 2.0, 0.0);
    object_points_map_["top_left_outer"] =
      cv::Point3d(-gate_size_exterior / 2.0, -gate_size_exterior / 2.0, 0.0);
  }

  object_points_map_["bottom_right_inner"] =
    cv::Point3d(gate_size_interior / 2.0, gate_size_interior / 2.0, 0.0);
  object_points_map_["bottom_left_inner"] =
    cv::Point3d(-gate_size_interior / 2.0, gate_size_interior / 2.0, 0.0);
  object_points_map_["top_right_inner"] =
    cv::Point3d(gate_size_interior / 2.0, -gate_size_interior / 2.0, 0.0);
  object_points_map_["top_left_inner"] =
    cv::Point3d(-gate_size_interior / 2.0, -gate_size_interior / 2.0, 0.0);

  inverted_keypoints_map_["bottom_right_inner"] = "bottom_left_inner";
  inverted_keypoints_map_["bottom_left_inner"] = "bottom_right_inner";
  inverted_keypoints_map_["top_right_inner"] = "top_left_inner";
  inverted_keypoints_map_["top_left_inner"] = "top_right_inner";
  inverted_keypoints_map_["bottom_right_outer"] = "bottom_left_outer";
  inverted_keypoints_map_["bottom_left_outer"] = "bottom_right_outer";
  inverted_keypoints_map_["top_right_outer"] = "top_left_outer";
  inverted_keypoints_map_["top_left_outer"] = "top_right_outer";

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "pnp_pose initialized. min_keypoints=%d ransac=%s rectification=%s",
    min_keypoints_, use_ransac_ ? "true" : "false", enable_rectification_ ? "true" : "false");
}

bool Plugin::own_activate(as2_msgs::action::DetectObjects::Goal & /*goal*/)
{
  std::lock_guard<std::mutex> lock(detections_mutex_);
  has_new_detections_ = false;
  input_detections_.perceptions.clear();
  return true;
}

bool Plugin::own_modify(as2_msgs::action::DetectObjects::Goal & /*goal*/)
{
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & /*message*/)
{
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & /*message*/)
{
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & /*message*/)
{
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & /*state*/) {}

void Plugin::image_callback(const cv::Mat & /*image*/, const std_msgs::msg::Header & /*header*/)
{
  // This stage works from detections received through setInputDetections().
}

void Plugin::camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info)
{
  detection_plugin_base::DetectionBase::camera_info_callback(camera_info);
  camera_info_received_ = !camera_matrix_.empty();
}

void Plugin::setInputDetections(const as2_msgs::msg::ObjectPerceptionArray & detections)
{
  std::lock_guard<std::mutex> lock(detections_mutex_);
  input_detections_ = detections;
  has_new_detections_ = true;
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  as2_msgs::msg::ObjectPerceptionArray input_copy;
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    if (!has_new_detections_) {
      return as2_behavior::ExecutionStatus::RUNNING;
    }
    input_copy = input_detections_;
    has_new_detections_ = false;
  }

  if (!camera_info_received_) {
    latest_detections_ = input_copy;
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  as2_msgs::msg::ObjectPerceptionArray output;
  output.header = input_copy.header;
  output.perceptions.reserve(input_copy.perceptions.size());

  for (const auto & detection : input_copy.perceptions) {
    as2_msgs::msg::ObjectPerception out_det = detection;
    out_det.pose_valid = false;

    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point3d> object_points_inverted;

    if (!buildCorrespondences(detection, image_points, object_points, object_points_inverted)) {
      output.perceptions.emplace_back(std::move(out_det));
      continue;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    const bool ok_direct = computePnP(image_points, object_points, rvec, tvec);

    cv::Mat rvec_inv;
    cv::Mat tvec_inv;
    const bool ok_inverted = computePnP(image_points, object_points_inverted, rvec_inv, tvec_inv);

    if (!ok_direct && !ok_inverted) {
      output.perceptions.emplace_back(std::move(out_det));
      continue;
    }

    cv::Mat selected_rvec = rvec;
    cv::Mat selected_tvec = tvec;
    if (ok_direct && ok_inverted) {
      const double rms_direct = reprojectionRms(object_points, image_points, rvec, tvec);
      const double rms_inverted = reprojectionRms(
        object_points_inverted, image_points, rvec_inv, tvec_inv);
      if (rms_inverted < rms_direct) {
        selected_rvec = rvec_inv;
        selected_tvec = tvec_inv;
      }
    } else if (ok_inverted) {
      selected_rvec = rvec_inv;
      selected_tvec = tvec_inv;
    }

    out_det.pose_valid = true;
    out_det.header.stamp = input_copy.header.stamp;
    out_det.header.frame_id = detection.hypothesis.hypothesis.class_id.empty() ? (
      detection.header.frame_id.empty() ? input_copy.header.frame_id : detection.header.frame_id) :
      detection.hypothesis.hypothesis.class_id;
    out_det.hypothesis.pose = buildRobotPoseFromPnP(selected_rvec, selected_tvec);
    output.perceptions.emplace_back(std::move(out_det));
  }

  latest_detections_ = output;
  return as2_behavior::ExecutionStatus::RUNNING;
}

bool Plugin::buildCorrespondences(
  const as2_msgs::msg::ObjectPerception & detection,
  std::vector<cv::Point2d> & image_points,
  std::vector<cv::Point3d> & object_points,
  std::vector<cv::Point3d> & object_points_inverted) const
{
  image_points.clear();
  object_points.clear();
  object_points_inverted.clear();

  const size_t names_count = detection.keypoint_names.size();
  const size_t points_count = detection.keypoints.size();
  const size_t scores_count = detection.keypoint_scores.size();
  const size_t n = std::min(names_count, points_count);

  for (size_t i = 0; i < n; ++i) {
    const auto & kp_name = detection.keypoint_names[i];
    const auto object_it = object_points_map_.find(kp_name);
    if (object_it == object_points_map_.end()) {
      continue;
    }

    const bool has_score = i < scores_count;
    if (has_score && detection.keypoint_scores[i] < keypoint_score_threshold_) {
      continue;
    }

    const auto inverted_it = inverted_keypoints_map_.find(kp_name);
    if (inverted_it == inverted_keypoints_map_.end()) {
      continue;
    }

    const auto inv_obj_it = object_points_map_.find(inverted_it->second);
    if (inv_obj_it == object_points_map_.end()) {
      continue;
    }

    image_points.emplace_back(detection.keypoints[i].x, detection.keypoints[i].y);
    object_points.emplace_back(object_it->second);
    object_points_inverted.emplace_back(inv_obj_it->second);
  }

  return static_cast<int>(image_points.size()) >= min_keypoints_;
}

bool Plugin::computePnP(
  const std::vector<cv::Point2d> & image_points,
  const std::vector<cv::Point3d> & object_points,
  cv::Mat & rvec, cv::Mat & tvec) const
{
  if (image_points.size() < 4 || object_points.size() < 4 ||
    image_points.size() != object_points.size())
  {
    return false;
  }

  cv::Mat distortion = dist_coeffs_;
  if (distortion.empty()) {
    distortion = cv::Mat::zeros(1, 5, CV_64F);
  }
  if (enable_rectification_) {
    distortion = cv::Mat::zeros(1, distortion.cols, distortion.type());
  }

  bool success = false;
  if (use_ransac_) {
    std::vector<int> inliers;
    success = cv::solvePnPRansac(
      object_points, image_points, camera_matrix_, distortion,
      rvec, tvec, false, ransac_iters_, reproj_thresh_px_, ransac_conf_,
      inliers, cv::SOLVEPNP_AP3P);
    if (success && inliers.size() < 4) {
      success = false;
    }
  }

  if (!success) {
    success = cv::solvePnP(
      object_points, image_points, camera_matrix_, distortion, rvec, tvec, false,
      cv::SOLVEPNP_ITERATIVE);
  }

  if (!success) {
    success = cv::solvePnP(
      object_points, image_points, camera_matrix_, distortion, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);
  }

  return success;
}

double Plugin::reprojectionRms(
  const std::vector<cv::Point3d> & object_points,
  const std::vector<cv::Point2d> & image_points,
  const cv::Mat & rvec, const cv::Mat & tvec) const
{
  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, cv::Mat(), projected_points);

  if (projected_points.empty() || projected_points.size() != image_points.size()) {
    return 1e9;
  }

  double sse = 0.0;
  for (size_t i = 0; i < projected_points.size(); ++i) {
    const double dx = projected_points[i].x - image_points[i].x;
    const double dy = projected_points[i].y - image_points[i].y;
    sse += (dx * dx) + (dy * dy);
  }

  return std::sqrt(sse / static_cast<double>(projected_points.size()));
}

geometry_msgs::msg::PoseWithCovariance Plugin::buildRobotPoseFromPnP(
  const cv::Mat & rvec, const cv::Mat & tvec) const
{
  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);

  if (apply_axis_transform_) {
    const cv::Mat axis_transform = (cv::Mat_<double>(3, 3) <<
      0, -1, 0,
      0, 0, -1,
      1, 0, 0);
    rotation_matrix = rotation_matrix * axis_transform;
  }

  Eigen::Matrix3d object_to_camera_rotation;
  if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
    object_to_camera_rotation.setIdentity();
  } else {
    cv::Mat R64;
    if (rotation_matrix.type() != CV_64F) {
      rotation_matrix.convertTo(R64, CV_64F);
    } else {
      R64 = rotation_matrix;
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        object_to_camera_rotation(r, c) = R64.at<double>(r, c);
      }
    }
  }

  Eigen::Vector3d object_to_camera_translation = Eigen::Vector3d::Zero();
  cv::Mat t64;
  if (tvec.type() != CV_64F) {
    tvec.convertTo(t64, CV_64F);
  } else {
    t64 = tvec;
  }
  if (t64.total() >= 3) {
    if (t64.rows == 3) {
      object_to_camera_translation.x() = t64.at<double>(0, 0);
      object_to_camera_translation.y() = t64.at<double>(1, 0);
      object_to_camera_translation.z() = t64.at<double>(2, 0);
    } else if (t64.cols == 3) {
      object_to_camera_translation.x() = t64.at<double>(0, 0);
      object_to_camera_translation.y() = t64.at<double>(0, 1);
      object_to_camera_translation.z() = t64.at<double>(0, 2);
    }
  }

  // solvePnP returns the object pose in the camera frame. Invert it to publish
  // the camera/robot pose in the gate frame.
  const Eigen::Matrix3d camera_to_object_rotation = object_to_camera_rotation.transpose();
  const Eigen::Vector3d camera_in_object =
    -(camera_to_object_rotation * object_to_camera_translation);

  Eigen::Quaterniond quaternion(camera_to_object_rotation);
  quaternion.normalize();

  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = camera_in_object.x();
  pose.pose.position.y = camera_in_object.y();
  pose.pose.position.z = camera_in_object.z();
  pose.pose.orientation.w = quaternion.w();
  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  return pose;
}

}  // namespace pnp_pose

PLUGINLIB_EXPORT_CLASS(pnp_pose::Plugin, detection_plugin_base::DetectionBase)
