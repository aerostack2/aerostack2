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
 * @file tracker_tf.cpp
 *
 * TF-based gate tracker plugin.
 *
 * Identifies detected gates by projecting all known gate positions (loaded from
 * a YAML config file) into the image using the current camera pose from TF, then
 * matching each raw detection's 2D bbox centre to the nearest projected gate.
 *
 * No pose estimation is performed: the known ground-truth pose from the config
 * file is used directly in the output ObjectPerception.
 *
 * @authors Alba López del Águila
 */

#include "tracker_tf/tracker_tf.hpp"

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <as2_core/names/topics.hpp>

namespace tracker_tf
{

// ─────────────────────────────────────────────────────────────────────────────
//  Initialisation
// ─────────────────────────────────────────────────────────────────────────────

void Plugin::ownInit()
{
  reference_frame_ =
    node_ptr_->declare_parameter<std::string>("tracker_tf.reference_frame", "map");
  tf_timeout_ =
    node_ptr_->declare_parameter<double>("tracker_tf.tf_timeout", 0.1);
  max_pixel_dist_ =
    node_ptr_->declare_parameter<double>("tracker_tf.max_pixel_dist", 150.0);
  detections_topic_ =
    node_ptr_->declare_parameter<std::string>("tracker_tf.detections_topic", "");
  const std::string output_topic =
    node_ptr_->declare_parameter<std::string>("tracker_tf.output_topic", "");
  const std::string gates_config_file =
    node_ptr_->declare_parameter<std::string>("tracker_tf.gates_config_file", "");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);
  tf_handler_->setTfTimeoutThreshold(tf_timeout_);

  if (gates_config_file.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "tracker_tf: gates_config_file parameter is empty");
  } else {
    loadGateConfig(gates_config_file);
  }

  if (detections_topic_.empty()) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "tracker_tf: detections_topic parameter is empty — no detections will be received");
  } else {
    detections_sub_ =
      node_ptr_->create_subscription<as2_msgs::msg::ObjectPerceptionArray>(
      detections_topic_,
      as2_names::topics::sensor_measurements::qos,
      [this](const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg) {
        detectionsCallback(msg);
      });
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "tracker_tf: subscribing to detections on '%s'", detections_topic_.c_str());
  }

  if (output_topic.empty()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "tracker_tf: output_topic parameter is empty — identified detections will not be published");
  } else {
    output_pub_ =
      node_ptr_->create_publisher<as2_msgs::msg::ObjectPerceptionArray>(
      output_topic,
      as2_names::topics::sensor_measurements::qos);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "tracker_tf: publishing identified detections on '%s'", output_topic.c_str());
  }

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "tracker_tf initialised. reference_frame='%s'  max_pixel_dist=%.0f px  %zu known gate(s)",
    reference_frame_.c_str(), max_pixel_dist_, known_gates_.size());
}

void Plugin::loadGateConfig(const std::string & config_file)
{
  try {
    YAML::Node root = YAML::LoadFile(config_file);
    const YAML::Node gates = root["gates_poses"];
    if (!gates || !gates.IsMap()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "tracker_tf: 'gates_poses' key not found or not a map in '%s'",
        config_file.c_str());
      return;
    }
    for (const auto & entry : gates) {
      GatePose gp;
      gp.name = entry.first.as<std::string>();
      const auto vec = entry.second.as<std::vector<double>>();
      if (vec.size() < 4) {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "tracker_tf: gate '%s' has only %zu values (expected [x,y,z,yaw]) — skipping",
          gp.name.c_str(), vec.size());
        continue;
      }
      gp.x = vec[0]; gp.y = vec[1]; gp.z = vec[2]; gp.yaw = vec[3];
      known_gates_.push_back(std::move(gp));
    }
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "tracker_tf: loaded %zu gate(s) from '%s'",
      known_gates_.size(), config_file.c_str());
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "tracker_tf: failed to load gate config '%s': %s",
      config_file.c_str(), e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Behaviour lifecycle
// ─────────────────────────────────────────────────────────────────────────────

bool Plugin::own_activate(as2_msgs::action::DetectObjects::Goal & /*goal*/)
{
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    new_detections_ = false;
    raw_detections_.perceptions.clear();
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf activated");
  return true;
}

bool Plugin::own_modify(as2_msgs::action::DetectObjects::Goal & /*goal*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf modified");
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf deactivated");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf paused");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & /*message*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf resumed");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & /*state*/)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "tracker_tf execution ended");
}

void Plugin::image_callback(const cv::Mat & /*image*/, const std_msgs::msg::Header & /*header*/)
{
  // Image data is not used; camera intrinsics come from camera_info_callback (base class).
}

// ─────────────────────────────────────────────────────────────────────────────
//  Detections subscription callback
// ─────────────────────────────────────────────────────────────────────────────

void Plugin::detectionsCallback(const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(detections_mutex_);
  raw_detections_ = *msg;
  new_detections_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main run loop
// ─────────────────────────────────────────────────────────────────────────────

as2_behavior::ExecutionStatus Plugin::own_run()
{
  as2_msgs::msg::ObjectPerceptionArray raw_copy;
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    if (!new_detections_) {
      return as2_behavior::ExecutionStatus::RUNNING;
    }
    raw_copy = raw_detections_;
    new_detections_ = false;
  }

  if (camera_matrix_.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "tracker_tf: camera matrix not yet available — skipping frame");
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  const std::string cam_frame = camera_info_.header.frame_id;
  if (cam_frame.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "tracker_tf: camera frame ID not yet known — skipping frame");
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // Fetch map→camera transform once for this whole cycle.
  geometry_msgs::msg::TransformStamped map_to_camera;
  try {
    const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(tf_timeout_));
    map_to_camera = tf_handler_->getTfBuffer()->lookupTransform(
      cam_frame, reference_frame_,
      tf2::TimePointZero, timeout_ns);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "tracker_tf: TF '%s'→'%s' unavailable: %s",
      reference_frame_.c_str(), cam_frame.c_str(), ex.what());
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  as2_msgs::msg::ObjectPerceptionArray output;
  output.header = raw_copy.header;
  output.header.frame_id = reference_frame_;

  for (const auto & det : raw_copy.perceptions) {
    // 2D centre from bounding box.
    const cv::Point2d center(
      (det.bbox_min.x + det.bbox_max.x) / 2.0,
      (det.bbox_min.y + det.bbox_max.y) / 2.0);

    const GatePose * matched = matchGate(center, map_to_camera);
    if (!matched) {
      RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "tracker_tf: no known gate projected within %.0f px of detection centre (%.0f, %.0f)",
        max_pixel_dist_, center.x, center.y);
      continue;
    }

    as2_msgs::msg::ObjectPerception p;
    p.id = matched->name;
    p.class_name = matched->name;
    p.confidence = det.confidence;
    p.pose_valid = true;

    p.pose.header.stamp = raw_copy.header.stamp;
    p.pose.header.frame_id = reference_frame_;
    p.pose.pose.position.x = matched->x;
    p.pose.pose.position.y = matched->y;
    p.pose.pose.position.z = matched->z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, matched->yaw);
    p.pose.pose.orientation.x = q.x();
    p.pose.pose.orientation.y = q.y();
    p.pose.pose.orientation.z = q.z();
    p.pose.pose.orientation.w = q.w();

    // Forward 2D detection data from the raw detection.
    p.keypoints = det.keypoints;
    p.keypoint_names = det.keypoint_names;
    p.keypoint_scores = det.keypoint_scores;
    p.bbox_min = det.bbox_min;
    p.bbox_max = det.bbox_max;

    output.perceptions.emplace_back(std::move(p));

    RCLCPP_DEBUG(
      node_ptr_->get_logger(),
      "tracker_tf: detection at (%.0f, %.0f) px → '%s'",
      center.x, center.y, matched->name.c_str());
  }

  latest_detections_ = output;

  if (output_pub_) {
    output_pub_->publish(output);
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Reverse projection: known gate 3D → 2D image point
// ─────────────────────────────────────────────────────────────────────────────

std::optional<cv::Point2d> Plugin::projectToImage(
  const GatePose & gate,
  const geometry_msgs::msg::TransformStamped & map_to_camera) const
{
  // Transform gate centre from map frame to camera frame.
  geometry_msgs::msg::PointStamped gate_map, gate_cam;
  gate_map.header.frame_id = reference_frame_;
  gate_map.point.x = gate.x;
  gate_map.point.y = gate.y;
  gate_map.point.z = gate.z;
  tf2::doTransform(gate_map, gate_cam, map_to_camera);

  // Gate must be in front of the camera (z > 0 in camera frame).
  if (gate_cam.point.z <= 0.0) {
    return std::nullopt;
  }

  // Pinhole projection using the camera intrinsic matrix.
  const double fx = camera_matrix_.at<double>(0, 0);
  const double fy = camera_matrix_.at<double>(1, 1);
  const double cx = camera_matrix_.at<double>(0, 2);
  const double cy = camera_matrix_.at<double>(1, 2);

  const double u = fx * gate_cam.point.x / gate_cam.point.z + cx;
  const double v = fy * gate_cam.point.y / gate_cam.point.z + cy;

  return cv::Point2d(u, v);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Gate matching: nearest projected gate within max_pixel_dist_
// ─────────────────────────────────────────────────────────────────────────────

const GatePose * Plugin::matchGate(
  const cv::Point2d & detection_center,
  const geometry_msgs::msg::TransformStamped & map_to_camera) const
{
  const GatePose * best = nullptr;
  double best_dist = max_pixel_dist_;

  for (const auto & gate : known_gates_) {
    const auto proj = projectToImage(gate, map_to_camera);
    if (!proj) {
      continue;  // gate is behind the camera
    }
    const double dist = cv::norm(*proj - detection_center);
    if (dist < best_dist) {
      best_dist = dist;
      best = &gate;
    }
  }
  return best;
}

}  // namespace tracker_tf

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tracker_tf::Plugin, detection_plugin_base::DetectionBase)
