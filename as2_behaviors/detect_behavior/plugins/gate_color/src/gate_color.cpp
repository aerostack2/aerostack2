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
 * @file gate_color.hpp
 *
 * This file contains the implementation to a gate detector based on rgb values.
 *
 * @authors Guillermo GP-Lenza
 */

#include "gate_color/gate_color.hpp"

namespace gate_color
{
void Plugin::ownInit()
{
  gate_color_ = node_ptr_->declare_parameter<std::vector<int>>("gate_color");
  gate_color_tolerance_ = node_ptr_->declare_parameter<std::vector<int>>(
    "gate_color_tolerance");
  gate_width_ = node_ptr_->declare_parameter<float>("gate_width");
  gate_height_ = node_ptr_->declare_parameter<float>("gate_height");
  min_cont_size_ = node_ptr_->declare_parameter<int>("min_contour_size", 100);
  aspect_ratio_th = node_ptr_->declare_parameter<float>("aspect_ratio_th", 0.2);
}

bool Plugin::own_activate(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }
  conf_threshold_ = goal.threshold;
  return true;
}

bool Plugin::own_modify(as2_msgs::action::Detect::Goal & goal)
{
  if (goal.threshold < 0.0 || goal.threshold > 1.0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Threshold must be between 0 and 1");
    return false;
  }
  conf_threshold_ = goal.threshold;
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
  RCLCPP_INFO(node_ptr_->get_logger(), "Running gate color detection");
  // Return the execution status
  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::image_callback(const cv::Mat img)
{
  cv::Mat img_rectified;
  cv::undistort(img, img_rectified, camera_matrix_, dist_coeffs_);

  cv::Scalar lower_bound(
    std::max(0L, gate_color_[0] - gate_color_tolerance_[0]),
    std::max(0L, gate_color_[1] - gate_color_tolerance_[1]),
    std::max(0L, gate_color_[2] - gate_color_tolerance_[2]));

  cv::Scalar upper_bound(
    std::min(255L, gate_color_[0] + gate_color_tolerance_[0]),
    std::min(255L, gate_color_[1] + gate_color_tolerance_[1]),
    std::min(255L, gate_color_[2] + gate_color_tolerance_[2]));

  cv::Mat mask;

  cv::inRange(img_rectified, lower_bound, upper_bound, mask);

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    if (cv::contourArea(contour) > min_cont_size_) {      // Filter small contours
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

      if (approx.size() == 4) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Detected gate with %zu vertices", approx.size());
        auto corners = getCorners(approx);
        float det_width = cv::norm(corners[0] - corners[1]);
        float det_height = cv::norm(corners[1] - corners[2]);
        if (std::abs(det_width / det_height - gate_width_ / gate_height_) < aspect_ratio_th) {
          localizeGate(corners);
        }
      }
    }
  }
}

void Plugin::localizeGate(const std::array<cv::Point, 4> & corners)
{
  cv::Mat rvec, tvec;
  std::vector<cv::Point3f> object_points = {
    {0, 0, 0}, {gate_width_, 0, 0}, {gate_width_, gate_height_, 0}, {0, gate_height_, 0}
  };

  cv::solvePnP(object_points, corners, camera_matrix_, dist_coeffs_, rvec, tvec);

  det_rvec_ = rvec;
  det_tvec_ = tvec;
  confidence_ = 1.0;
}


std::array<cv::Point, 4> Plugin::getCorners(const std::vector<cv::Point> & approx)
{
  std::array<cv::Point, 4> corners;
  corners[0] = *std::min_element(
    approx.begin(), approx.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y < b.y || (a.y == b.y && a.x < b.x);
    });
  corners[1] = *std::max_element(
    approx.begin(), approx.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y < b.y || (a.y == b.y && a.x < b.x);
    });
  corners[2] = *std::min_element(
    approx.begin(), approx.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y > b.y || (a.y == b.y && a.x < b.x);
    });
  corners[3] = *std::max_element(
    approx.begin(), approx.end(),
    [](const cv::Point & a, const cv::Point & b) {
      return a.y > b.y || (a.y == b.y && a.x < b.x);
    });
  return corners;
}

}  //  namespace gate_color

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  gate_color::Plugin,
  detect_behavior_plugin_base::DetectBase);
