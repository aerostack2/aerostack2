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

#ifndef PNP_POSE__PNP_POSE_HPP_
#define PNP_POSE__PNP_POSE_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "as2_behaviors_object_perception/detection_plugin_base.hpp"

namespace pnp_pose
{

class Plugin : public detection_plugin_base::DetectionBase
{
public:
  Plugin() = default;
  ~Plugin() override = default;

  void ownInit() override;

  bool own_activate(as2_msgs::action::DetectObjects::Goal & goal) override;
  bool own_modify(as2_msgs::action::DetectObjects::Goal & goal) override;
  bool own_deactivate(const std::shared_ptr<std::string> & message) override;
  bool own_pause(const std::shared_ptr<std::string> & message) override;
  bool own_resume(const std::shared_ptr<std::string> & message) override;
  void own_execution_end(const as2_behavior::ExecutionStatus & state) override;
  as2_behavior::ExecutionStatus own_run() override;

  void image_callback(const cv::Mat & image, const std_msgs::msg::Header & header) override;
  void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info) override;
  void setInputDetections(const as2_msgs::msg::ObjectPerceptionArray & detections) override;

private:
  bool computePnP(
    const std::vector<cv::Point2d> & image_points,
    const std::vector<cv::Point3d> & object_points,
    cv::Mat & rvec, cv::Mat & tvec) const;

  bool buildCorrespondences(
    const as2_msgs::msg::ObjectPerception & detection,
    std::vector<cv::Point2d> & image_points,
    std::vector<cv::Point3d> & object_points,
    std::vector<cv::Point3d> & object_points_inverted) const;

  double reprojectionRms(
    const std::vector<cv::Point3d> & object_points,
    const std::vector<cv::Point2d> & image_points,
    const cv::Mat & rvec,
    const cv::Mat & tvec) const;

  geometry_msgs::msg::PoseWithCovariance buildRobotPoseFromPnP(
    const cv::Mat & rvec,
    const cv::Mat & tvec) const;

  std::unordered_map<std::string, cv::Point3d> object_points_map_;
  std::unordered_map<std::string, std::string> inverted_keypoints_map_;

  std::mutex detections_mutex_;
  as2_msgs::msg::ObjectPerceptionArray input_detections_;
  bool has_new_detections_{false};
  bool camera_info_received_{false};

  bool enable_rectification_{false};
  bool use_only_inner_corners_{false};
  bool use_ransac_{true};
  bool apply_axis_transform_{true};
  double keypoint_score_threshold_{0.1};
  int min_keypoints_{4};
  int ransac_iters_{100};
  double reproj_thresh_px_{8.0};
  double ransac_conf_{0.99};
};

}  // namespace pnp_pose

#endif  // PNP_POSE__PNP_POSE_HPP_
