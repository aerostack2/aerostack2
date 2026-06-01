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
 *  \file       aruco.hpp
 *  \brief      ArUco marker detection + pose estimation plugin header.
 *
 *  Detects ArUco markers in the pre-processed (BGR) image provided by
 *  PerceptionBehavior and emits one ObjectPerception per marker, with the four
 *  corners as keypoints, an axis-aligned bounding box, and (when camera
 *  intrinsics are available) the marker pose in the camera frame.
 *
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef ARUCO__ARUCO_HPP_
#define ARUCO__ARUCO_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "as2_msgs/msg/object_perception_array.hpp"
#include "as2_behaviors_object_perception/detection_plugin_base.hpp"

namespace aruco
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

private:
  static cv::aruco::PredefinedDictionaryType dictFromString(const std::string & s);

  void processImage(const cv::Mat & image, const std_msgs::msg::Header & header);

  bool isTargetClass(int marker_id) const;

  double marker_size_{0.1};
  bool estimate_pose_{true};
  bool enable_rectification_{false};

  cv::aruco::ArucoDetector detector_;

  std::vector<std::string> target_classes_;

  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  std::mutex frame_mutex_;
  bool new_frame_{false};
};

}  // namespace aruco

#endif  // ARUCO__ARUCO_HPP_
