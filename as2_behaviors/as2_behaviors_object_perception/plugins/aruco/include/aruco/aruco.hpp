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
 *  \brief      aruco header file.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef ARUCO__ARUCO_HPP_
#define ARUCO__ARUCO_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "as2_msgs/msg/keypoint_detection_array.hpp"
#include "as2_behaviors_object_perception/object_perception_plugin_base.hpp"

namespace aruco
{

struct ArucoDetectionInputData
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

struct ArucoDetectionOutputData
{
  std_msgs::msg::Header header;
  as2_msgs::msg::KeypointDetectionArray detections;
};

class Plugin : public object_perception_plugin_base::PerceptionBase
{
private:
  bool new_frame_{false};
  bool camera_params_available_{false};

  float aruco_size_{0.1f};
  std::string camera_model_{"pinhole"};

  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  std::vector<uint16_t> target_ids_;

  bool processImage(
    const ArucoDetectionInputData & input_data,
    ArucoDetectionOutputData & output_data);

  bool processDetection(
    const std::vector<int> & marker_ids,
    const std::vector<std::vector<cv::Point2f>> & marker_corners,
    as2_msgs::msg::KeypointDetectionArray & detections_array);

  void setCameraParameters(const sensor_msgs::msg::CameraInfo & camera_info);
  bool checkIdIsTarget(int id) const;
  std::string targetIdsToString(const std::vector<uint16_t> & target_ids) const;
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictFromString(const std::string & s);
  void resetDetectionState();

public:
  Plugin() = default;
  ~Plugin() override = default;

  void ownInit() override;

  bool own_activate(as2_msgs::action::Detect::Goal & goal) override;
  bool own_modify(as2_msgs::action::Detect::Goal & goal) override;
  bool own_deactivate(const std::shared_ptr<std::string> & message) override;
  bool own_pause(const std::shared_ptr<std::string> & message) override;
  bool own_resume(const std::shared_ptr<std::string> & message) override;

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override;
  as2_behavior::ExecutionStatus own_run() override;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg, cv::Mat image) override;
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg) override;

  ArucoDetectionInputData input_data_;
  ArucoDetectionOutputData output_data_;
};

}  // namespace aruco

#endif
