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

/*!******************************************************************************
 *  \file       apriltag.hpp
 *  \brief      apriltag plugin header file.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef APRILTAG__APRILTAG_HPP_
#define APRILTAG__APRILTAG_HPP_

#include <string>
#include <queue>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_core/node.hpp>
#include <as2_core/names/topics.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "as2_behaviors_marker_detection/msg/marker_detection_array_with_id.hpp"


#include "as2_behaviors_marker_detection/as2_behaviors_marker_detection_plugin_base.hpp"


namespace apriltag
{

struct MarkerDetectionInputData
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

struct MarkerDetectionOutputData
{
  std_msgs::msg::Header header;
  as2_behaviors_marker_detection::msg::MarkerDetectionArrayWithID tags_detection;
};

class Plugin : public as2_behaviors_marker_detection_plugin_base::MarkerDetectBase
{
private:
  bool new_frame_;

  // Preprocessing
  cv::Mat buffer_;

  // Threshold
  double confidence_threshold_;
  double nms_threshold_;
  double keypoint_threshold_;

  // apriltag
  cv::aruco::Dictionary dictionary_;
  cv::aruco::DetectorParameters params_;
  cv::aruco::ArucoDetector detector_;
  std::vector<int> ids_;
  std::vector<std::vector<cv::Point2f>> corners_;
  std::vector<std::vector<cv::Point2f>> rejected_;


  // Camera calibration for rectification
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;
  bool enable_rectification_ = false;
  bool significant_distorsion_ = false;
  bool camera_info_received_ = false;

  // Detections
  size_t max_detections_;
  as2_behaviors_marker_detection::msg::MarkerDetectionArrayWithID detections_;

  sensor_msgs::msg::CameraInfo camera_info_;

  // debug
  rclcpp::Publisher<as2_behaviors_marker_detection::msg::MarkerDetectionArrayWithID>::SharedPtr
    detections_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr detections_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  auto dictFromString(const std::string & s);

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

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg) override;

  void compressedImagePreprocessing(
    const sensor_msgs::msg::CompressedImage::SharedPtr camera_image_msg,
    cv::Mat & image);

  MarkerDetectionInputData input_data_;
  MarkerDetectionOutputData output_data_;
};
}  // namespace apriltag
#endif
