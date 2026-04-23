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
 *  \file       detector_yolo_gates.hpp
 *  \brief      YOLO-based gate detection plugin header.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef DETECTOR_YOLO_GATES__DETECTOR_YOLO_GATES_HPP_
#define DETECTOR_YOLO_GATES__DETECTOR_YOLO_GATES_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_core/node.hpp>
#include <as2_core/names/topics.hpp>
#include <yolo_inference_cpp/inference_backend.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "as2_msgs/msg/object_perception_array.hpp"
#include "as2_behaviors_object_perception/detection_plugin_base.hpp"

namespace detector_yolo_gates
{

class Plugin : public detection_plugin_base::DetectionBase
{
private:
  // Inference rate control
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_inference_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Duration inference_period_{0, 0};

  std::string model_path_;
  std::string task_str_;
  int input_size_;

  // Latest frame (written by image_callback, read by own_run)
  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  std::mutex frame_mutex_;
  bool new_frame_{false};

  // Inference parameters
  double inference_frequency_;
  double confidence_threshold_;
  double nms_threshold_;
  double keypoint_threshold_;
  size_t max_detections_;

  std::vector<cv::String> class_names_;
  std::vector<cv::String> keypoint_names_;

  std::unique_ptr<yolo_inference::InferenceBackend> backend_ptr_;

  // Debug publishers (optional)
  rclcpp::Publisher<as2_msgs::msg::ObjectPerceptionArray>::SharedPtr detections_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr detections_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  void processImage(const cv::Mat & image, const std_msgs::msg::Header & header);

  void processInference(const cv::Mat & image, yolo_inference::InferenceResult & result);

  bool processDetection(
    const yolo_inference::InferenceResult & inference,
    as2_msgs::msg::ObjectPerceptionArray & perceptions);

  void publishDebug(
    const as2_msgs::msg::ObjectPerceptionArray & perceptions,
    const cv::Mat & image,
    const std_msgs::msg::Header & header);

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

  void image_callback(
    const cv::Mat & image,
    const std_msgs::msg::Header & header) override;

  void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info) override;
};

}  // namespace detector_yolo_gates
#endif  // DETECTOR_YOLO_GATES__DETECTOR_YOLO_GATES_HPP_
