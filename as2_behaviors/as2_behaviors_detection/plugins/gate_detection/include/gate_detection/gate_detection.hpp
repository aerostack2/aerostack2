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
 *  \file       detect_behavior_plugin_base.hpp
 *  \brief      detect_behavior_plugin_base header file.
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************/

#ifndef GATE_DETECTION__GATE_DETECTION_HPP_
#define GATE_DETECTION__GATE_DETECTION_HPP_

#include <Eigen/Dense>
#include <string>
#include <queue>
#include <memory>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_core/node.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <yolo_inference_cpp/inference_backend.hpp>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "as2_msgs/msg/keypoint_detection_with_id_array.hpp"
#include "gate_detection/utils/arducam_interface.hpp"
#include "gate_detection/utils/common.hpp"
#include "gate_detection/utils/hungarian.hpp"
#include "as2_behaviors_detection/as2_behaviors_detection_plugin_base.hpp"


namespace gate_detection
{

struct GatesDetectionInputData
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

struct GatesDetectionOutputData
{
  std_msgs::msg::Header header;
  as2_msgs::msg::KeypointDetectionWithIDArray detections;
};

class Plugin : public as2_behaviors_detection_plugin_base::DetectBase
{
private:
  // Control time
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_inference_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Duration inference_period_{0, 0};

  std::string model_path_;
  std::string task_str_;
  int input_size_;
  bool new_frame_;

  // Preprocessing
  cv::Mat buffer_;

  // Camera calibration for rectification
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;
  bool enable_rectification_ = false;
  bool significant_distorsion_ = false;
  bool camera_info_received_ = false;

  // Unique initialization of rectification maps
  cv::Mat map1_, map2_;
  cv::cuda::GpuMat d_map1_, d_map2_;
  cv::Size rectified_size_;
  bool rect_maps_initialized_{false};

  // Inference
  double inference_frequency_;
  double confidence_threshold_;
  double nms_threshold_;
  double keypoint_threshold_;
  yolo_inference::InferenceResult inference_;

  // Detections
  size_t max_detections_;
  double max_px_;
  std::vector<cv::String> class_names_;
  std::vector<cv::String> keypoint_names_;
  as2_msgs::msg::KeypointDetectionWithIDArray detections_;

  MutexQueue<GatesDetectionOutputData> output_queue_;

  std::unique_ptr<yolo_inference::InferenceBackend> backend_ptr_;
  std::unique_ptr<ArducamInterface> arducam_interface_handler_;

  sensor_msgs::msg::CameraInfo camera_info_;

  // debug
  rclcpp::Publisher<as2_msgs::msg::KeypointDetectionWithIDArray>::SharedPtr
    detections_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr detections_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Transforms
  bool enable_id;
  bool camera_initialize_tf_ = false;
  std::string map_frame_id_;
  std::string base_link_frame_id_;
  std::string camera_frame_id_;
  std::unique_ptr<as2::tf::TfHandler> tf_handler_;
  std::map<std::string, Eigen::Isometry3d> map_T_gates_map_;
  Eigen::Isometry3d base_link_T_camera_;
  Eigen::Isometry3d map_T_base_link_;

  void compressedImagePreprocessing(
    const sensor_msgs::msg::CompressedImage::SharedPtr camera_image_msg,
    cv::Mat & image);

  bool processImage(
    const GatesDetectionInputData & input_data,
    GatesDetectionOutputData & output_data);

  void processInference(
    const cv::Mat & image,
    yolo_inference::InferenceResult & result);

  bool processDetection(
    const yolo_inference::InferenceResult & inference,
    as2_msgs::msg::KeypointDetectionWithIDArray & detections);

  void initRectificationMaps(const cv::Size & size);

  void publishDebug(
    const as2_msgs::msg::KeypointDetectionWithIDArray & detections,
    const cv::Mat & image,
    const std_msgs::msg::Header & header);

  bool updateTransform(const builtin_interfaces::msg::Time & stamp);

  bool assignGates(as2_msgs::msg::KeypointDetectionWithIDArray & detections);

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

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg) override;

  GatesDetectionInputData input_data_;
  GatesDetectionOutputData output_data_;
};
}  // namespace gate_detection
#endif
