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
 *  \file       tracker_tf.hpp
 *  \brief      TF-based gate tracker plugin.
 *
 *  Subscribes to raw 2D detections from detector_yolo_gates and identifies
 *  each detected gate by projecting all known gate positions (from a YAML
 *  config file) into the image using the current camera pose from TF, then
 *  matching each detection's 2D centre to the nearest projected gate.
 *
 *  Pipeline per cycle:
 *    TF: map → camera_frame  (camera pose)
 *    for each known gate: project 3D map position → 2D image point
 *    for each raw detection: compute 2D bbox centre
 *    nearest-neighbour match (pixels) → gate identity
 *    emit ObjectPerception with matched gate id + known pose, pose_valid=true
 *
 *  Parameters (declared in ownInit):
 *    tracker_tf.detections_topic   — ObjectPerceptionArray topic to subscribe to
 *    tracker_tf.reference_frame    — world frame for pose output     (default: "map")
 *    tracker_tf.tf_timeout         — TF lookup timeout in seconds    (default: 0.1)
 *    tracker_tf.max_pixel_dist     — max pixel distance for matching (default: 150.0)
 *    tracker_tf.gates_config_file  — path to YAML file with gate poses
 *
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef TRACKER_TF__TRACKER_TF_HPP_
#define TRACKER_TF__TRACKER_TF_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <as2_core/utils/tf_utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "as2_behaviors_object_perception/detection_plugin_base.hpp"
#include "as2_msgs/msg/object_perception_array.hpp"

namespace tracker_tf
{

/// Known gate entry loaded from the config file.
struct GatePose
{
  std::string name;
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
};

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

  // Image data is ignored; the plugin works from the detections subscription.
  void image_callback(const cv::Mat & image, const std_msgs::msg::Header & header) override;

private:
  /// Load known gate poses from a YAML file (gates_poses map).
  void loadGateConfig(const std::string & config_file);

  /// Callback for incoming raw detections from detector_yolo_gates.
  void detectionsCallback(const as2_msgs::msg::ObjectPerceptionArray::SharedPtr msg);

  /**
   * @brief Project a known gate's 3D map position into 2D image coordinates.
   *
   * Uses the camera intrinsics (camera_matrix_) and the pre-fetched
   * map→camera transform.
   *
   * @return Pixel coordinates, or nullopt if the gate is behind the camera.
   */
  std::optional<cv::Point2d> projectToImage(
    const GatePose & gate,
    const geometry_msgs::msg::TransformStamped & map_to_camera) const;

  /**
   * @brief Find the known gate whose projected image position is closest to
   *        detection_center, within max_pixel_dist_.
   *
   * @return Pointer to the matched GatePose, or nullptr if none qualifies.
   */
  const GatePose * matchGate(
    const cv::Point2d & detection_center,
    const geometry_msgs::msg::TransformStamped & map_to_camera) const;

  // --- Config ----------------------------------------------------------------
  std::string reference_frame_;
  double tf_timeout_;
  double max_pixel_dist_;
  std::vector<GatePose> known_gates_;

  // --- TF --------------------------------------------------------------------
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  // --- Subscription to raw detections ----------------------------------------
  std::string detections_topic_;
  rclcpp::Subscription<as2_msgs::msg::ObjectPerceptionArray>::SharedPtr detections_sub_;

  std::mutex detections_mutex_;
  as2_msgs::msg::ObjectPerceptionArray raw_detections_;
  bool new_detections_{false};

  // --- Publisher of identified detections ------------------------------------
  rclcpp::Publisher<as2_msgs::msg::ObjectPerceptionArray>::SharedPtr output_pub_;
};

}  // namespace tracker_tf

#endif  // TRACKER_TF__TRACKER_TF_HPP_
