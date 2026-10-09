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
 *  ObjectPerceptionBehavior and emits one ObjectPerception per marker, with the four
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

#include <opencv2/core.hpp>

// OpenCV 4.7 moved ArUco detection into the objdetect module, replacing the free
// detectMarkers() function and the Ptr-based dictionary/parameters with the
// cv::aruco::ArucoDetector class. Ubuntu 22.04 (OpenCV 4.5.4) and 24.04 (4.6.0)
// still ship the older API, so both are supported.
#define AS2_ARUCO_HAS_DETECTOR_CLASS \
  (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

#if AS2_ARUCO_HAS_DETECTOR_CLASS
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "as2_msgs/msg/object_perception_array.hpp"
#include "as2_behaviors_object_perception/detection_plugin_base.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace aruco
{

/// Enum naming the predefined ArUco dictionaries. Renamed in OpenCV 4.7.
#if AS2_ARUCO_HAS_DETECTOR_CLASS
using PredefinedDictionary = cv::aruco::PredefinedDictionaryType;
#else
using PredefinedDictionary = cv::aruco::PREDEFINED_DICTIONARY_NAME;
#endif

class Plugin : public detection_plugin_base::DetectionBase
{
public:
  Plugin() = default;
  ~Plugin() override = default;

  /// @brief Reads parameters and builds the ArUco detector. Called once on load.
  void ownInit() override;

  /**
   * @brief Validates the goal and activates the detector.
   * @param goal  Detection goal (threshold and target marker ids).
   * @return true if the goal is valid.
   */
  bool own_activate(as2_msgs::action::DetectObjects::Goal & goal) override;

  /**
   * @brief Updates the active goal while the detector is running.
   * @param goal  New detection goal.
   * @return true if the goal is valid.
   */
  bool own_modify(as2_msgs::action::DetectObjects::Goal & goal) override;

  /// @brief Stops the detector. @return true on success.
  bool own_deactivate(const std::shared_ptr<std::string> & message) override;

  /// @brief Pauses the detector. @return true on success.
  bool own_pause(const std::shared_ptr<std::string> & message) override;

  /// @brief Resumes the detector. @return true on success.
  bool own_resume(const std::shared_ptr<std::string> & message) override;

  /// @brief Cleanup hook called when the behavior execution ends.
  void own_execution_end(const as2_behavior::ExecutionStatus & state) override;

  /**
   * @brief Runs one detection cycle on the latest frame and writes the result.
   * @return Behavior execution status (RUNNING while active).
   */
  as2_behavior::ExecutionStatus own_run() override;

  /**
   * @brief Stores the latest pre-processed frame for the next detection cycle.
   * @param image   Pre-processed (BGR) image from ObjectPerceptionBehavior.
   * @param header  Timestamp and frame of the image.
   */
  void image_callback(const cv::Mat & image, const std_msgs::msg::Header & header) override;

  /**
   * @brief Receives the camera calibration (intrinsics + distortion) for pose estimation.
   * @param camera_info  Camera info of the source image.
   */
  void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info) override;

  /**
   * @brief Publish specific debug for each plugin.
   * @param detections Input detections for publish debug.
   */
  void publishDebug(const as2_msgs::msg::ObjectPerceptionArray & detections) override;

private:
  static PredefinedDictionary dictFromString(const std::string & s);

  static cv::aruco::CornerRefineMethod refineMethodFromString(const std::string & s);

  void processImage(const cv::Mat & image, const std_msgs::msg::Header & header);

  bool isTargetClass(int marker_id) const;

  /**
   * @brief Builds a human-readable description of the active id filter, meant
   *        for the activation log: an empty target_classes means no filtering.
   * @return Sentence describing which markers will be reported.
   */
  std::string describeTargetClasses() const;

  double marker_size_{0.1};
  bool estimate_pose_{true};
  bool enable_rectification_{false};
  std::string tag_dict_name_;

#if AS2_ARUCO_HAS_DETECTOR_CLASS
  cv::aruco::ArucoDetector detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
#endif

  std::vector<std::string> target_classes_;

  mutable std::mutex target_classes_mutex_;

  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  std::mutex frame_mutex_;
  bool new_frame_{false};

  // Debug: marker poses + id text drawn as markers (e.g. for RViz).
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
};

}  // namespace aruco

#endif  // ARUCO__ARUCO_HPP_
