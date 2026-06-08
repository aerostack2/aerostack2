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
 *  \file       detection_plugin_base.hpp
 *  \brief      Base class for object detection plugins.
 *  \authors    Alba López del Águila
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_OBJECT_PERCEPTION__DETECTION_PLUGIN_BASE_HPP_
#define AS2_BEHAVIORS_OBJECT_PERCEPTION__DETECTION_PLUGIN_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <as2_core/node.hpp>
#include <as2_behavior/behavior_utils.hpp>
#include "as2_msgs/action/detect_objects.hpp"
#include "as2_msgs/msg/object_perception_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"

namespace detection_plugin_base
{

/**
 * @brief Abstract base class for object detection plugins.
 *
 * Plugins receive pre-processed images (cv::Mat) from the central
 * PerceptionBehavior and output detections via latest_detections_.
 * Image preprocessing (decompression, rectification) is handled by
 * the behavior server, not by individual plugins.
 */
class DetectionBase
{
public:
  DetectionBase() {}
  virtual ~DetectionBase() {}

  /**
   * @brief Connects the plugin to the ROS 2 node and initializes it.
   * @param node_ptr  Owning behavior node, shared with the plugin.
   *
   * Called once after construction. Delegates to ownInit().
   */
  void initialize(as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
    ownInit();
  }

  /**
   * @brief Activates the plugin with a new detection goal.
   * @param goal  Requested detection goal.
   * @return true if the goal is accepted (stored as the active goal).
   */
  bool on_activate(std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
  {
    as2_msgs::action::DetectObjects::Goal goal_candidate = *goal;
    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  /**
   * @brief Updates the goal while the plugin is active.
   * @param goal  New detection goal.
   * @return true if the new goal is accepted.
   */
  bool on_modify(std::shared_ptr<const as2_msgs::action::DetectObjects::Goal> goal)
  {
    as2_msgs::action::DetectObjects::Goal goal_candidate = *goal;
    return own_modify(goal_candidate);
  }

  /// @brief Deactivates the plugin. @return true on success.
  inline bool on_deactivate(const std::shared_ptr<std::string> & message)
  {return own_deactivate(message);}

  /// @brief Pauses the plugin. @return true on success.
  inline bool on_pause(const std::shared_ptr<std::string> & message)
  {return own_pause(message);}

  /// @brief Resumes the plugin. @return true on success.
  inline bool on_resume(const std::shared_ptr<std::string> & message)
  {return own_resume(message);}

  /// @brief Cleanup hook called when the behavior execution ends.
  void on_execution_end(const as2_behavior::ExecutionStatus & state)
  {own_execution_end(state);}

  /// @brief Runs one processing cycle. @return Behavior execution status.
  inline as2_behavior::ExecutionStatus on_run()
  {return own_run();}

  /**
   * @brief Receive a pre-processed image frame from the behavior server.
   *
   * The image has already been decompressed and optionally rectified.
   * Plugins store the frame and run inference in own_run().
   */
  virtual void image_callback(
    const cv::Mat & image,
    const std_msgs::msg::Header & header) = 0;

  /**
   * @brief Receives the camera calibration.
   * @param camera_info  Camera info of the source image.
   *
   * Default implementation extracts the camera matrix and distortion coefficients.
   */
  virtual void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info)
  {
    camera_info_ = camera_info;
    distortion_model_ = camera_info.distortion_model;
    camera_matrix_ = cv::Mat(
      3, 3, CV_64F,
      const_cast<double *>(camera_info.k.data())).clone();
    if (!camera_info.d.empty()) {
      dist_coeffs_ = cv::Mat(
        1, static_cast<int>(camera_info.d.size()), CV_64F,
        const_cast<double *>(camera_info.d.data())).clone();
    }
  }

  /**
   * @brief Returns the latest detection results.
   * @return Detections produced by the last cycle. Read by PerceptionBehavior.
   */
  as2_msgs::msg::ObjectPerceptionArray getDetections() const
  {return latest_detections_;}

  /**
   * @brief Feeds detections from a previous or external pipeline stage.
   * @param detections  Input detections for this stage to consume.
   */
  virtual void setInputDetections(const as2_msgs::msg::ObjectPerceptionArray & detections)
  {
    latest_detections_ = detections;
  }

  /**
   * @brief Publish specific debug for each plugin.
   * @param detections Input detections for publish debug.
   */
  virtual void publishDebug(const as2_msgs::msg::ObjectPerceptionArray & /*detections*/) {}

protected:
  virtual void ownInit() {}

  virtual bool own_activate(as2_msgs::action::DetectObjects::Goal & goal) = 0;

  virtual bool own_modify(as2_msgs::action::DetectObjects::Goal & goal) = 0;

  virtual bool own_deactivate(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_resume(const std::shared_ptr<std::string> & message) = 0;

  virtual as2_behavior::ExecutionStatus own_run() = 0;

  virtual void own_execution_end(const as2_behavior::ExecutionStatus & state) = 0;

  as2::Node * node_ptr_ = nullptr;
  as2_msgs::action::DetectObjects::Goal goal_;

  sensor_msgs::msg::CameraInfo camera_info_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::string distortion_model_;

  as2_msgs::msg::ObjectPerceptionArray latest_detections_;
};

}  // namespace detection_plugin_base

#endif  // AS2_BEHAVIORS_OBJECT_PERCEPTION__DETECTION_PLUGIN_BASE_HPP_
