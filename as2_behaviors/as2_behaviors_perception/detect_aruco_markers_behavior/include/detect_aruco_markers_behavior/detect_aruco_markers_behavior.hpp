// Copyright 2024 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       detect_aruco_markers_behavior.hpp
 *  \brief      Aruco detector header file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef DETECT_ARUCO_MARKERS_BEHAVIOR__DETECT_ARUCO_MARKERS_BEHAVIOR_HPP_
#define DETECT_ARUCO_MARKERS_BEHAVIOR__DETECT_ARUCO_MARKERS_BEHAVIOR_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_msgs/action/detect_aruco_markers.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class DetectArucoMarkersBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::DetectArucoMarkers>
{
public:
  /**
   * @brief Construct a new Aruco Detector object
   */
  DetectArucoMarkersBehavior();

  /**
   * @brief Destroy the Aruco Detector object
   */
  ~DetectArucoMarkersBehavior() {}

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithID>::SharedPtr aruco_pose_pub_;
  std::shared_ptr<as2::sensors::Camera> aruco_img_transport_;

  std::vector<uint16_t> target_ids_;
  float aruco_size_;
  std::string camera_model_;
  std::string distorsion_model_;
  bool camera_qos_reliable_;
  bool camera_params_available_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  std::string img_encoding_;
  std::string camera_image_topic_ = "camera/image_raw";
  std::string camera_info_topic_ = "camera/camera_info";

  void loadParameters();
  void setup();
  void setCameraParameters(const sensor_msgs::msg::CameraInfo & _camera_info);
  bool checkIdIsTarget(const int _id);

public:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);
  void camerainfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::DetectArucoMarkers::Goal> & goal,
    std::shared_ptr<as2_msgs::action::DetectArucoMarkers::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::DetectArucoMarkers::Result> & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;
};

std::string targetIds2string(const std::vector<uint16_t> & target_ids);

#endif  // DETECT_ARUCO_MARKERS_BEHAVIOR__DETECT_ARUCO_MARKERS_BEHAVIOR_HPP_
