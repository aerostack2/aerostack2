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

/*!*******************************************************************************************
 *  \file       as2_behaviors_marker_detection.hpp
 *  \authors    Alba López del Águila
 *  \copyright  Copyright (c) 2026 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/


#ifndef AS2_BEHAVIORS_MARKER_DETECTION__AS2_BEHAVIORS_MARKER_DETECTION_HPP_
#define AS2_BEHAVIORS_MARKER_DETECTION__AS2_BEHAVIORS_MARKER_DETECTION_HPP_

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_core/names/topics.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/detect.hpp"
#include "as2_behaviors_marker_detection/as2_behaviors_marker_detection_plugin_base.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using std::placeholders::_1;

namespace as2_behaviors_marker_detection
{

class MarkerDetectBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Detect>
{
public:
  explicit MarkerDetectBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MarkerDetectBehavior() {}

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg);

protected:
  bool on_activate(std::shared_ptr<const as2_msgs::action::Detect::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::Detect::Goal> goal) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Detect::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Detect::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Detect::Result> & result_msg) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  std::shared_ptr
  <pluginlib::ClassLoader<as2_behaviors_marker_detection_plugin_base::MarkerDetectBase>>
  detect_loader_;
  std::shared_ptr<as2_behaviors_marker_detection_plugin_base::MarkerDetectBase> detect_plugin_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  std::string camera_image_topic_;
  std::string camera_info_topic_;

  bool persistent_;

  std::string plugin_name_;
};

}  // namespace as2_behaviors_marker_detection
#endif  // AS2_BEHAVIORS_MARKER_DETECTION__AS2_BEHAVIORS_MARKER_DETECTION_HPP_
