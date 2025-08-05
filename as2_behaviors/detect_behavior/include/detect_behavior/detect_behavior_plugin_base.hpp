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

#ifndef DETECT_BEHAVIOR__DETECT_BEHAVIOR_PLUGIN_BASE_HPP_
#define DETECT_BEHAVIOR__DETECT_BEHAVIOR_PLUGIN_BASE_HPP_

#include <memory>
#include <vector>
#include <string>

#include <as2_core/node.hpp>
#include <as2_behavior/behavior_utils.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "as2_msgs/action/detect.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


namespace detect_behavior_plugin_base
{
class DetectBase
{
public:
  DetectBase() {}
  virtual ~DetectBase() {}
  void initialize(
    as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  bool on_activate(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
  {
    as2_msgs::action::Detect::Goal goal_candidate = *goal;
    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return true;
  }
  inline bool on_deactivate(const std::shared_ptr<std::string> & message)
  {return own_deactivate(message);}
  inline bool on_modify(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
  {
    as2_msgs::action::Detect::Goal goal_candidate = *goal;
    return own_modify(goal_candidate);
  }
  inline bool on_pause(const std::shared_ptr<std::string> & message) {return own_pause(message);}
  inline bool on_resume(const std::shared_ptr<std::string> & message) {return own_resume(message);}

  void on_execution_end(const as2_behavior::ExecutionStatus & state)
  {
    own_execution_end(state);
    return;
  }
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Detect::Goal> goal,
    std::shared_ptr<as2_msgs::action::Detect::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Detect::Result> & result_msg)
  {
    feedback_msg = std::make_shared<as2_msgs::action::Detect::Feedback>(
      feedback_);
    result_msg = std::make_shared<as2_msgs::action::Detect::Result>(result_);
    feedback_msg->rvec = det_rvec_;
    feedback_msg->tvec = det_tvec_;
    feedback_msg->confidence = confidence_;
    feedback_msg->corners = corners_;
    if (confidence_ > conf_threshold_) {
      return as2_behavior::ExecutionStatus::SUCCESS;
    } else {
      return as2_behavior::ExecutionStatus::RUNNING;
    }
  }

  virtual void image_callback(const cv::Mat image) = 0;
  virtual void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg)
  {
    distortion_model_ = cam_info_msg->distortion_model;
    camera_matrix_ = cv::Mat(3, 3, CV_64F, cam_info_msg->k.data()).clone();
    dist_coeffs_ = cv::Mat(1, cam_info_msg->d.size(), CV_64F, cam_info_msg->d.data()).clone();
    im_height = cam_info_msg->height;
    im_width = cam_info_msg->width;
  }

protected:
  virtual void ownInit() {}
  virtual bool own_activate(as2_msgs::action::Detect::Goal & goal) = 0;
  virtual bool own_modify(as2_msgs::action::Detect::Goal & goal) = 0;

  virtual bool own_deactivate(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_resume(const std::shared_ptr<std::string> & message) = 0;

  virtual void own_execution_end(const as2_behavior::ExecutionStatus & state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

protected:
  as2::Node * node_ptr_;
  as2_msgs::action::Detect::Goal goal_;
  as2_msgs::action::Detect::Feedback feedback_;
  as2_msgs::action::Detect::Result result_;

  float conf_threshold_;

  cv::Mat det_rvec_;
  cv::Mat det_tvec_;
  std::vector<int> corners_;
  float confidence_;

  std::string distortion_model_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  int im_height;
  int im_width;
};

}  // namespace detect_behavior_plugin_base

#endif  // DETECT_BEHAVIOR__DETECT_BEHAVIOR_PLUGIN_BASE_HPP_
