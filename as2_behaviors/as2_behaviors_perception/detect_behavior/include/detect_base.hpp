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

/**
 * @file go_to_base.hpp
 *
 * Base class for detect plugins header
 *
 * @authors Guillermo GP-Lenza
 */

#ifndef DETECT_BEHAVIOR__DETECT_BASE_HPP_
#define DETECT_BEHAVIOR__DETECT_BASE_HPP_

#include <memory>
#include <string>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "as2_msgs/action/detect.hpp"

namespace detect_base
{


class DetectBase
{
public:
  using GoalHandleDetect = rclcpp_action::ServerGoalHandle<as2_msgs::action::Detect>;

  DetectBase() {}
  virtual ~DetectBase() {}

  void initialize(as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  bool on_activate(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
  {
    if (own_activate(goal)) {
      goal_ = *goal;
      return true;
    }
    return false;
  }

  inline bool on_modify(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> & goal)
  {
    return own_modify(goal);
  }

  inline bool on_deactivate(const std::shared_ptr<std::string> & message)
  {
    return own_deactivate(message);
  }

  inline bool on_pause(const std::shared_ptr<std::string> & message)
  {
    return own_pause(message);
  }

  inline bool on_resume(const std::shared_ptr<std::string> & message)
  {
    return own_resume(message);
  }

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Detect::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Detect::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Detect::Result> & result_msg)
  {
    as2_behavior::ExecutionStatus status = own_run();
    return status;
  }

protected:
  as2::Node * node_ptr_;
  as2_msgs::action::Detect::Goal goal_;

  virtual void ownInit() {}
  virtual bool own_activate(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Detect can not be activated, not implemented");
    return false;
  }

  virtual bool own_modify(
    std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Detect can not be modified, not implemented");
    return false;
  }

  virtual bool own_deactivate(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> & message)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Detect can not be paused, not implemented");
    return false;
  }

  virtual bool own_resume(const std::shared_ptr<std::string> & message)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Detect can not be resumed, not implemented");
    return false;
  }

  virtual as2_behavior::ExecutionStatus own_run();

public:
  virtual void image_callback(
    const sensor_msgs::msg::Image::SharedPtr & image_msg);

  virtual void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr & cam_info_msg);
};

} //namespace detect_base
#endif
