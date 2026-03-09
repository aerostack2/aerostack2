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
 *  \file       as2_behaviors_marker_detection.cpp
 *  \brief      Maker detector node file.
 *  \authors    Alba López del Águila
 *  \copyright  Copyright (c) 2026 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "as2_behaviors_marker_detection/as2_behaviors_marker_detection.hpp"
#include <string>
#include <memory>

namespace as2_behaviors_marker_detection
{

MarkerDetectBehavior::MarkerDetectBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::Detect>("MarkerDetectBehavior",
    options)
{
  RCLCPP_DEBUG(this->get_logger(), "Detect Behavior ready!");
}


bool MarkerDetectBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
{
  if (!detect_plugin_->on_activate(goal)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate detect plugin");
    return false;
  }
  // Additional activation logic can be added here if neededinit
  RCLCPP_INFO(this->get_logger(), "Detect behavior activated successfully");
  return true;
}

bool MarkerDetectBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::Detect::Goal> goal)
{
  return detect_plugin_->on_modify(goal);
}

bool MarkerDetectBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_deactivate(message);
}

bool MarkerDetectBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_pause(message);
}

bool MarkerDetectBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return detect_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus MarkerDetectBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Detect::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Detect::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Detect::Result> & result_msg)
{
  auto status = detect_plugin_->on_run(goal, feedback_msg, result_msg);
  if (persistent_) {
    return as2_behavior::ExecutionStatus::RUNNING;
  } else {
    return status;
  }
}

void MarkerDetectBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
}

}  // namespace as2_behaviors_marker_detection
