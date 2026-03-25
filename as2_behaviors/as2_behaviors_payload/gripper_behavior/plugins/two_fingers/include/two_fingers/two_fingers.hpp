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
 * @file two_fingers.hpp
 *
 * This file contains the implementation to handler the gripper in gazebo.
 *
 * @authors Carmen De Rojas Pita-Romero
 */

#ifndef TWO_FINGERS__TWO_FINGERS_HPP_
#define TWO_FINGERS__TWO_FINGERS_HPP_

#include <memory>
#include <string>
#include "std_msgs/msg/float64.hpp"
#include "gripper_behavior/gripper_behavior_plugin_base.hpp"

namespace two_fingers
{
class Plugin : public gripper_behavior_plugin_base::GripperBase
{
private:
  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::PublisherOptions pub_options;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_l_finger_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_r_finger_;
  double l_finger_open_;
  double r_finger_open_;
  double l_finger_close_;
  double r_finger_close_;
  std::string topic_l_finger_;
  std::string topic_r_finger_;

public:
  void ownInit() override;
  bool own_activate(as2_msgs::action::GripperHandler::Goal & _goal) override;
  bool own_modify(as2_msgs::action::GripperHandler::Goal & _goal) override;
  bool own_deactivate(const std::shared_ptr<std::string> & message) override;
  bool own_pause(const std::shared_ptr<std::string> & message) override;
  bool own_resume(const std::shared_ptr<std::string> & message) override;
  void own_execution_end(const as2_behavior::ExecutionStatus & state) override;
  as2_behavior::ExecutionStatus own_run() override;

private:
  void closeGripper();
  void openGripper();
};
}  // namespace two_fingers

#endif  // TWO_FINGERS__TWO_FINGERS_HPP_
