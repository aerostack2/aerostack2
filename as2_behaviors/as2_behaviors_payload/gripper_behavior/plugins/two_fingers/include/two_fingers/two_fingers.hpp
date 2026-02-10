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
  void ownInit()
  {
    l_finger_open_ = node_ptr_->declare_parameter<double>("l_finger_open", 1.0);
    r_finger_open_ = node_ptr_->declare_parameter<double>("r_finger_open", -1.0);
    l_finger_close_ = node_ptr_->declare_parameter<double>("l_finger_close", 0.0);
    r_finger_close_ = node_ptr_->declare_parameter<double>("r_finger_close", 0.0);
    topic_l_finger_ = node_ptr_->declare_parameter<std::string>(
      "topic_l_finger", "joint/r_gripper_l_finger_joint/cmd_pos");
    topic_r_finger_ = node_ptr_->declare_parameter<std::string>(
      "topic_r_finger", "joint/r_gripper_r_finger_joint/cmd_pos");
    cbk_group_ = node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_options.callback_group = cbk_group_;
    pub_l_finger_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
      topic_l_finger_, rclcpp::SystemDefaultsQoS(),
      pub_options);
    pub_r_finger_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
      topic_r_finger_, rclcpp::SystemDefaultsQoS(),
      pub_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler two fingers plugin initialized");
  }

  bool own_activate(as2_msgs::action::GripperHandler::Goal & _goal) override
  {
    if (_goal.request_gripper) {
      closeGripper();
      RCLCPP_INFO(node_ptr_->get_logger(), "Close gripper fingers");
      feedback_.state_gripper = 1;
    } else {
      if (!_goal.request_gripper) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Open gripper fingers");
        openGripper();
        feedback_.state_gripper = 0;
      }
    }
    result_.gripper_success = true;
    return true;
  }

  bool own_modify(as2_msgs::action::GripperHandler::Goal & _goal) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler modiy accepted");
    if (_goal.request_gripper) {
      closeGripper();
      RCLCPP_INFO(node_ptr_->get_logger(), "Close gripper fingers");
      feedback_.state_gripper = 1;
    } else {
      if (!_goal.request_gripper) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Open gripper fingers");
        openGripper();
        feedback_.state_gripper = 0;
      }
    }
    result_.gripper_success = true;
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler paused");
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler resumed");
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler end");
    std_msgs::msg::Float64 open_l_position;
    open_l_position.data = l_finger_open_;
    std_msgs::msg::Float64 open_r_position;
    open_r_position.data = r_finger_open_;
    pub_l_finger_->publish(open_l_position);
    pub_r_finger_->publish(open_r_position);
    return;
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  void closeGripper()
  {
    std_msgs::msg::Float64 close_l_position;
    close_l_position.data = l_finger_close_;
    std_msgs::msg::Float64 close_r_position;
    close_r_position.data = r_finger_close_;
    pub_l_finger_->publish(close_l_position);
    pub_r_finger_->publish(close_r_position);
  }
  void openGripper()
  {
    std_msgs::msg::Float64 open_l_position;
    open_l_position.data = l_finger_open_;
    std_msgs::msg::Float64 open_r_position;
    open_r_position.data = r_finger_open_;
    pub_l_finger_->publish(open_l_position);
    pub_r_finger_->publish(open_r_position);
  }
};
}  // namespace two_fingers

#endif  // TWO_FINGERS__TWO_FINGERS_HPP_
