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
 * @file gripper_handler_pluging_gazebo.cpp
 *
 * This file contains the implementation to handler the gripper in gazebo.
 *
 * @authors Carmen De Rojas Pita-Romero
 */

#include "dc_servo/dc_servo.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace dc_servo
{

void Plugin::ownInit()
{
  angle_to_open_ = node_ptr_->declare_parameter<double>("angle_to_open", 45.0);
  angle_to_close_ = node_ptr_->declare_parameter<double>("angle_to_close", 62.0);
  max_angle_ = node_ptr_->declare_parameter<double>("max_angle", 180.0);
  duty_min_ = node_ptr_->declare_parameter<double>("duty_min", 2.5);
  duty_max_ = node_ptr_->declare_parameter<double>("duty_max", 12.5);
  topic_pwm_ = node_ptr_->declare_parameter<std::string>("topic_pwm", "gripper_pwm");
  cbk_group_ = node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_options.callback_group = cbk_group_;
  pub_pwm_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
    topic_pwm_, rclcpp::SystemDefaultsQoS(),
    pub_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler dc_servo plugin initialized");
}

bool Plugin::own_activate(as2_msgs::action::GripperHandler::Goal & _goal)
{
  if (_goal.request_gripper) {
    closeGripper();
    RCLCPP_INFO(node_ptr_->get_logger(), "Close gripper ");
    feedback_.state_gripper = 1;
  } else {
    if (!_goal.request_gripper) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Open gripper ");
      openGripper();
      feedback_.state_gripper = 0;
    }
  }
  result_.gripper_success = true;
  return true;
}

bool Plugin::own_modify(as2_msgs::action::GripperHandler::Goal & _goal)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler modiy accepted");

  if (_goal.request_gripper) {
    closeGripper();
    RCLCPP_INFO(node_ptr_->get_logger(), "Close gripper ");
    feedback_.state_gripper = 1;
  } else {
    if (!_goal.request_gripper) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Open gripper ");
      openGripper();
      feedback_.state_gripper = 0;
    }
  }
  result_.gripper_success = true;
  return true;
}

bool Plugin::own_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
  return true;
}

bool Plugin::own_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler paused");
  return true;
}

bool Plugin::own_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler resumed");
  return true;
}

void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler end");
  // openGripper();
  std_msgs::msg::Float64 pwm;
  pwm.data = -1.0;
  pub_pwm_->publish(pwm);
  return;
}

as2_behavior::ExecutionStatus Plugin::own_run()
{
  return as2_behavior::ExecutionStatus::RUNNING;
}

void Plugin::closeGripper()
{
  std_msgs::msg::Float64 pwm;
  pwm.data = angle_to_pwm(angle_to_close_);
  pub_pwm_->publish(pwm);
}

void Plugin::openGripper()
{
  std_msgs::msg::Float64 pwm;
  pwm.data = angle_to_pwm(angle_to_open_);
  pub_pwm_->publish(pwm);
}

double Plugin::angle_to_pwm(double angle)
{
  double duty_cycle = (angle / max_angle_) * (duty_max_ - duty_min_) + duty_min_;
  return duty_cycle;
}

}  // namespace dc_servo

PLUGINLIB_EXPORT_CLASS(
  dc_servo::Plugin,
  gripper_behavior_plugin_base::GripperBase);
