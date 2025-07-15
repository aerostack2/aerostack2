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

 #include "std_msgs/msg/float64.hpp"
 #include "as2_behaviors_gripper/gripper_handler_plugin_base.hpp"

namespace gripper_handler_plugin_gazebo
{
class Plugin : public gripper_handler_base::PluginBase
{
private:
  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::PublisherOptions pub_options;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_l_finger_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_r_finger_;
  // TODO(carmen): send those params via config
  double l_finger_limit = 1.0;
  double r_finger_limit = -1.0;

public:
  void ownInit()
  {
    cbk_group_ = node_ptr_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_options.callback_group = cbk_group_;
    pub_l_finger_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
      "/gz/drone0/joint/r_gripper_l_finger_joint/cmd_pos", rclcpp::SystemDefaultsQoS(),
      pub_options);
    pub_r_finger_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
      "/gz/drone0/joint/r_gripper_r_finger_joint/cmd_pos", rclcpp::SystemDefaultsQoS(),
      pub_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Gripper handler plugin initialized");
  }

  bool own_activate(as2_msgs::action::GripperHandler::Goal & _goal) override
  {
    // TODO(carmen): check if the gripper is really open or close

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

    // TODO(carmen): check if the gripper is really open or close

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
    // if (state == as2_behavior::ExecutionStatus::SUCCESS) {..
    std_msgs::msg::Float64 open_l_position;
    open_l_position.data = 1.0;
    std_msgs::msg::Float64 open_r_position;
    open_r_position.data = -1.0;
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
    close_l_position.data = 0.0;
    std_msgs::msg::Float64 close_r_position;
    close_r_position.data = 0.0;
    pub_l_finger_->publish(close_l_position);
    pub_r_finger_->publish(close_r_position);
  }
  void openGripper()
  {
    std_msgs::msg::Float64 open_l_position;
    open_l_position.data = l_finger_limit;
    std_msgs::msg::Float64 open_r_position;
    open_r_position.data = r_finger_limit;
    pub_l_finger_->publish(open_l_position);
    pub_r_finger_->publish(open_r_position);
  }
};
}  // namespace gripper_handler_plugin_gazebo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  gripper_handler_plugin_gazebo::Plugin,
  gripper_handler_base::PluginBase);
