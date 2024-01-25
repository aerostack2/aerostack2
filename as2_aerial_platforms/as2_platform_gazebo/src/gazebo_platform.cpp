/*!*******************************************************************************************
 *  \file       gazebo_platform.cpp
 *  \brief      Implementation of an Gazebo UAV platform
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "gazebo_platform.hpp"

namespace gazebo_platform {
GazeboPlatform::GazeboPlatform() : as2::AerialPlatform() {
  this->declare_parameter<std::string>("cmd_vel_topic");
  std::string cmd_vel_topic_param = this->get_parameter("cmd_vel_topic").as_string();

  this->declare_parameter<std::string>("arm_topic");
  std::string arm_topic_param = this->get_parameter("arm_topic").as_string();

  // Use takeoff and land with platform for debugging purposes
  this->declare_parameter<bool>("enable_takeoff_platform");
  enable_takeoff_ = this->get_parameter("enable_takeoff_platform").as_bool();
  if (enable_takeoff_) {
    RCLCPP_INFO(this->get_logger(), "Enabled takeoff platform");
  }

  this->declare_parameter<bool>("enable_land_platform");
  enable_land_ = this->get_parameter("enable_land_platform").as_bool();
  if (enable_land_) {
    RCLCPP_INFO(this->get_logger(), "Enabled land platform");
  }

  twist_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_param, rclcpp::QoS(1));

  arm_pub_ = this->create_publisher<std_msgs::msg::Bool>(arm_topic_param, rclcpp::QoS(1));
}

void GazeboPlatform::resetCommandTwistMsg() {
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x  = 0.0f;
  twist_msg.linear.y  = 0.0f;
  twist_msg.linear.z  = 0.0f;
  twist_msg.angular.x = 0.0f;
  twist_msg.angular.y = 0.0f;
  twist_msg.angular.z = 0.0f;
  twist_pub_->publish(twist_msg);
}

bool GazeboPlatform::ownSendCommand() {
  if (control_in_.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x  = 0;
    twist_msg.linear.y  = 0;
    twist_msg.linear.z  = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;
    twist_pub_->publish(twist_msg);
    return true;
  }

  if (command_twist_msg_.twist.angular.z > yaw_rate_limit_) {
    command_twist_msg_.twist.angular.z = yaw_rate_limit_;
  } else if (command_twist_msg_.twist.angular.z < -yaw_rate_limit_) {
    command_twist_msg_.twist.angular.z = -yaw_rate_limit_;
  }
  twist_pub_->publish(command_twist_msg_.twist);
  return true;
}

bool GazeboPlatform::ownSetArmingState(bool state) {
  std_msgs::msg::Bool arm_msg;
  arm_msg.data = state;
  arm_pub_->publish(arm_msg);
  resetCommandTwistMsg();
  return true;
}

bool GazeboPlatform::ownSetOffboardControl(bool offboard) { return true; }

bool GazeboPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in) {
  RCLCPP_INFO(this->get_logger(), "Control mode: [%s]",
              as2::control_mode::controlModeToString(control_in).c_str());
  control_in_ = control_in;
  return true;
}

void GazeboPlatform::ownKillSwitch() {
  ownSetArmingState(false);
  return;
}

void GazeboPlatform::ownStopPlatform() {
  geometry_msgs::msg::Twist twist_msg_hover;
  twist_msg_hover.linear.x  = 0.0;
  twist_msg_hover.linear.y  = 0.0;
  twist_msg_hover.linear.z  = 0.0;
  twist_msg_hover.angular.x = 0.0;
  twist_msg_hover.angular.y = 0.0;
  twist_msg_hover.angular.z = 0.0;

  twist_pub_->publish(twist_msg_hover);
  return;
}

bool GazeboPlatform::ownTakeoff() {
  if (!enable_takeoff_) {
    RCLCPP_WARN(this->get_logger(), "Takeoff platform not enabled");
    return false;
  }

  // Initialize tf and state callbacks
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  twist_state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
      std::bind(&GazeboPlatform::state_callback, this, std::placeholders::_1), sub_option);

  state_received_   = false;
  rclcpp::Time time = this->now();
  while (!state_received_) {
    callback_group_executor_.spin_some();
    if ((this->now() - time).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state");
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Take off with Gazebo Platform");

  std::string base_link = as2::tf::generateTfName(this, "base_link");

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp    = this->now();
  twist_msg.header.frame_id = "earth";
  twist_msg.twist.linear.x  = 0.0;
  twist_msg.twist.linear.y  = 0.0;
  twist_msg.twist.linear.z  = 0.5;
  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = 0.0;

  geometry_msgs::msg::Twist twist_msg_hover;
  twist_msg_hover.linear.x  = 0.0;
  twist_msg_hover.linear.y  = 0.0;
  twist_msg_hover.linear.z  = 0.0;
  twist_msg_hover.angular.x = 0.0;
  twist_msg_hover.angular.y = 0.0;
  twist_msg_hover.angular.z = 0.0;

  callback_group_executor_.spin_some();

  double desired_height = current_height_ + 1.0;

  geometry_msgs::msg::TwistStamped twist_msg_flu;
  rclcpp::Rate rate(10);
  while ((desired_height - current_height_) > 0.0) {
    callback_group_executor_.spin_some();

    // Send command
    twist_msg_flu = twist_msg;
    if (tf_handler_->tryConvert(twist_msg_flu, base_link)) {
      twist_pub_->publish(twist_msg_flu.twist);
    }

    rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff complete");
  twist_pub_->publish(twist_msg_hover);

  // Clear pointers
  tf_handler_      = nullptr;
  twist_state_sub_ = nullptr;
  state_received_  = false;
  return true;
}

bool GazeboPlatform::ownLand() {
  if (!enable_land_) {
    RCLCPP_WARN(this->get_logger(), "Land platform not enabled");
    return false;
  }

  // Initialize tf and state callbacks
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  twist_state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
      std::bind(&GazeboPlatform::state_callback, this, std::placeholders::_1), sub_option);

  state_received_   = false;
  rclcpp::Time time = this->now();
  while (!state_received_) {
    callback_group_executor_.spin_some();
    if ((this->now() - time).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state");
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Take off with Gazebo Platform");

  std::string base_link = as2::tf::generateTfName(this, "base_link");

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp    = this->now();
  twist_msg.header.frame_id = "earth";
  twist_msg.twist.linear.x  = 0.0;
  twist_msg.twist.linear.y  = 0.0;
  twist_msg.twist.linear.z  = -0.5;
  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = 0.0;

  geometry_msgs::msg::Twist twist_msg_hover;
  twist_msg_hover.linear.x  = 0.0;
  twist_msg_hover.linear.y  = 0.0;
  twist_msg_hover.linear.z  = 0.0;
  twist_msg_hover.angular.x = 0.0;
  twist_msg_hover.angular.y = 0.0;
  twist_msg_hover.angular.z = 0.0;

  callback_group_executor_.spin_some();

  double desired_height = current_height_ + 1.0;

  geometry_msgs::msg::TwistStamped twist_msg_flu;
  rclcpp::Rate rate(10);
  time = this->now();
  while ((desired_height - current_height_) > 0.0) {
    callback_group_executor_.spin_some();

    if (fabs(current_vertical_speed_) < 0.05) {
      if ((this->now() - time).seconds() > 2) {
        RCLCPP_INFO(this->get_logger(), "Land complete");
        break;
      }
    } else {
      time = this->now();
    }

    // Send command
    twist_msg_flu = tf_handler_->convert(twist_msg, base_link);
    twist_pub_->publish(twist_msg_flu.twist);

    rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff complete");
  twist_pub_->publish(twist_msg_hover);

  // Clear pointers
  tf_handler_      = nullptr;
  twist_state_sub_ = nullptr;
  state_received_  = false;
  return true;
}

void GazeboPlatform::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    auto [pose_msg, twist_msg] = tf_handler_->getState(*_twist_msg, "earth", "earth",
                                                       as2::tf::generateTfName(this, "base_link"));
    current_height_            = pose_msg.pose.position.z;
    current_vertical_speed_    = twist_msg.twist.linear.z;
    state_received_            = true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

}  // namespace gazebo_platform
