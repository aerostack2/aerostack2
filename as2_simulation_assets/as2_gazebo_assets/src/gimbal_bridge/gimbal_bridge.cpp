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
 *  \file       ground_truth_bridge.cpp
 *  \brief      Gazebo bridge ground truth implementation file.
 *  \authors    Javier Melero Deza
 *              Pedro Arias Pérez
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
#include "as2_gazebo_assets/gimbal_bridge.hpp"


GimbalBridge::GimbalBridge()
: Node("gimbal_bridge")
{
  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter("namespace", *model_name_);
  this->declare_parameter<std::string>("sensor_name");
  this->get_parameter("sensor_name", *sensor_name_);
  this->declare_parameter<std::string>("gimbal_name");
  this->get_parameter("gimbal_name", *gimbal_name_);
  this->declare_parameter<std::string>("control_mode");
  this->get_parameter("control_mode", *control_mode_);
  this->declare_parameter<std::string>("world_name");
  this->get_parameter("world_name", *world_name_);

  clock_ = this->get_clock();

  gz_node_ptr_ = std::make_shared<gz::transport::Node>();
  if (*control_mode_ == "position") {
    gimbal_roll_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/position/0");
    gimbal_pitch_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/position/1");
    gimbal_yaw_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/position/2");
    gimbal_cmd_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
      "/" + *model_name_ + "/platform/" + *gimbal_name_ + "/gimbal_command", 10,
      std::bind(&GimbalBridge::gimbalCmdCallback, this, std::placeholders::_1));
  } else if (*control_mode_ == "speed") {
    gimbal_roll_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/twist/0");
    gimbal_pitch_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/twist/1");
    gimbal_yaw_pub = gz_node_ptr_->Advertise<gz::msgs::Double>(
      "/" + *model_name_ + "/" + *gimbal_name_ + "/gimbal_cmd/twist/2");
    gimbal_cmd_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
      "/" + *model_name_ + "/platform/" + *gimbal_name_ + "/gimbal_command", 10,
      std::bind(&GimbalBridge::gimbalCmdCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Control mode not supported");
    throw std::runtime_error("Control mode not supported");
  }
  std::string gimbal_state_topic =
    "/world/" + *world_name_ + "/model/" + *model_name_ + "/joint_state";
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", gimbal_state_topic.c_str());
  gz_node_ptr_->Subscribe(gimbal_state_topic, this->gzJointStateCallback);

  gimbal_attitude_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "/" + *model_name_ + "/sensor_measurements/" + *gimbal_name_ + "/attitude",
    as2_names::topics::sensor_measurements::qos);

  gimbal_angular_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/" + *model_name_ + "/sensor_measurements/" + *gimbal_name_ + "/twist",
    as2_names::topics::sensor_measurements::qos);
}


void GimbalBridge::gimbalCmdCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg)
{
  gz::msgs::Double gimbal_roll;
  if (msg->control_mode == msg->POSITION_MODE && *control_mode_ != "position") {
    RCLCPP_ERROR(
      this->get_logger(), "Control mode mismatch: %s != %s", "position",
      (*control_mode_).c_str());
  } else if (msg->control_mode == msg->SPEED_MODE && *control_mode_ != "speed") {
    RCLCPP_ERROR(
      this->get_logger(), "Control mode mismatch: %s != %s", "speed",
      (*control_mode_).c_str());
  } else {
    gimbal_roll.set_data(msg->target.vector.x);
    gimbal_roll_pub.Publish(gimbal_roll);
    gz::msgs::Double gimbal_pitch;
    gimbal_pitch.set_data(msg->target.vector.y);
    gimbal_pitch_pub.Publish(gimbal_pitch);
    gz::msgs::Double gimbal_yaw;
    gimbal_yaw.set_data(msg->target.vector.z);
    gimbal_yaw_pub.Publish(gimbal_yaw);
  }
}


void GimbalBridge::gzJointStateCallback(
  const gz::msgs::Model & gz_msg,
  const gz::transport::MessageInfo & msg_info)
{
  geometry_msgs::msg::QuaternionStamped gimbal_attitude_msg;
  geometry_msgs::msg::Vector3Stamped gimbal_angular_velocity_msg;
  gimbal_attitude_msg.header.stamp = clock_.get()->now();
  gimbal_angular_velocity_msg.header.stamp = clock_.get()->now();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  for (int i = 0; i < gz_msg.joint_size(); i++) {
    if (gz_msg.joint(i).name() == *gimbal_name_ + "_roll_joint") {
      roll = gz_msg.joint(i).axis1().position();
      gimbal_angular_velocity_msg.vector.x = gz_msg.joint(i).axis1().velocity();
    } else if (gz_msg.joint(i).name() == *gimbal_name_ + "_pitch_joint") {
      pitch = gz_msg.joint(i).axis1().position();
      gimbal_angular_velocity_msg.vector.y = gz_msg.joint(i).axis1().velocity();
    } else if (gz_msg.joint(i).name() == *gimbal_name_ + "_yaw_joint") {
      yaw = gz_msg.joint(i).axis1().position();
      gimbal_angular_velocity_msg.vector.z = gz_msg.joint(i).axis1().velocity();
    }
  }
  as2::frame::eulerToQuaternion(roll, pitch, yaw, gimbal_attitude_msg.quaternion);
  gimbal_attitude_msg.header.frame_id = *model_name_ + "/" + *gimbal_name_;
  gimbal_angular_velocity_msg.header.frame_id = *model_name_ + "/" + *gimbal_name_;
  gimbal_attitude_pub_->publish(gimbal_attitude_msg);
  gimbal_angular_velocity_pub_->publish(gimbal_angular_velocity_msg);
}

std::shared_ptr<rclcpp::Clock> GimbalBridge::clock_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
GimbalBridge::gimbal_attitude_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
GimbalBridge::gimbal_angular_velocity_pub_ = nullptr;

std::shared_ptr<std::string> GimbalBridge::model_name_ = std::make_shared<std::string>();
std::shared_ptr<std::string> GimbalBridge::sensor_name_ = std::make_shared<std::string>();
std::shared_ptr<std::string> GimbalBridge::gimbal_name_ = std::make_shared<std::string>();
std::shared_ptr<std::string> GimbalBridge::control_mode_ = std::make_shared<std::string>();
std::shared_ptr<std::string> GimbalBridge::world_name_ = std::make_shared<std::string>();
