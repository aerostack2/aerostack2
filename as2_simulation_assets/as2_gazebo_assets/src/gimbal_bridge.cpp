/*!*******************************************************************************************
 *  \file       ground_truth_bridge.cpp
 *  \brief      Ignition bridge ground truth implementation file.
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
#include <iostream>
#include <memory>
#include <string>

#include <math.h>
#include <as2_core/names/topics.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <rclcpp/clock.hpp>
#include <ros_gz_bridge/convert.hpp>
#include "as2_msgs/msg/gimbal_control.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

class GimbalBridge : public rclcpp::Node {
public:
  GimbalBridge() : Node("gimbal_bridge") {
    this->declare_parameter<std::string>("namespace", "");
    this->get_parameter("namespace", model_name_);
    this->declare_parameter<std::string>("sensor_name");
    this->get_parameter("sensor_name", sensor_name_);
    this->declare_parameter<std::string>("gimbal_name");
    this->get_parameter("gimbal_name", gimbal_name_);
    this->declare_parameter<std::string>("control_mode");
    this->get_parameter("control_mode", control_mode_);
    this->declare_parameter<std::string>("world_name");
    this->get_parameter("world_name", world_name_);

    clock_ = this->get_clock();

    ign_node_ptr_ = std::make_shared<ignition::transport::Node>();
    if (control_mode_ == "position") {
      gimbal_roll_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/position/0");
      gimbal_pitch_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/position/1");
      gimbal_yaw_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/position/2");
      gimbal_cmd_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
          "/" + model_name_ + "/platform/" + gimbal_name_ + "/gimbal_command", 10,
          std::bind(&GimbalBridge::gimbalCmdCallback, this, std::placeholders::_1));
    } else if (control_mode_ == "speed") {
      gimbal_roll_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/twist/0");
      gimbal_pitch_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/twist/1");
      gimbal_yaw_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
          "/" + model_name_ + "/" + gimbal_name_ + "/gimbal_cmd/twist/2");
      gimbal_cmd_sub_ = this->create_subscription<as2_msgs::msg::GimbalControl>(
          "/" + model_name_ + "/platform/" + gimbal_name_ + "/gimbal_command", 10,
          std::bind(&GimbalBridge::gimbalCmdCallback, this, std::placeholders::_1));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Control mode not supported");
      throw std::runtime_error("Control mode not supported");
    }
    std::string gimbal_state_topic =
        "/world/" + world_name_ + "/model/" + model_name_ + "/joint_state";
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", gimbal_state_topic.c_str());
    ign_node_ptr_->Subscribe(gimbal_state_topic, this->ignitionJointStateCallback);

    gimbal_attitude_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "/" + model_name_ + "/sensor_measurements/" + gimbal_name_ + "/attitude",
        as2_names::topics::sensor_measurements::qos);

    gimbal_angular_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/" + model_name_ + "/sensor_measurements/" + gimbal_name_ + "/twist",
        as2_names::topics::sensor_measurements::qos);
  }

private:
  static std::string model_name_;
  static std::string gimbal_name_;
  static std::string sensor_name_;
  static std::string control_mode_;
  static std::string world_name_;

  void gimbalCmdCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg) {
    ignition::msgs::Double gimbal_roll;
    if (msg->control_mode == msg->POSITION_MODE && control_mode_ != "position") {
      RCLCPP_ERROR(this->get_logger(), "Control mode mismatch: %s != %s", "position",
                   control_mode_.c_str());
    } else if (msg->control_mode == msg->SPEED_MODE && control_mode_ != "speed") {
      RCLCPP_ERROR(this->get_logger(), "Control mode mismatch: %s != %s", "speed",
                   control_mode_.c_str());
    } else {
      gimbal_roll.set_data(msg->target.vector.x);
      gimbal_roll_pub.Publish(gimbal_roll);
      ignition::msgs::Double gimbal_pitch;
      gimbal_pitch.set_data(msg->target.vector.y);
      gimbal_pitch_pub.Publish(gimbal_pitch);
      ignition::msgs::Double gimbal_yaw;
      gimbal_yaw.set_data(msg->target.vector.z);
      gimbal_yaw_pub.Publish(gimbal_yaw);
    }
  }

private:
  rclcpp::Subscription<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_cmd_sub_;
  static rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr gimbal_attitude_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      gimbal_angular_velocity_pub_;

  static std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
  ignition::transport::Node::Publisher gimbal_roll_pub;
  ignition::transport::Node::Publisher gimbal_pitch_pub;
  ignition::transport::Node::Publisher gimbal_yaw_pub;

  static void ignitionJointStateCallback(const ignition::msgs::Model &ign_msg,
                                         const ignition::transport::MessageInfo &msg_info) {
    geometry_msgs::msg::QuaternionStamped gimbal_attitude_msg;
    geometry_msgs::msg::Vector3Stamped gimbal_angular_velocity_msg;
    gimbal_attitude_msg.header.stamp         = clock_.get()->now();
    gimbal_angular_velocity_msg.header.stamp = clock_.get()->now();
    double roll                              = 0.0;
    double pitch                             = 0.0;
    double yaw                               = 0.0;
    for (int i = 0; i < ign_msg.joint_size(); i++) {
      if (ign_msg.joint(i).name() == gimbal_name_ + "_roll_joint") {
        roll                                 = ign_msg.joint(i).axis1().position();
        gimbal_angular_velocity_msg.vector.x = ign_msg.joint(i).axis1().velocity();
      } else if (ign_msg.joint(i).name() == gimbal_name_ + "_pitch_joint") {
        pitch                                = ign_msg.joint(i).axis1().position();
        gimbal_angular_velocity_msg.vector.y = ign_msg.joint(i).axis1().velocity();
      } else if (ign_msg.joint(i).name() == gimbal_name_ + "_yaw_joint") {
        yaw                                  = ign_msg.joint(i).axis1().position();
        gimbal_angular_velocity_msg.vector.z = ign_msg.joint(i).axis1().velocity();
      }
    }
    as2::frame::eulerToQuaternion(roll, pitch, yaw, gimbal_attitude_msg.quaternion);
    gimbal_attitude_msg.header.frame_id         = model_name_ + "/" + gimbal_name_;
    gimbal_angular_velocity_msg.header.frame_id = model_name_ + "/" + gimbal_name_;
    gimbal_attitude_pub_->publish(gimbal_attitude_msg);
    gimbal_angular_velocity_pub_->publish(gimbal_angular_velocity_msg);
  };
};

std::string GimbalBridge::model_name_   = "";
std::string GimbalBridge::sensor_name_  = "";
std::string GimbalBridge::gimbal_name_  = "";
std::string GimbalBridge::control_mode_ = "";
std::string GimbalBridge::world_name_   = "";

std::shared_ptr<rclcpp::Clock> GimbalBridge::clock_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
    GimbalBridge::gimbal_attitude_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    GimbalBridge::gimbal_angular_velocity_pub_ = nullptr;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalBridge>());
  rclcpp::shutdown();
  return 0;
}