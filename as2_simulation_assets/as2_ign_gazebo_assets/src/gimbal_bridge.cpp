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
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <rclcpp/clock.hpp>
#include <ros_gz_bridge/convert.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

class GimbalBridge : public rclcpp::Node {
public:
  GimbalBridge() : Node("gimbal_bridge") {
    this->declare_parameter<std::string>("namespace", "");
    this->get_parameter("namespace", model_name_);
    this->declare_parameter<std::string>("sensor_name");
    this->get_parameter("sensor_name", sensor_name_);
    ign_node_ptr_   = std::make_shared<ignition::transport::Node>();
    gimbal_roll_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        "/" + model_name_ + "/" + sensor_name_ + "/gimbal_cmd/0");
    gimbal_pitch_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        "/" + model_name_ + "/" + sensor_name_ + "/gimbal_cmd/1");
    gimbal_yaw_pub = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        "/" + model_name_ + "/" + sensor_name_ + "/gimbal_cmd/2");

    gimbal_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/" + model_name_ + "/" + sensor_name_ + "/gimbal_cmd", 10,
        std::bind(&GimbalBridge::gimbalCmdCallback, this, std::placeholders::_1));
  }

private:
  static std::string model_name_;
  static std::string sensor_name_;
  void gimbalCmdCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    ignition::msgs::Double gimbal_roll;
    gimbal_roll.set_data(msg->x);
    gimbal_roll_pub.Publish(gimbal_roll);
    ignition::msgs::Double gimbal_pitch;
    gimbal_pitch.set_data(msg->y);
    gimbal_pitch_pub.Publish(gimbal_pitch);
    ignition::msgs::Double gimbal_yaw;
    gimbal_yaw.set_data(msg->z);
    gimbal_yaw_pub.Publish(gimbal_yaw);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_cmd_sub_;
  std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
  ignition::transport::Node::Publisher gimbal_roll_pub;
  ignition::transport::Node::Publisher gimbal_pitch_pub;
  ignition::transport::Node::Publisher gimbal_yaw_pub;
  //   ignition::transport::Node::Publisher pub = node.Advertise<ignition::msgs::Vector3d>(
  //       "/" + model_name_ + "/" + sensor_name_ + "/gimbal_cmd");
};
std::string GimbalBridge::model_name_  = "";
std::string GimbalBridge::sensor_name_ = "";

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalBridge>());
  rclcpp::shutdown();
  return 0;
}