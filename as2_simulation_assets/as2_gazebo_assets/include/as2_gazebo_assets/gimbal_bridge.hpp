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


#include <math.h>

#include <iostream>
#include <memory>
#include <string>

#include <as2_core/names/topics.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <rclcpp/clock.hpp>
#include <ros_gz_bridge/convert.hpp>
#include "as2_msgs/msg/gimbal_control.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef AS2_GAZEBO_ASSETS__GIMBAL_BRIDGE_HPP_
#define AS2_GAZEBO_ASSETS__GIMBAL_BRIDGE_HPP_

class GimbalBridge : public rclcpp::Node
{
public:
  GimbalBridge();

private:
  static std::shared_ptr<std::string> model_name_;
  static std::shared_ptr<std::string> gimbal_name_;
  static std::shared_ptr<std::string> sensor_name_;
  static std::shared_ptr<std::string> control_mode_;
  static std::shared_ptr<std::string> world_name_;

  void gimbalCmdCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg);

  rclcpp::Subscription<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_cmd_sub_;
  static rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr gimbal_attitude_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    gimbal_angular_velocity_pub_;

  static std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<gz::transport::Node> gz_node_ptr_;
  gz::transport::Node::Publisher gimbal_roll_pub;
  gz::transport::Node::Publisher gimbal_pitch_pub;
  gz::transport::Node::Publisher gimbal_yaw_pub;

  static void gzJointStateCallback(
    const gz::msgs::Model & gz_msg,
    const gz::transport::MessageInfo & msg_info);
};

#endif  // AS2_GAZEBO_ASSETS__GIMBAL_BRIDGE_HPP_
