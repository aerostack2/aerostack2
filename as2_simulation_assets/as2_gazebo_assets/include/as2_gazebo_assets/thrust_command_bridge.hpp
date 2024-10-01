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


#include <iostream>
#include <memory>
#include <string>

#include <as2_core/names/topics.hpp>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <ros_gz_bridge/convert.hpp>
#include "rclcpp/rclcpp.hpp"
#include "as2_msgs/msg/thrust.hpp"

#ifndef AS2_GAZEBO_ASSETS__THRUST_COMMAND_BRIDGE_HPP_
#define AS2_GAZEBO_ASSETS__THRUST_COMMAND_BRIDGE_HPP_

class ThrustCommandBridge : public rclcpp::Node
{
public:
  ThrustCommandBridge();

private:
  std::shared_ptr<gz::transport::Node> gz_node_ptr_;
  gz::transport::Node::Publisher gz_node_pub;
  std::string world_name, name_space, sensor_name, link_name, sensor_type;
  static bool use_sim_time_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr subscription;

private:
  static void thrustCallback(
    const as2_msgs::Thrust & ros_msg);
};

#endif  // AS2_GAZEBO_ASSETS__THRUST_COMMAND_BRIDGE_HPP_
