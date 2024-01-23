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
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <as2_core/names/topics.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ros_gz_bridge/convert.hpp>
#include <std_msgs/msg/float32.hpp>

class AzimuthBridge : public rclcpp::Node {
public:
  AzimuthBridge() : Node("azimuth_bridge") {
    this->declare_parameter<std::string>("name_space");
    this->get_parameter("name_space", model_name_);

    ps_pub_ = this->create_publisher<std_msgs::msg::Float32>("gps/azimuth",
                                                             as2_names::topics::ground_truth::qos);

    // Initialize the ignition node
    ign_node_ptr_                  = std::make_shared<ignition::transport::Node>();
    std::string ground_truth_topic = "/model/gps/odometry";
    ign_node_ptr_->Subscribe(ground_truth_topic, this->ignitionGroundTruthCallback);
  }

private:
  std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
  std::string model_name_;
  static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ps_pub_;
  struct Quaternion {
    float w, x, y, z;
  };

private:
  static void ignitionGroundTruthCallback(const ignition::msgs::Odometry &ign_msg,
                                          const ignition::transport::MessageInfo &msg_info) {
    std_msgs::msg::Float32 az_msg;
    Quaternion q;
    q.x = ign_msg.pose().orientation().x();
    q.y = ign_msg.pose().orientation().y();
    q.z = ign_msg.pose().orientation().z();
    q.w = ign_msg.pose().orientation().w();

    az_msg.data = toAzimuth(toEulerYaw(q));

    ps_pub_->publish(az_msg);
  }

  static float toEulerYaw(Quaternion q) {
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    float yaw       = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
  }

  static float toAzimuth(float yaw) {
    float deg = yaw * 360 / (M_PI * 2);  // [-180, 180]
    deg       = -deg + 90.0f;
    if (deg < 0) {
      deg = 360 + deg;
    }

    return deg;
  }
};

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr AzimuthBridge::ps_pub_ = nullptr;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AzimuthBridge>());
  rclcpp::shutdown();
  return 0;
}