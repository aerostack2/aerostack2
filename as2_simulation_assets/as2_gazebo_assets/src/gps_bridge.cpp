/*!*******************************************************************************************
 *  \file       gps_bridge.cpp
 *  \brief      Ignition bridge gps implementation file.
 *  \authors    Pedro Arias Pérez
 *              Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
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

#include <as2_core/names/topics.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ros_gz_bridge/convert.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSBridge : public rclcpp::Node {
public:
  GPSBridge() : Node("gps_bridge") {
    this->get_parameter("use_sim_time", use_sim_time_);

    this->declare_parameter<std::string>("world_name");
    this->get_parameter("world_name", world_name);

    this->declare_parameter<std::string>("name_space");
    this->get_parameter("name_space", name_space);

    this->declare_parameter<std::string>("sensor_name");
    this->get_parameter("sensor_name", sensor_name);

    this->declare_parameter<std::string>("link_name");
    this->get_parameter("link_name", link_name);

    this->declare_parameter<std::string>("sensor_type");
    this->get_parameter("sensor_type", sensor_type);

    // Initialize the ignition node
    ign_node_ptr_         = std::make_shared<ignition::transport::Node>();
    std::string gps_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                            sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                            "/navsat";
    ign_node_ptr_->Subscribe(gps_topic, this->ignitionGPSCallback);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        as2_names::topics::sensor_measurements::gps, as2_names::topics::sensor_measurements::qos);
  }

private:
  std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
  std::string world_name, name_space, sensor_name, link_name, sensor_type;
  static bool use_sim_time_;
  static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

private:
  static std::string replace_delimiter(const std::string &input,
                                       const std::string &old_delim,
                                       const std::string new_delim) {
    std::string output;
    output.reserve(input.size());

    std::size_t last_pos = 0;

    while (last_pos < input.size()) {
      std::size_t pos = input.find(old_delim, last_pos);
      output += input.substr(last_pos, pos - last_pos);
      if (pos != std::string::npos) {
        output += new_delim;
        pos += old_delim.size();
      }
      last_pos = pos;
    }
    return output;
  }

  static void ignitionGPSCallback(const ignition::msgs::NavSat &ign_msg,
                                  const ignition::transport::MessageInfo &msg_info) {
    sensor_msgs::msg::NavSatFix ros_msg;

    ros_gz_bridge::convert_gz_to_ros(ign_msg.header(), ros_msg.header);
    if (!use_sim_time_) {
      auto time            = rclcpp::Clock().now();
      ros_msg.header.stamp = time;
    }
    ros_msg.header.frame_id = GPSBridge::replace_delimiter(ign_msg.frame_id(), "::", "/");
    ros_msg.latitude        = ign_msg.latitude_deg();
    ros_msg.longitude       = ign_msg.longitude_deg();
    ros_msg.altitude        = ign_msg.altitude();

    // position_covariance is not supported in Ignition::Msgs::NavSat.
    ros_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    ros_msg.status.status            = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    gps_pub_->publish(ros_msg);
  };
};

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr GPSBridge::gps_pub_ = nullptr;
bool GPSBridge::use_sim_time_                                                 = false;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSBridge>());
  rclcpp::shutdown();
  return 0;
}
