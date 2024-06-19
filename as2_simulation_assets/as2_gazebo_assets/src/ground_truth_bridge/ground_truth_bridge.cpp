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
 *  \brief      Ignition bridge ground truth implementation file.
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

#include "as2_gazebo_assets/ground_truth_bridge.hpp"


GroundTruthBridge::GroundTruthBridge()
: Node("ground_truth_bridge")
{
  this->declare_parameter<std::string>("name_space");
  this->get_parameter("name_space", model_name_);

  this->declare_parameter<std::string>("pose_frame_id");
  this->get_parameter("pose_frame_id", *pose_frame_id_);

  this->declare_parameter<std::string>("twist_frame_id");
  this->get_parameter("twist_frame_id", *twist_frame_id_);

  ps_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos);
  ts_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos);

  // Initialize the ignition node
  ign_node_ptr_ = std::make_shared<gz::transport::Node>();
  std::string ground_truth_topic = "/model/" + model_name_ + "/odometry";
  ign_node_ptr_->Subscribe(ground_truth_topic, this->ignitionGroundTruthCallback);
}

std::string GroundTruthBridge::replace_delimiter(
  const std::string & input,
  const std::string & old_delim,
  const std::string new_delim)
{
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

void GroundTruthBridge::ignitionGroundTruthCallback(
  const gz::msgs::Odometry & ign_msg,
  const gz::transport::MessageInfo & msg_info)
{
  geometry_msgs::msg::PoseStamped ps_msg;
  geometry_msgs::msg::TwistStamped ts_msg;

  ros_gz_bridge::convert_gz_to_ros(ign_msg.header(), ps_msg.header);
  ps_msg.header.frame_id = *pose_frame_id_;
  ps_msg.pose.position.x = ign_msg.pose().position().x();
  ps_msg.pose.position.y = ign_msg.pose().position().y();
  ps_msg.pose.position.z = ign_msg.pose().position().z();
  ps_msg.pose.orientation.w = ign_msg.pose().orientation().w();
  ps_msg.pose.orientation.x = ign_msg.pose().orientation().x();
  ps_msg.pose.orientation.y = ign_msg.pose().orientation().y();
  ps_msg.pose.orientation.z = ign_msg.pose().orientation().z();
  ros_gz_bridge::convert_gz_to_ros(ign_msg.header(), ts_msg.header);
  ts_msg.header.frame_id = *twist_frame_id_;
  ts_msg.twist.linear.x = ign_msg.twist().linear().x();
  ts_msg.twist.linear.y = ign_msg.twist().linear().y();
  ts_msg.twist.linear.z = ign_msg.twist().linear().z();
  ts_msg.twist.angular.x = ign_msg.twist().angular().x();
  ts_msg.twist.angular.y = ign_msg.twist().angular().y();
  ts_msg.twist.angular.z = ign_msg.twist().angular().z();

  ps_pub_->publish(ps_msg);
  ts_pub_->publish(ts_msg);
}


rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr GroundTruthBridge::ps_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr GroundTruthBridge::ts_pub_ = nullptr;
std::shared_ptr<std::string> GroundTruthBridge::pose_frame_id_ = std::make_shared<std::string>();
std::shared_ptr<std::string> GroundTruthBridge::twist_frame_id_ = std::make_shared<std::string>();
