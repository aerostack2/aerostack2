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
 *  \file       acro_bridge.cpp
 *  \brief      Gazebo bridge ACRO implementation file.
 *  \authors    Francisco José Anguita Chamorro
 ********************************************************************************/

#include "as2_gazebo_assets/acro_bridge.hpp"

AcroBridge::AcroBridge()
: Node("acro_bridge")
{
  this->get_parameter("use_sim_time", use_sim_time_);

  this->declare_parameter<std::string>("name_space");
  this->get_parameter("name_space", name_space);

  // Initialize Gazebo node
  gz_node_ptr_ = std::make_shared<gz::transport::Node>();

  std::string acro_sub_topic = "/gz/" + name_space + "/acro";

  acro_sub_ = this->create_subscription<as2_msgs::msg::Acro>(
    acro_sub_topic, 10, std::bind(&AcroBridge::acroCallback, this, std::placeholders::_1));

  std::string acro_pub_topic = "/model/" + name_space + "/acro";
  acro_pub_ = std::make_shared<gz::transport::Node::Publisher>();
  *acro_pub_ = gz_node_ptr_->Advertise<gz::msgs::Float_V>(acro_pub_topic);
}

void AcroBridge::acroCallback(const as2_msgs::msg::Acro & acro_msg)
{
  gz::msgs::Float_V gz_acro_msg;

  gz_acro_msg.mutable_header()->mutable_stamp()->set_sec(acro_msg.header.stamp.sec);
  gz_acro_msg.mutable_header()->mutable_stamp()->set_nsec(acro_msg.header.stamp.nanosec);

  if (!use_sim_time_) {
    auto time = rclcpp::Clock().now();
    // ros_gz_bridge::convert_ros_to_gz(time, *(gz_acro_msg.mutable_header()->mutable_stamp()));
    gz_acro_msg.mutable_header()->mutable_stamp()->set_sec(time.seconds());
    gz_acro_msg.mutable_header()->mutable_stamp()->set_nsec(time.nanoseconds());
  }

  // gz_acro_msg.set_allocated_header(headerPtr);
  gz_acro_msg.add_data(acro_msg.angular_rates.x);
  gz_acro_msg.add_data(acro_msg.angular_rates.y);
  gz_acro_msg.add_data(acro_msg.angular_rates.z);
  gz_acro_msg.add_data(acro_msg.thrust.z);

  acro_pub_->Publish(gz_acro_msg);
}

bool AcroBridge::use_sim_time_ = false;
std::shared_ptr<gz::transport::Node::Publisher> AcroBridge::acro_pub_ = nullptr;
