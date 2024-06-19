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
 *  \file       object_tf_broadcaster.cpp
 *  \brief      Ignition bridge tf broadcaster implementation file.
 *  \authors    Javier Melero Deza
 *              Pedro Arias Pérez
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

#include "as2_gazebo_assets/object_tf_broadcaster.hpp"


ObjectFramePublisher::ObjectFramePublisher()
: Node("frame_publisher")
{
  this->declare_parameter<std::string>("world_frame", "");
  this->get_parameter("world_frame", *world_frame_);
  this->declare_parameter<std::string>("namespace", "");
  this->get_parameter("namespace", *model_name_);
  this->declare_parameter<std::string>("world_name", "");
  this->get_parameter("world_name", *world_name);
  this->get_parameter("use_sim_time", use_sim_time_);
  this->tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  ign_node_ptr_ = std::make_shared<gz::transport::Node>();
  std::string pose_topic = "/model/" + *model_name_ + "/pose";
  ign_node_ptr_->Subscribe(pose_topic, this->poseCallback);
}

void ObjectFramePublisher::poseCallback(
  const gz::msgs::Pose_V & ign_msg,
  const gz::transport::MessageInfo & msg_info)
{
  geometry_msgs::msg::TransformStamped transform;
  for (const gz::msgs::Pose & pose : ign_msg.pose()) {
    ros_gz_bridge::convert_gz_to_ros(pose, transform);
    if (transform.header.frame_id == *world_name) {
      transform.header.frame_id = *world_frame_;
    }
    if (!use_sim_time_) {
      auto time = rclcpp::Clock().now();
      transform.header.stamp = time;
    }
    tfBroadcaster->sendTransform(transform);
  }
}

bool ObjectFramePublisher::use_sim_time_ = false;
std::unique_ptr<tf2_ros::TransformBroadcaster> ObjectFramePublisher::tfBroadcaster = NULL;

std::shared_ptr<std::string> ObjectFramePublisher::world_frame_ = std::make_shared<std::string>();
std::shared_ptr<std::string> ObjectFramePublisher::model_name_ = std::make_shared<std::string>();
std::shared_ptr<std::string> ObjectFramePublisher::world_name = std::make_shared<std::string>();
