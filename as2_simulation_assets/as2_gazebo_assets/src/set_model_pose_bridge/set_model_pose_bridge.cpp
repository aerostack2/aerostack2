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
 *  \file       set_model_pose_bridge.cpp
 *  \brief      Set model pose bridge implementation file.
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

#include "as2_gazebo_assets/set_model_pose_bridge.hpp"

SetModelPoseBridge::SetModelPoseBridge()
: Node("set_model_pose_bridge")
{
  this->declare_parameter<std::string>("world_name");
  this->get_parameter("world_name", world_name_);

  ps_srv_sub_ = this->create_service<as2_msgs::srv::SetPoseWithID>(
    "/world/" + world_name_ + "/set_pose",
    std::bind(
      &SetModelPoseBridge::setModelPoseServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Initialize the gz node
  gz_node_ptr_ = std::make_shared<gz::transport::Node>();
  std::string set_model_pose_service = "/world/" + world_name_ + "/set_pose";
}

void SetModelPoseBridge::setModelPoseServiceCallback(
  const as2_msgs::srv::SetPoseWithID::Request::SharedPtr request,
  as2_msgs::srv::SetPoseWithID::Response::SharedPtr result)
{
  gz::msgs::Pose gz_msg = gz::msgs::Pose();

  gz_msg.set_name(request->pose.id);
  gz_msg.mutable_position()->set_x(request->pose.pose.position.x);
  gz_msg.mutable_position()->set_y(request->pose.pose.position.y);
  gz_msg.mutable_position()->set_z(request->pose.pose.position.z);
  gz_msg.mutable_orientation()->set_x(request->pose.pose.orientation.x);
  gz_msg.mutable_orientation()->set_y(request->pose.pose.orientation.y);
  gz_msg.mutable_orientation()->set_z(request->pose.pose.orientation.z);
  gz_msg.mutable_orientation()->set_w(request->pose.pose.orientation.w);

  // gz_msg_req.set_request("/world/grass/set_pose");
  gz::msgs::Boolean response;
  bool _result;

  bool gz_result = gz_node_ptr_->Request(
    "/world/" + world_name_ + "/set_pose", gz_msg, 1000,
    response, _result);

  if (!gz_result) {
    RCLCPP_WARN(this->get_logger(), "Failed to set model pose");
  } else {
    RCLCPP_INFO(this->get_logger(), "Model pose set");
  }
  result->success = gz_result;
}
