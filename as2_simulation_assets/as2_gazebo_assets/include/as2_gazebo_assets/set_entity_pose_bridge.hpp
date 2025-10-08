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
 *  \file       set_entity_pose_bridge.hpp
 *  \brief      Set entity pose bridge header file.
 *  \authors    Javier Melero Deza
 *              Pedro Arias Pérez
 ********************************************************************************/

#include <math.h>

#include <memory>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ros_gz_bridge/convert.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#ifndef AS2_GAZEBO_ASSETS__SET_ENTITY_POSE_BRIDGE_HPP_
#define AS2_GAZEBO_ASSETS__SET_ENTITY_POSE_BRIDGE_HPP_

class SetEntityPoseBridge : public rclcpp::Node
{
public:
  SetEntityPoseBridge();

private:
  std::shared_ptr<gz::transport::Node> gz_node_ptr_;
  std::string world_name_;
  std::string set_entity_pose_service;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ps_sub_;
  rclcpp::Service<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr ps_srv_sub_;

  void setEntityPoseServiceCallback(
    const ros_gz_interfaces::srv::SetEntityPose::Request::SharedPtr request,
    ros_gz_interfaces::srv::SetEntityPose::Response::SharedPtr result);
};

#endif  // AS2_GAZEBO_ASSETS__SET_ENTITY_POSE_BRIDGE_HPP_
