/*!*******************************************************************************************
 *  \file       set_model_pose_bridge.cpp
 *  \brief      gz bridge set model implementation file.
 *  \authors    Javier Melero Deza
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
#include <as2_msgs/srv/set_pose_with_id.hpp>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <ros_gz_bridge/convert.hpp>
#include <std_msgs/msg/float32.hpp>

class SetModelPoseBridge : public rclcpp::Node
{
public:
  SetModelPoseBridge()
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

private:
  std::shared_ptr<gz::transport::Node> gz_node_ptr_;
  std::string world_name_;
  std::string set_model_pose_service;
  static rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ps_sub_;
  static rclcpp::Service<as2_msgs::srv::SetPoseWithID>::SharedPtr ps_srv_sub_;

  void setModelPoseServiceCallback(
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
};

rclcpp::Service<as2_msgs::srv::SetPoseWithID>::SharedPtr SetModelPoseBridge::ps_srv_sub_;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<SetModelPoseBridge>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
