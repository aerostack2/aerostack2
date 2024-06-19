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


#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <memory>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

#ifndef AS2_GAZEBO_ASSETS__OBJECT_TF_BROADCASTER_HPP_
#define AS2_GAZEBO_ASSETS__OBJECT_TF_BROADCASTER_HPP_

class ObjectFramePublisher : public rclcpp::Node
{
public:
  ObjectFramePublisher();

private:
  static std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  static std::shared_ptr<std::string> world_frame_;
  static std::shared_ptr<std::string> model_name_;
  static std::shared_ptr<std::string> world_name;
  static bool use_sim_time_;

private:
  static void poseCallback(
    const gz::msgs::Pose_V & ign_msg,
    const gz::transport::MessageInfo & msg_info);

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription;
  std::shared_ptr<gz::transport::Node> ign_node_ptr_;
};

#endif  // AS2_GAZEBO_ASSETS__OBJECT_TF_BROADCASTER_HPP_
