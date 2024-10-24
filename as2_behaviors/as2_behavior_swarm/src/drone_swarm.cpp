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

#include "drone_swarm.hpp"

DroneSwarm::DroneSwarm(as2::Node * node_ptr)
: node_ptr_(node_ptr)
{
  drone_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
    std::bind(&DroneSwarm::drone_pose_callback, this, std::placeholders::_1));

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  base_link_frame_id_ = as2::tf::generateTfName(node_ptr, "base_link");

  platform_info_sub_ = node_ptr->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&DroneSwarm::platform_info_callback, this, std::placeholders::_1));
}

void DroneSwarm::ownInit()
{
  position_motion_handler_ =
    std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
}

void DroneSwarm::drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _pose_msg)
{
  // self_localization is always on frame earth
  drone_pose_ = *_pose_msg;
}
void DroneSwarm::platform_info_callback(
  const as2_msgs::msg::PlatformInfo::SharedPtr _platform_info_msg)
{
  platform_info_ = *_platform_info_msg;
}
