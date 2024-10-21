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


#include "swarm_behavior.hpp"


SwarmBehavior::SwarmBehavior()
: as2_behavior::BehaviorServer<as2_behavior_swarm_msgs::action::Swarm>("Swarm")
{
  swarm_base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  swarm_tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
  timer_ = this->create_timer(
    std::chrono::milliseconds(20),
    std::bind(&SwarmBehavior::swarmCallback, this));
}

void SwarmBehavior::swarmCallback()
{
  this->initDrones(this->drones_names_);
}

void SwarmBehavior::initDrones(std::vector<std::string> drones_names_)
{
  RCLCPP_INFO(this->get_logger(), " swarm %s", swarm_base_link_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "Initializing drones");
  for (auto drone_name : drones_names_) {
    std::shared_ptr<DroneSwarm> drone = std::make_shared<DroneSwarm>(this);
    drones_[drone_name] = drone;
    drones_[drone_name]->drone_name_ = drone_name;
    RCLCPP_INFO(
      this->get_logger(), "%s %f", drones_.at(drone_name)->drone_name_.c_str(), drones_.at(
        drone_name)->drone_pose_.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "%s", drones_.at(drone_name)->base_link_frame_id_.c_str());
  }
}
