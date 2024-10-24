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
  // RCLCPP_INFO(this->get_logger(), " swarm %s", swarm_base_link_frame_id_.c_str());
  // RCLCPP_INFO(this->get_logger(), "Initializing drones");
  for (auto drone_name : drones_names_) {
    std::shared_ptr<DroneSwarm> drone = std::make_shared<DroneSwarm>(this);
    drones_[drone_name] = drone;
    drones_[drone_name]->drone_name_ = drone_name;
    // RCLCPP_INFO(
    // this->get_logger(), "%s %f", drones_.at(drone_name)->drone_name_.c_str(), drones_.at(
    //   drone_name)->drone_pose_.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "%s", drones_.at(drone_name)->base_link_frame_id_.c_str());
  }
}

bool SwarmBehavior::process_goal(
  std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal,
  as2_behavior_swarm_msgs::action::Swarm::Goal & new_goal)
{
  RCLCPP_INFO(this->get_logger(), "Processing goal");

  // Check if the path is in the earth frame if not convert it
  if (goal->header.frame_id == "") {
    RCLCPP_ERROR(this->get_logger(), "Path frame_id is empty");
    return false;
  }
  if (goal->path.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Path is empty");
    return false;
  }
  if (goal->header.frame_id != "earth") {
    std::vector<as2_msgs::msg::PoseWithID> path_converted;
    path_converted.reserve(goal->path.size());

    geometry_msgs::msg::PoseStamped pose_msg;

    for (as2_msgs::msg::PoseWithID waypoint : goal->path) {
      pose_msg.pose = waypoint.pose;
      pose_msg.header = goal->header;
      if (!swarm_tf_handler_->tryConvert(pose_msg, "earth", tf_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: can not get waypoint in earth frame");
        return false;
      }
      waypoint.pose = pose_msg.pose;
      path_converted.push_back(waypoint);
    }
    new_goal.header.frame_id = "earth";
    new_goal.path = path_converted;
  }
  // Check if the centroid is in the earth frame if not convert it
  if (goal->centroid_pose.header.frame_id != "earth") {
    RCLCPP_ERROR(this->get_logger(), "Centroid pose frame_id is empty");
    return false;
  }
  if (!swarm_tf_handler_->tryConvert(new_goal.centroid_pose, "earth", tf_timeout)) {
    RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: can not get centroid position in earth frame");
    return false;
  }
  // Check if the swarm_yaw is in the earth frame if not convert it
  geometry_msgs::msg::QuaternionStamped q;
  q.header = goal->header;
  as2::frame::eulerToQuaternion(0.0f, 0.0f, new_goal.yaw_swarm.angle, q.quaternion);

  if (!swarm_tf_handler_->tryConvert(q, "earth", tf_timeout)) {
    RCLCPP_ERROR(this->get_logger(), "GoToBehavior: can not get target orientation in earth frame");
    return false;
  }

  new_goal.yaw_swarm.angle = as2::frame::getYawFromQuaternion(q.quaternion);
  return true;
}

bool SwarmBehavior::swarm_formation(
  const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
  std::unordered_map<std::string, std::shared_ptr<DroneSwarm>> & drones)
{
  RCLCPP_INFO(this->get_logger(), "Swarm formation");
  std::size_t n = drones.size();  // cuantos drones forman el enjambre

  // Esto es una chapuza para ver como mover el dron
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(n);
  for (auto drone : drones) {
    poses.push_back(drone.second->drone_pose_);
  }
  poses.at(0).pose.position.x = goal->centroid_pose.pose.position.x - 1.0;
  poses.at(1).pose.position.y = goal->centroid_pose.pose.position.x + 1.0;

  // if (!position_motion_handler_->sendPositionCommandWithYawAngle(
  //     poses.at(0)))
  // {
  //   RCLCPP_ERROR(node_ptr_->get_logger(), "GOTO PLUGIN: Error sending position command");
  //   result_.go_to_success = false;
  //   return as2_behavior::ExecutionStatus::FAILURE;
  // }

  return true;
}
bool SwarmBehavior::on_activate(
  std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Processing activate");
  as2_behavior_swarm_msgs::action::Swarm::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }

  // To DO
  // bool p = swarm_formation(new_goal);
  return true;
}
