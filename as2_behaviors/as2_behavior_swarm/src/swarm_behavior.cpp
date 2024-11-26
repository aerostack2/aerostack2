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
: as2_behavior::BehaviorServer<as2_behavior_swarm_msgs::action::Swarm>("SwarmBehavior")
{
// Get parameters
  try{
    this->declare_parameter<double>("initial_centroid.x");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid.x> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid.y");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid.y> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid.z");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid.z> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
  try{
    this->declare_parameter<std::vector<std::string>>("drone_namespaces");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <drone_namespaces> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
  initial_centroid_.header.frame_id = "earth";
  initial_centroid_.pose.position.x = this->get_parameter("initial_centroid.x").as_double();
  initial_centroid_.pose.position.y = this->get_parameter("initial_centroid.y").as_double();
  initial_centroid_.pose.position.z = this->get_parameter("initial_centroid.z").as_double(); 
  drones_names_ = this->get_parameter("drone_namespaces").as_string_array();
  
  new_centroid_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  // new_centroid_->pose.position.x = 6;
  // new_centroid_->pose.position.y = 0;
  // new_centroid_->pose.position.z = 1.5;

  swarm_tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
  broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  swarm_base_link_frame_id_ = as2::tf::generateTfName(this, "Swarm");
  transform_.header.stamp = this->get_clock()->now();
  transform_.header.frame_id = "earth";
  transform_.child_frame_id = swarm_base_link_frame_id_;
  transform_.transform.translation.x = initial_centroid_.pose.position.x;
  transform_.transform.translation.y = initial_centroid_.pose.position.y;
  transform_.transform.translation.z = initial_centroid_.pose.position.z;
  broadcaster->sendTransform(transform_);
  cbk_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ =
    this->create_wall_timer(
    std::chrono::microseconds(20),
    std::bind(&SwarmBehavior::timer_callback, this), cbk_group_);
   init_drones(this->initial_centroid_, this->drones_names_);
   timer2_=this->create_wall_timer(
    std::chrono::minutes(1),
    std::bind(&SwarmBehavior::timer_callback2, this), cbk_group_);
}

// Update Swarm Pose
void SwarmBehavior::update_pose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> new_centroid, std::shared_ptr<geometry_msgs::msg::PoseStamped> & update_centroid){

}
// Updates dinamic Swam TF
void SwarmBehavior::timer_callback()
{
  transform_.header.stamp = this->get_clock()->now();
  broadcaster->sendTransform(transform_);
}
void SwarmBehavior::timer_callback2()
{
  transform_.header.stamp = this->get_clock()->now();
  transform_.transform.translation.x = -4;
  broadcaster->sendTransform(transform_);
}

void SwarmBehavior::init_drones(
  geometry_msgs::msg::PoseStamped centroid,
  std::vector<std::string> drones_names_)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses = two_drones(initial_centroid_);

  for (auto drone_name : drones_names_) {
    std::shared_ptr<DroneSwarm> drone =
      std::make_shared<DroneSwarm>(this, drone_name, poses.front(), cbk_group_);
    drones_[drone_name] = drone;
    poses.erase(poses.begin());
    RCLCPP_INFO(
      this->get_logger(), "%s has the initial pose at x: %f, y: %f, z: %f", drones_.at(
        drone_name)->drone_id_.c_str(), drones_.at(
        drone_name)->init_pose_.position.x, drones_.at(
        drone_name)->init_pose_.position.y, drones_.at(
        drone_name)->init_pose_.position.z);
  }
}

bool SwarmBehavior::process_goal(
  std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal,
  as2_behavior_swarm_msgs::action::Swarm::Goal & new_goal)
{
  RCLCPP_INFO(this->get_logger(), "Processing goal");
  // Check if the path is in the earth frame, if not convert it
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




bool SwarmBehavior::on_activate(
  std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal)
{
  as2_behavior_swarm_msgs::action::Swarm::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: Error processing goal");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  for (auto drone : drones_) {
    goal_future_handles_.push_back(drone.second->own_init());
  }

  
  return true;
}
as2_behavior::ExecutionStatus SwarmBehavior::monitoring(
  const std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
  goal_future_handles)
{
  as2_behavior::ExecutionStatus local_status;
  for (auto goal_handle : goal_future_handles) {
    switch (goal_handle->get_status()) {
      case rclcpp_action::GoalStatus::STATUS_EXECUTING:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        // printf("Running exe\n");
        break;
      case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        // printf("Running suc\n");
        break;
      case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        // printf("Running acep\n");
        break;
      case rclcpp_action::GoalStatus::STATUS_ABORTED:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        // printf("Not running abort\n");
        break;
      case rclcpp_action::GoalStatus::STATUS_CANCELED:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        // printf("Not running cance\n");
        break;
      default:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        // printf("Not running def\n");
        break;
    }
  }
  return local_status;
}
as2_behavior::ExecutionStatus SwarmBehavior::on_run(
  const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
  std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Feedback> & feedback_msg,
  std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Result> & result_msg)
{
  as2_behavior::ExecutionStatus local_status = monitoring(goal_future_handles_);
  if (local_status == as2_behavior::ExecutionStatus::FAILURE) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  return as2_behavior::ExecutionStatus::RUNNING;

}

bool SwarmBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Stopped");
  transform_.transform.translation.z=0; // Land the swarm
  return true;
}

bool SwarmBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Paused");
  // seguir mandando la transformada de ese momento y para de enviar waypoints.

  return true;
}

bool SwarmBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Resumed");
  // Return de tf to the original position
  transform_.transform.translation.x = initial_centroid_.pose.position.x;
  transform_.transform.translation.y = initial_centroid_.pose.position.y;
  transform_.transform.translation.z = initial_centroid_.pose.position.z;
  return true;
}

void SwarmBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Finished");
  return;
}