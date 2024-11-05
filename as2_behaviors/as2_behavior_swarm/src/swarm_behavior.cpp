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
  broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  swarm_base_link_frame_id_ = as2::tf::generateTfName(this, "Swarm");
  RCLCPP_INFO(this->get_logger(), "%s", swarm_base_link_frame_id_.c_str());
  swarm_tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior constructor");
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "earth";
  transform.child_frame_id = swarm_base_link_frame_id_;
  broadcaster->sendTransform(transform);
}


void SwarmBehavior::init_drones(
  const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
  std::vector<std::string> drones_names_)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses = two_drones(goal->centroid_pose);

  for (auto drone_name : drones_names_) {
    std::shared_ptr<DroneSwarm> drone =
      std::make_shared<DroneSwarm>(this, drone_name, poses.front());
    drones_[drone_name] = drone;
    poses.erase(poses.begin());
    RCLCPP_INFO(
      this->get_logger(), "%s %f", drones_.at(drone_name)->drone_id_.c_str(), drones_.at(
        drone_name)->init_pose_.position.x);
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
  // TO DO
  return true;
}

bool SwarmBehavior::on_activate(
  std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Processing activate");
  as2_behavior_swarm_msgs::action::Swarm::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: Error processing goal");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Initializing drones");
  init_drones(goal, this->drones_names_);

  // Call Follow path behavior
  if (!this->follow_path_client_->wait_for_action_server(
      std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Follow Path Action server not available after waiting. Aborting navigation.");
    return false;
  }
  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header = goal->header;
  goal_msg.path = goal->path;
  goal_msg.yaw = goal->yaw_swarm;
  goal_msg.max_speed = goal->max_speed;
  // Call follow path
  auto send_goal_options = rclcpp_action::Client<as2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &SwarmBehavior::follow_path_response_cbk, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &SwarmBehavior::follow_path_feedback_cbk, this, std::placeholders::_1,
    std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&SwarmBehavior::follow_path_result_cbk, this, std::placeholders::_1);
  follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  return true;
}
as2_behavior::ExecutionStatus SwarmBehavior::on_run(
  const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
  std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Feedback> & feedback_msg,
  std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Result> & result_msg)
{
  goal_accepted_ = true;
  if (follow_path_rejected_ || swarm_aborted_) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (!follow_path_feedback_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for feedback from FollowPath behavior");
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  feedback_msg->actual_distance_to_next_waypoint =
    follow_path_feedback_->actual_distance_to_next_waypoint;
  return as2_behavior::ExecutionStatus::RUNNING;

  if (follow_path_succeeded_) {   // aqui tendre que añadir tambien la condicion de que los drones esten siguiendo correctamente a sus referencias
    result_msg->swarm_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
}

void SwarmBehavior::follow_path_response_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(),
      "FollowPath was rejected by behavior server. Aborting the swarm's movement.");
    follow_path_rejected_ = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "FollowPath accepted, flying to point.");
  }
}

void SwarmBehavior::follow_path_feedback_cbk(
  rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr goal_handle,
  const std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback)
{
  if (swarm_aborted_) {
    // cancel follow path too
    follow_path_client_->async_cancel_goal(goal_handle);
    return;
  }

  follow_path_feedback_ = feedback;
}

void SwarmBehavior::follow_path_result_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "FollowPath was aborted.Aborting the swarm's movement.");
      swarm_aborted_ = true;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "FollowPath was canceled. Cancelling the swarm's movement");
      swarm_aborted_ = true;
      return;
    default:
      RCLCPP_ERROR(
        this->get_logger(), "Unknown result code from FollowPath. Aborting the swarm's movement.");
      swarm_aborted_ = true;
      return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Follow Path succeeded. Goal point reached. Swarm's movement succeeded.");
  follow_path_succeeded_ = true;
}
