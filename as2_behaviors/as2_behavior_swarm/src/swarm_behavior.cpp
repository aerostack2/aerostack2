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

void generateDynamicPoint(
  const as2_msgs::msg::PoseWithID & msg,
  dynamic_traj_generator::DynamicWaypoint & dynamic_point)
{
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

SwarmBehavior::SwarmBehavior()
: as2_behavior::BehaviorServer<as2_behavior_swarm_msgs::action::Swarm>("SwarmBehavior")
{
// Get parameters
  try{
    this->declare_parameter<double>("initial_centroid_position.x");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_position.x> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid_position.y");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_position.y> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid_position.z");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_position.z> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
  try{
    this->declare_parameter<double>("initial_centroid_orientation.x");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_orientation.x> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid_orientation.y");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_orientation.y> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid_orientation.z");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_orientation.z> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
    try{
    this->declare_parameter<double>("initial_centroid_orientation.w");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <initial_centroid_orientation.w> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
  try{
    this->declare_parameter<std::vector<std::string>>("drone_namespaces");
  }catch(const rclcpp::ParameterTypeException & e){
    RCLCPP_FATAL(this->get_logger(), "Launch argument <drone_namespaces> not defined or malformed: %s",e.what());
    this->~SwarmBehavior();
  }
  service_start_ = this->create_service<as2_behavior_swarm_msgs::srv::StartSwarm>(
      "start_swarm", std::bind(&SwarmBehavior::startBehavior, this, _1, _2));
  initial_centroid_.header.frame_id = "earth";
  initial_centroid_.pose.position.x = this->get_parameter("initial_centroid_position.x").as_double();
  initial_centroid_.pose.position.y = this->get_parameter("initial_centroid_position.y").as_double();
  initial_centroid_.pose.position.z = this->get_parameter("initial_centroid_position.z").as_double(); 
  initial_centroid_.pose.orientation.x = this->get_parameter("initial_centroid_orientation.x").as_double();
  initial_centroid_.pose.orientation.y = this->get_parameter("initial_centroid_orientation.y").as_double();
  initial_centroid_.pose.orientation.w = this->get_parameter("initial_centroid_orientation.w").as_double();
  initial_centroid_.pose.orientation.z = this->get_parameter("initial_centroid_orientation.z").as_double();
  drones_names_ = this->get_parameter("drone_namespaces").as_string_array();

  swarm_tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);
  broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  transform_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
  swarm_base_link_frame_id_ = as2::tf::generateTfName(this, "Swarm");
  transform_->header.stamp = this->get_clock()->now();
  transform_->header.frame_id = "earth";
  transform_->child_frame_id = swarm_base_link_frame_id_;
  transform_->transform.translation.x = initial_centroid_.pose.position.x;
  transform_->transform.translation.y = initial_centroid_.pose.position.y;
  transform_->transform.translation.z = initial_centroid_.pose.position.z;
  broadcaster->sendTransform(*( transform_));
  cbk_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ =
    this->create_wall_timer(
    std::chrono::microseconds(20),
    std::bind(&SwarmBehavior::timerCallback, this), cbk_group_);
  // timer2_ = this->create_wall_timer(
  //   std::chrono::seconds(10),
  //   std::bind(&SwarmBehavior::timerCallback2, this));
   initDrones(this->initial_centroid_, this->drones_names_);
  trajectory_generator_ = std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  // trajectory_generator_->updateVehiclePosition(
  //     Eigen::Vector3d(
  //       initial_centroid_.pose.position.x,initial_centroid_.pose.position.y,initial_centroid_.pose.position.z));
  // startBehavior();
}


// Updates dinamic Swam TF
void SwarmBehavior::timerCallback()
{
  transform_->header.stamp = this->get_clock()->now();
  broadcaster->sendTransform(*(transform_));

}
// void SwarmBehavior::timerCallback2()
// {
     
// }

void SwarmBehavior::initDrones(
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
      this->get_logger(), "%s has the initial pose at x: %f, y: %f, z: %f relative to the centroid", drones_.at(
        drone_name)->drone_id_.c_str(), drones_.at(
        drone_name)->init_pose_.position.x, drones_.at(
        drone_name)->init_pose_.position.y, drones_.at(
        drone_name)->init_pose_.position.z);
    
  } 
}

// check if the drones are in the correct position to start the trayectory 
void SwarmBehavior::startBehavior(const std::shared_ptr<as2_behavior_swarm_msgs::srv::StartSwarm::Request> request,
    const std::shared_ptr<as2_behavior_swarm_msgs::srv::StartSwarm::Response> response)
{
  std::string start;
  start = request->start;
  
 if(start.compare("start")==0){
   for (auto drone : drones_) {
    goal_future_handles_.push_back(drone.second->ownInit());
   }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  bool flag = false;
while (!flag) {
    flag = true; 

    for (auto& drones : drones_) {
        if (!drones.second->checkPosition()) {
            flag = false; 
            break;        
        }
    }if (flag) {
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "All drones are in position");
      this->start_behavior = true;
        break; 
    }
}}}

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
  if(!this->start_behavior){
    RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: Drones are not in position");
    return false;
  }
  if (!process_goal(goal, new_goal)) {
    RCLCPP_ERROR(this->get_logger(), "SwarmBehavior: Error processing goal");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Check speed
  if (goal->max_speed < 0) {
    RCLCPP_ERROR(this->get_logger(), "Goal max speed is negative");
    return false;
  }
  trajectory_generator_->updateVehiclePosition(
      Eigen::Vector3d(
        initial_centroid_.pose.position.x,initial_centroid_.pose.position.y,initial_centroid_.pose.position.z));
  current_yaw_ = as2::frame::getYawFromQuaternion(initial_centroid_.pose.orientation);
  
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(goal->path.size() + 1);
  trajectory_generator_->setSpeed(goal->max_speed);
  // For each waypoint in the path, generate a dynamic waypoint and store it in the vector waypoints_to_set
  for (auto waypoint: goal->path){
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }
  // When all the waypoints are stored, set them in the trajectory generator
  trajectory_generator_->setWaypoints(waypoints_to_set);
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
  // send the current time to the trajectory generator
  if (first_run_) {
    if(!evaluateTrajectory(trajectory_generator_->getMinTime())){
      return as2_behavior::ExecutionStatus::FAILURE;
    }
    time_zero_ = this->now();
    eval_time_ = rclcpp::Duration(0, 0);
    first_run_ = false;
  } else {
    eval_time_ = this->now() - time_zero_;
    if(!evaluateTrajectory(eval_time_.seconds())){
      return as2_behavior::ExecutionStatus::FAILURE;
    }
  }
    transform_->transform.translation.x = trajectory_command_.setpoints.back().position.x;
    transform_->transform.translation.y = trajectory_command_.setpoints.back().position.y;
    transform_->transform.translation.z = trajectory_command_.setpoints.back().position.z;
    geometry_msgs::msg::Quaternion q;
    as2::frame::eulerToQuaternion(0.0f, 0.0f, trajectory_command_.setpoints.back().yaw_angle, q);
    transform_->transform.rotation.w = q.w;
    transform_->transform.rotation.x = q.x;
    transform_->transform.rotation.y = q.y;
    transform_->transform.rotation.z = q.z;

  
  return as2_behavior::ExecutionStatus::RUNNING;

}

bool SwarmBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Stopped");
  // transform_->transform.translation.z=0; // Land the swarm
  //hacer un hovver
  this->start_behavior = false;
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
  transform_->transform.translation.x = initial_centroid_.pose.position.x;
  transform_->transform.translation.y = initial_centroid_.pose.position.y;
  transform_->transform.translation.z = initial_centroid_.pose.position.z;
  this->start_behavior = false;
  return true;
}

void SwarmBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(this->get_logger(), "SwarmBehavior Finished");
  this->start_behavior = false;

    // Reset the trajectory generator
  trajectory_generator_ =
    std::make_shared<dynamic_traj_generator::DynamicTrajectory>();

  return;
}

bool SwarmBehavior::evaluateTrajectory(double eval_time){
   dynamic_traj_generator::References traj_command;
   as2_msgs::msg::TrajectoryPoint setpoint;
  // Check the time is in the range of the trajectory
  if (eval_time <= trajectory_generator_->getMinTime()) {
    eval_time = trajectory_generator_->getMinTime();
  } else if (eval_time >= trajectory_generator_->getMaxTime()) {
    eval_time = trajectory_generator_->getMaxTime();
  }
  
  bool succes_eval =
    trajectory_generator_->evaluateTrajectory(eval_time, traj_command);

  setpoint.position.x = traj_command.position.x();
  setpoint.position.y = traj_command.position.y();
  setpoint.position.z = traj_command.position.z();
  setpoint.twist.x = traj_command.velocity.x();
  setpoint.twist.y = traj_command.velocity.y();
  setpoint.twist.z = traj_command.velocity.z();
  setpoint.acceleration.x = traj_command.acceleration.x();
  setpoint.acceleration.y = traj_command.acceleration.y();
  setpoint.acceleration.z = traj_command.acceleration.z();
  setpoint.yaw_angle = computeYawAnglePathFacing(
        traj_command.velocity.x(),
        traj_command.velocity.y());
  trajectory_command_.setpoints.push_back(setpoint);
  return succes_eval;
}

// Swarm Path Facing
double SwarmBehavior::computeYawAnglePathFacing(
  double vx, double vy)
{
  if (sqrt(vx * vx + vy * vy) > 0.1) {
    return as2::frame::getVector2DAngle(vx, vy);
  }
  return current_yaw_;
}
