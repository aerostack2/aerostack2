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

/*!******************************************************************************
 *  \file       swarm_flocking_behavior.hpp
 *  \brief      Aerostack2 swarm_behavior file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/


 #include <swarm_flocking_behavior.hpp>


SwarmFlockingBehavior::SwarmFlockingBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::SwarmFlocking>("SwarmFlockingBehavior")
{
  modify_srv_ = this->create_service<as2_msgs::action::SwarmFlocking::Impl::SendGoalService>(
    "swarm_modify_srv",
    std::bind(&SwarmFlockingBehavior::modifySwarmSrv, this, _1, _2));


  cbk_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  tfstatic_swarm_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  transform_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
  swarm_base_link_frame_id_ = as2::tf::generateTfName(this, "Swarm");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool SwarmFlockingBehavior::setUpVirtualCentroid(
  const geometry_msgs::msg::PoseStamped & virtual_centroid)
{
  if (virtual_centroid.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Virtual centroid frame_id is empty");
    return false;
  } else {
    transform_->header.stamp = this->get_clock()->now();
    transform_->header.frame_id = virtual_centroid.header.frame_id;
    transform_->child_frame_id = swarm_base_link_frame_id_;
    transform_->transform.translation.x = virtual_centroid.pose.position.x;
    transform_->transform.translation.y = virtual_centroid.pose.position.y;
    transform_->transform.translation.z = virtual_centroid.pose.position.z;
    transform_->transform.rotation.x = virtual_centroid.pose.orientation.x;
    transform_->transform.rotation.y = virtual_centroid.pose.orientation.y;
    transform_->transform.rotation.z = virtual_centroid.pose.orientation.z;
    transform_->transform.rotation.w = virtual_centroid.pose.orientation.w;
    tfstatic_swarm_broadcaster_->sendTransform(*( transform_));
    return true;
  }
}
void SwarmFlockingBehavior::modifySwarmSrv(
  const std::shared_ptr<as2_msgs::action::SwarmFlocking::Impl::SendGoalService::Request> request,
  std::shared_ptr<as2_msgs::action::SwarmFlocking::Impl::SendGoalService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Modify service called");
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior: Modifying flocking");
  goal_ = request->goal;
  goal_future_handles_.clear();
  for (auto drone : drones_) {
    if (!drone.second->stopFollowReference()) {
      RCLCPP_ERROR(
        this->get_logger(), "Drone %s could not be detached from Swarm tf",
        drone.second->drone_id_.c_str());
      response->accepted = false;
    }
  }
  if (!setUpVirtualCentroid(goal_.virtual_centroid)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SwarmFlockingBehavior: Error setting up the virtual centroid");
    response->accepted = false;
  } else {
    for (auto id : goal_.swarm_formation) {
      if (drones_.find(id.id) == drones_.end()) {
        drones_.at(id.id)->updateStaticTf(id.pose);
        goal_future_handles_.push_back(drones_.at(id.id)->initFollowReference());
      } else {
        std::shared_ptr<DroneSwarm> drone =
          std::make_shared<DroneSwarm>(this, id.id, id.pose, cbk_group_);
        drones_[id.id] = drone;
        goal_future_handles_.push_back(drones_.at(id.id)->initFollowReference());
      }
    }
    std::vector<std::string> drones_names_erase;
    std::vector<std::string> drones_names_aux;
    for (auto drone : drones_) {
      drones_names_aux.push_back(drone.first);
    }
    std::sort(drones_names_aux.begin(), drones_names_aux.end());
    std::sort(goal_.drones_namespace.begin(), goal_.drones_namespace.end());
    std::set_difference(
      drones_names_aux.begin(), drones_names_aux.end(),
      goal_.drones_namespace.begin(), goal_.drones_namespace.end(),
      std::back_inserter(drones_names_erase), [](std::string a, std::string b) {return a < b;});
    for (auto drones : drones_names_erase) {
      drones_.erase(drones);
    }
  }
}

void SwarmFlockingBehavior::dynamicSwarmFormationCallback(
  as2_msgs::msg::PoseWithIDArray new_formation)
{
  for (auto new_pose : new_formation.poses) {
    if (drones_.find(new_pose.id) != drones_.end()) {
      drones_.at(new_pose.id)->updateStaticTf(new_pose.pose);
      for (auto drone : drones_) {
        goal_future_handles_.push_back(drone.second->initFollowReference());
      }
    }
  }
}

bool SwarmFlockingBehavior::setUpDronesFormation(
  geometry_msgs::msg::PoseStamped centroid,
  std::vector<std::string> drones_names, std::vector<as2_msgs::msg::PoseWithID> formation)
{
  for (auto drone_name : drones_names) {
    for (auto id : formation) {
      if (id.id == drone_name) {
        std::shared_ptr<DroneSwarm> drone =
          std::make_shared<DroneSwarm>(this, drone_name, id.pose, cbk_group_);
        drones_[drone_name] = drone;
      }
    }
  }
  dynamic_swarm_formation_ = this->create_subscription<as2_msgs::msg::PoseWithIDArray>(
    "dynamic_swarm_formation", 1,
    std::bind(&SwarmFlockingBehavior::dynamicSwarmFormationCallback, this, _1));
  return true;
}

bool SwarmFlockingBehavior::initDroneReferences()
{
  for (auto drone : drones_) {
    goal_future_handles_.push_back(drone.second->initFollowReference());
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  if (goal_future_handles_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "SwarmFlockingBehavior: unable to init drones' follow references");
    return false;
  }
  bool flag = false;
  while (!flag) {
    flag = true;

    for (auto & drones : drones_) {
      if (!drones.second->checkPosition()) {
        flag = false;
        break;
      }
    }
    if (flag) {
      break;
    }
  }
  return true;
}

bool SwarmFlockingBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> goal)
{
  as2_msgs::action::SwarmFlocking::Goal new_goal = *goal;
  if (!setUpVirtualCentroid(new_goal.virtual_centroid)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SwarmFlockingBehavior: Error setting up the virtual centroid");
    return false;
  } else {
    if (setUpDronesFormation(
        new_goal.virtual_centroid, new_goal.drones_namespace,
        new_goal.swarm_formation))
    {
      if (!initDroneReferences()) {
        RCLCPP_ERROR(this->get_logger(), "SwarmFlockingBehavior: Drones are not in position");
        return false;
      }
    }
  }

  goal_ = *goal;
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior Activated");
  return true;
}

as2_behavior::ExecutionStatus SwarmFlockingBehavior::monitoring(
  const std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle
  <as2_msgs::action::FollowReference>>> goal_future_handles)
{
  as2_behavior::ExecutionStatus local_status;
  for (auto goal_handle : goal_future_handles) {
    switch (goal_handle->get_status()) {
      case rclcpp_action::GoalStatus::STATUS_EXECUTING:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        break;
      case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        break;
      case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
        local_status = as2_behavior::ExecutionStatus::RUNNING;
        break;
      case rclcpp_action::GoalStatus::STATUS_ABORTED:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        break;
      case rclcpp_action::GoalStatus::STATUS_CANCELED:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        break;
      default:
        local_status = as2_behavior::ExecutionStatus::FAILURE;
        break;
    }
  }
  return local_status;
}

as2_behavior::ExecutionStatus SwarmFlockingBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> & goal,
  std::shared_ptr<as2_msgs::action::SwarmFlocking::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::SwarmFlocking::Result> & result_msg)
{
  result_msg = std::make_shared<as2_msgs::action::SwarmFlocking::Result>(result_);
  as2_behavior::ExecutionStatus local_status = monitoring(goal_future_handles_);
  if (local_status == as2_behavior::ExecutionStatus::FAILURE) {
    result_.swarm_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }
  result_.swarm_success = true;
  return as2_behavior::ExecutionStatus::RUNNING;
}

bool SwarmFlockingBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior Deactivated");
  return true;
}

bool SwarmFlockingBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior Paused");
  *(transform_) =
    tf_buffer_->lookupTransform("earth", swarm_base_link_frame_id_, tf2::TimePointZero);
  tfstatic_swarm_broadcaster_->sendTransform(*( transform_));
  for (auto drone : drones_) {
    if (!drone.second->stopFollowReference()) {
      RCLCPP_ERROR(
        this->get_logger(), "Drone %s could not be detached from Swarm tf",
        drone.second->drone_id_.c_str());
      return false;
    }
  }
  goal_future_handles_.clear();
  return true;
}

bool SwarmFlockingBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior Resumed");
  if (!setUpVirtualCentroid(goal_.virtual_centroid)) {
    RCLCPP_ERROR(this->get_logger(), "SwarmFlockingBehavior: Error to follow the reference");
    return false;
  }
  if (!initDroneReferences()) {
    RCLCPP_ERROR(this->get_logger(), "SwarmFlockingBehavior: Drones are not in their reference");
    return false;
  }
  return true;
}

void SwarmFlockingBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior Finished");
    for (auto drone : drones_) {
      if (!drone.second->stopFollowReference()) {
        RCLCPP_ERROR(
          this->get_logger(), "Drone %s could not be detached from Swarm tf",
          drone.second->drone_id_.c_str());
      }
    }
  }
  return;
}

bool SwarmFlockingBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::SwarmFlocking::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "SwarmFlockingBehavior: Modifying flocking");
  goal_ = *goal;
  goal_future_handles_.clear();
  for (auto drone : drones_) {
    if (!drone.second->stopFollowReference()) {
      RCLCPP_ERROR(
        this->get_logger(), "Drone %s could not be detached from Swarm tf",
        drone.second->drone_id_.c_str());
      return false;
    }
  }
  drones_.clear();
  if (!setUpVirtualCentroid(goal_.virtual_centroid)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SwarmFlockingBehavior: Error setting up the virtual centroid");
    return false;
  } else {
    if (setUpDronesFormation(
        goal_.virtual_centroid, goal_.drones_namespace,
        goal_.swarm_formation))
    {
      if (!initDroneReferences()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "SwarmFlockingBehavior: Drones are not in their reference");
        return false;
      }
    }
  }
  return true;
}
