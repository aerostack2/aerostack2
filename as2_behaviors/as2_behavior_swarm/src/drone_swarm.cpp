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


DroneSwarm::DroneSwarm(
  as2::Node * node_ptr, std::string drone_id,
  geometry_msgs::msg::Pose init_pose)
: node_ptr_(node_ptr)
{
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_ptr);
  drone_id_ = drone_id;
  init_pose_ = init_pose;
  drone_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
    as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
    std::bind(&DroneSwarm::drone_pose_callback, this, std::placeholders::_1));

  // TO DO
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  base_link_frame_id_ = as2::tf::generateTfName(
    node_ptr, drone_id_ + "_ref");
  RCLCPP_INFO(node_ptr->get_logger(), "base_link_frame_id_: %s", base_link_frame_id_.c_str());


  // Static tf
  transform.header.stamp = node_ptr->get_clock()->now();
  transform.header.frame_id = "Swarm";
  transform.child_frame_id = base_link_frame_id_;
  transform.transform.translation.x = init_pose.position.x;
  transform.transform.translation.y = init_pose.position.y;
  transform.transform.translation.z = init_pose.position.z;
  tfstatic_broadcaster_->sendTransform(transform);

  position_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::PositionMotion>(
    node_ptr_);
  platform_info_sub_ = node_ptr->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&DroneSwarm::platform_info_callback, this, std::placeholders::_1));
}
bool DroneSwarm::own_init()
{
  if (position_motion_handler_->sendPositionCommandWithYawSpeed(
      base_link_frame_id_,
      0.0, 0.0, 0.0, 0.0, base_link_frame_id_,
      0.1, 0.1, 0.1))
  {
    return false;
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Drone %s failed attempting to reach the starting position",
      drone_id_.c_str());
  }
  //tengo mis dudas de si va a funcionar, porque no se si el swarm va apoder generar correctamente un servicio follow reference por cada dron que gestione
  if (!this->follow_reference_client_->wait_for_action_server(
      std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Follow Reference Action server not available after waiting.");
    return false;
  }
  auto goal_reference_msg = as2_msgs::action::FollowReference::Goal();
  goal_reference_msg.target_pose.header.frame_id = base_link_frame_id_;
  goal_reference_msg.target_pose.point.x = 0;
  goal_reference_msg.target_pose.point.y = 0;
  goal_reference_msg.target_pose.point.z = 0;
  /*cuando haga el control, tengo que ver como le voy pasando la velocidad*/
  RCLCPP_INFO(node_ptr_->get_logger(), "Follow Reference Action server available.");
  return true;
}

void DroneSwarm::drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _pose_msg)
{
  // self_localization is always on frame earth
  drone_pose_ = *_pose_msg;
}

// TO DO
void DroneSwarm::platform_info_callback(
  const as2_msgs::msg::PlatformInfo::SharedPtr _platform_info_msg)
{
  platform_info_ = *_platform_info_msg;
}

// Check status of FollowReference
as2_behavior::ExecutionStatus DroneSwarm::on_run(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>::SharedPtr & goal_handle)
{
  switch (goal_handle->get_status()) {
    case rclcpp_action::GoalStatus::STATUS_EXECUTING:
      RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference is executing");
      return as2_behavior::ExecutionStatus::RUNNING;
    case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference succeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    case rclcpp_action::GoalStatus::STATUS_CANCELED:
      RCLCPP_ERROR(node_ptr_->get_logger(), "FollowReference was canceled.");
      return as2_behavior::ExecutionStatus::FAILURE;
    case rclcpp_action::GoalStatus::STATUS_ABORTED:
      RCLCPP_ERROR(node_ptr_->get_logger(), "FollowReference was aborted.");
      return as2_behavior::ExecutionStatus::FAILURE;
    default:
      RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference is executing");
      return as2_behavior::ExecutionStatus::RUNNING;
  }


}
