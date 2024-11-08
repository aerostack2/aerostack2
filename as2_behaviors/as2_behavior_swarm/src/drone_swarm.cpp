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
    drone_id_ + "/" + as2_names::topics::self_localization::pose,
    as2_names::topics::self_localization::qos,
    std::bind(&DroneSwarm::drone_pose_callback, this, std::placeholders::_1));

  // TO DO
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  base_link_frame_id_ = as2::tf::generateTfName(
    node_ptr, drone_id_ + "_ref");
  parent_frame_id = as2::tf::generateTfName(node_ptr, "Swarm");
  RCLCPP_INFO(node_ptr->get_logger(), "base_link_frame_id_: %s", base_link_frame_id_.c_str());


  // Static tf
  transform.header.stamp = node_ptr->get_clock()->now();
  transform.header.frame_id = parent_frame_id;
  transform.child_frame_id = base_link_frame_id_;
  transform.transform.translation.x = init_pose.position.x;
  transform.transform.translation.y = init_pose.position.y;
  transform.transform.translation.z = init_pose.position.z;
  tfstatic_broadcaster_->sendTransform(transform);

  // position_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::PositionMotion>(
  //   node_ptr_, drone_id_);
  platform_info_sub_ = node_ptr->create_subscription<as2_msgs::msg::PlatformInfo>(
    drone_id_ + "/" + as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&DroneSwarm::platform_info_callback, this, std::placeholders::_1));
  cbk_group_ = node_ptr->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  go_to_client_ = rclcpp_action::create_client<as2_msgs::action::GoToWaypoint>(
    node_ptr_,
    drone_id_ + "/" + as2_names::actions::behaviors::gotowaypoint, cbk_group_);
  follow_reference_client_ = rclcpp_action::create_client<as2_msgs::action::FollowReference>(
    node_ptr_,
    drone_id_ + "/" + as2_names::actions::behaviors::followreference, cbk_group_);

}
bool DroneSwarm::own_init()
{
  // /* GoTO server */
  // if (!go_to_client_->wait_for_action_server(std::chrono::seconds(5))) {
  //   RCLCPP_ERROR(
  //     node_ptr_->get_logger(),
  //     "GoTo Action server not available "
  //     "after waiting. Aborting.");
  //   return false;
  // }

  // auto goal_msg = as2_msgs::action::GoToWaypoint::Goal();
  // goal_msg.target_pose.header.frame_id = "earth";
  // goal_msg.target_pose.header.stamp = node_ptr_->get_clock()->now();
  // goal_msg.target_pose.point.x = init_pose_.position.x;
  // goal_msg.target_pose.point.y = init_pose_.position.y;
  // goal_msg.target_pose.point.z = init_pose_.position.z;
  // goal_msg.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  // goal_msg.max_speed = 0.5;


  // auto goal_handle_future_go_to = go_to_client_->async_send_goal(goal_msg);

  // RCLCPP_INFO(node_ptr_->get_logger(), "Sending goal to GoTo behavior");
  // auto goal_handle_go_to = goal_handle_future_go_to.get();
  // if (goal_handle_go_to == nullptr) {
  //   RCLCPP_ERROR(node_ptr_->get_logger(), "GoTo rejected from server");
  //   return false;
  // } else {
  //   RCLCPP_INFO(node_ptr_->get_logger(), "GoTo accepted from server");
  // }

  /* FollowReference */
  if (!follow_reference_client_->wait_for_action_server(
      std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Follow Reference Action server not available after waiting.");
    return false;
  }
  auto goal_reference_msg = as2_msgs::action::FollowReference::Goal();
  goal_reference_msg.target_pose.header.frame_id = "earth";
  goal_reference_msg.target_pose.header.stamp = node_ptr_->get_clock()->now();
  goal_reference_msg.target_pose.point.x = init_pose_.position.x;
  goal_reference_msg.target_pose.point.y = init_pose_.position.y;
  goal_reference_msg.target_pose.point.z = init_pose_.position.z;
  goal_reference_msg.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  goal_reference_msg.max_speed_x = 0.5;
  goal_reference_msg.max_speed_y = 0.5;
  goal_reference_msg.max_speed_z = 0.5;

  auto goal_handle_future_follow_reference = follow_reference_client_->async_send_goal(
    goal_reference_msg);
  /*cuando haga el control, tengo que ver como le voy pasando la velocidad*/
  RCLCPP_INFO(node_ptr_->get_logger(), "Sending reference to FollowReference behavior");
  auto goal_handle_follow_reference = goal_handle_future_follow_reference.get();
  if (goal_handle_follow_reference == nullptr) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "FollowReference rejected from server");
    return false;
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference accepted from server");
  }
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
