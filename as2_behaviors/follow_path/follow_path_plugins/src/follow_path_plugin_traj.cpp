/*!*******************************************************************************************
 *  \file       follow_path_plugin_traj.cpp
 *  \brief      follow_path_plugin_traj implementation file
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "as2_core/names/services.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "follow_path_base.hpp"

#include "as2_core/synchronous_service_client.hpp"

namespace follow_path_plugin_traj {
class Plugin : public follow_path_base::FollowPathBase {
  using YAW_MODE         = as2_msgs::msg::TrajectoryWaypointsWithID;
  using SyncSetSpeed     = as2::SynchronousServiceClient<as2_msgs::srv::SetSpeed>;
  using SyncSendTrajWayp = as2::SynchronousServiceClient<as2_msgs::srv::SendTrajectoryWaypoints>;

public:
  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    auto req_speed  = as2_msgs::srv::SetSpeed::Request();
    auto resp_speed = as2_msgs::srv::SetSpeed::Response();

    req_speed.speed.speed = goal->trajectory_waypoints.max_speed;

    auto set_traj_speed_cli =
        SyncSetSpeed(as2_names::services::motion_reference::set_traj_speed, node_ptr_);
    if (!set_traj_speed_cli.sendRequest(req_speed, resp_speed, 1)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    auto req_traj  = as2_msgs::srv::SendTrajectoryWaypoints::Request();
    auto resp_traj = as2_msgs::srv::SendTrajectoryWaypoints::Response();
    as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints = goal->trajectory_waypoints;

    // Populate Waypoints queue
    for (auto &pose : trajectory_waypoints.poses) {
      waypoints_.emplace_back(
          Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
      as2_msgs::msg::PoseStampedWithID pose_with_id;
      pose_with_id.pose = pose.pose;
      req_traj.waypoints.poses.emplace_back(pose_with_id);
    }
    req_traj.waypoints.yaw_mode = goal->trajectory_waypoints.yaw_mode;

    last_waypoint_ = waypoints_.back();

    auto send_traj_wayp_cli =
        SyncSendTrajWayp(as2_names::services::motion_reference::send_traj_wayp, node_ptr_);
    if (!send_traj_wayp_cli.sendRequest(req_traj, resp_traj, 1)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(
      const std::shared_ptr<GoalHandleFollowPath> goal_handle) override {
    // TODO: since follow path is done by traj_gen + controllor, cancel has to be also handle by
    // them
    waypoints_.clear();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  bool onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) override {
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback   = std::make_shared<as2_msgs::action::FollowPath::Feedback>();
    auto result     = std::make_shared<as2_msgs::action::FollowPath::Result>();

    // FIXME get next (first) from queue
    Eigen::Vector3d next_wayp = waypoints_.front();
    waypoints_.pop_front();

    Eigen::Vector3d position(this->current_pose_x_, this->current_pose_y_, this->current_pose_z_);
    float distance_to_next_waypoint = (position - next_wayp).norm();

    time_ = node_ptr_->now();

    // Check if goal is done
    while (!checkGoalCondition()) {
      if (goal_handle->is_canceling()) {
        result->follow_path_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(node_ptr_->get_logger(), "Goal cannot be cancelled");
        // return;
      }

      position =
          Eigen::Vector3d(this->current_pose_x_, this->current_pose_y_, this->current_pose_z_);
      distance_to_next_waypoint = (position - next_wayp).norm();

      if ((distance_to_next_waypoint < goal_threshold_) && !waypoints_.empty()) {
        next_wayp = waypoints_.front();
        waypoints_.pop_front();
      }

      feedback->next_waypoint.x                  = next_wayp[0];
      feedback->next_waypoint.y                  = next_wayp[1];
      feedback->next_waypoint.z                  = next_wayp[2];
      feedback->remaining_waypoints              = waypoints_.size();
      feedback->actual_distance_to_next_waypoint = distance_to_next_waypoint;
      feedback->actual_speed                     = actual_speed_;

      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->follow_path_success = true;
    waypoints_.clear();

    goal_handle->succeed(result);
    return true;
  }

private:
  bool checkGoalCondition() {
    if (waypoints_.empty()) {
      return true;
    }

    Eigen::Vector3d position =
        Eigen::Vector3d(this->current_pose_x_, this->current_pose_y_, this->current_pose_z_);
    float distance_to_last_waypoint = (position - last_waypoint_).norm();
    if (actual_speed_ < 0.1 && distance_to_last_waypoint < goal_threshold_ * 5.0) {
      if ((node_ptr_->now() - time_) > rclcpp::Duration(1, 0)) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "Ending follow path because UAV is hovering for %f seconds over last waypoint",
                    (node_ptr_->now() - time_).seconds());
        return true;
      }
    } else {
      time_ = node_ptr_->now();
    }
    return false;
  };

  rclcpp::Time time_;
  Eigen::Vector3d last_waypoint_;
};  // Plugin class
}  // namespace follow_path_plugin_traj

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugin_traj::Plugin, follow_path_base::FollowPathBase)
