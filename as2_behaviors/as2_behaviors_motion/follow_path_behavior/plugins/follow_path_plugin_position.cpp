/*!*******************************************************************************************
 *  \file       follow_path_plugin_position.cpp
 *  \brief      This file contains the implementation of the follow path behavior position plugin
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
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

#include "as2_motion_reference_handlers/position_motion.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"
#include "follow_path_behavior/follow_path_base.hpp"
#include "std_msgs/msg/header.hpp"

namespace follow_path_plugin_position {
class Plugin : public follow_path_base::FollowPathBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool own_activate(as2_msgs::action::FollowPath::Goal &_goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path goal accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f", _goal.max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with yaw mode: %d", _goal.yaw.mode);

    path_ids_.reserve(_goal.path.size());
    path_ids_remaining_.reserve(_goal.path.size());
    for (auto &point : _goal.path) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Follow path to point %s: %f, %f, %f", point.id.c_str(),
                  point.pose.position.x, point.pose.position.y, point.pose.position.z);
      path_ids_.push_back(point.id);
    }
    path_ids_remaining_ = path_ids_;
    initial_yaw_        = as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
    updateDesiredPose(_goal, path_ids_remaining_[0]);
    feedback_.next_waypoint_id    = path_ids_remaining_.front();
    feedback_.remaining_waypoints = path_ids_remaining_.size();
    return true;
  }

  bool own_modify(as2_msgs::action::FollowPath::Goal &_goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path modiy accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f", _goal.max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with yaw mode: %d", _goal.yaw.mode);

    for (auto &point : _goal.path) {
      if (std::find(path_ids_.begin(), path_ids_.end(), point.id) == path_ids_.end()) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path modify point %s: %f, %f, %f",
                    point.id.c_str(), point.pose.position.x, point.pose.position.y,
                    point.pose.position.z);
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path add point %s: %f, %f, %f",
                    point.id.c_str(), point.pose.position.x, point.pose.position.y,
                    point.pose.position.z);
        path_ids_.push_back(point.id);
        path_ids_remaining_.push_back(point.id);
      }
    }
    initial_yaw_ = as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
    updateDesiredPose(_goal, path_ids_remaining_[0]);
    feedback_.next_waypoint_id    = path_ids_remaining_.front();
    feedback_.remaining_waypoints = path_ids_remaining_.size();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path resumed");
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path end");
    path_ids_.clear();
    path_ids_remaining_.clear();
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      // Leave the drone in the last position
      position_motion_handler_->sendPositionCommandWithYawAngle(desired_pose_, desired_twist_);
      return;
    }
    sendHover();
    return;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.follow_path_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(desired_pose_, desired_twist_)) {
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  Eigen::Vector3d getTargetPosition() override {
    return Eigen::Vector3d(desired_pose_.pose.position.x, desired_pose_.pose.position.y,
                           desired_pose_.pose.position.z);
  }

  geometry_msgs::msg::Quaternion processYaw(as2_msgs::action::FollowPath::Goal &_goal,
                                            const std::string &id) {
    geometry_msgs::msg::Quaternion orientation;
    switch (_goal.yaw.mode) {
      case as2_msgs::msg::YawMode::KEEP_YAW:
        _goal.yaw.angle = initial_yaw_;
        break;
      case as2_msgs::msg::YawMode::PATH_FACING: {
        Eigen::Vector2d next_pose = {desired_pose_.pose.position.x, desired_pose_.pose.position.y};
        Eigen::Vector2d actual_pose = {actual_pose_.pose.position.x, actual_pose_.pose.position.y};
        _goal.yaw.angle = as2::frame::getVector2DAngle((next_pose.x() - actual_pose.x()),
                                                       (next_pose.y() - actual_pose.y()));
        break;
      }
      case as2_msgs::msg::YawMode::FIXED_YAW:
        break;
      default:
        RCLCPP_WARN(node_ptr_->get_logger(), "Yaw mode %d not supported, using KEEP_YAW",
                    _goal.yaw.mode);
        _goal.yaw.angle = initial_yaw_;
        break;
    }

    as2::frame::eulerToQuaternion(0.0, 0.0, _goal.yaw.angle, orientation);
    return orientation;
  }

  void updateDesiredPose(as2_msgs::action::FollowPath::Goal &_goal,
                         const std::string &waypoint_id) {
    for (auto &waypoint : _goal.path) {
      if (waypoint.id == waypoint_id) {
        desired_pose_.header.frame_id  = _goal.header.frame_id;
        desired_pose_.header.stamp     = node_ptr_->now();
        desired_pose_.pose.position.x  = waypoint.pose.position.x;
        desired_pose_.pose.position.y  = waypoint.pose.position.y;
        desired_pose_.pose.position.z  = waypoint.pose.position.z;
        desired_pose_.pose.orientation = processYaw(_goal, waypoint.id);

        desired_twist_.header.frame_id = _goal.header.frame_id;
        desired_twist_.header.stamp    = node_ptr_->now();
        desired_twist_.twist.linear.x  = _goal.max_speed;
        desired_twist_.twist.linear.y  = _goal.max_speed;
        desired_twist_.twist.linear.z  = _goal.max_speed;

        RCLCPP_DEBUG(node_ptr_->get_logger(), "Next waypoint: %s", waypoint_id.c_str());
        RCLCPP_DEBUG(node_ptr_->get_logger(), "Next waypoint: [%f, %f, %f]",
                     waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        return;
      }
    }
    return;
  }

private:
  std::vector<std::string> path_ids_;
  std::vector<std::string> path_ids_remaining_;
  double initial_yaw_;
  geometry_msgs::msg::PoseStamped desired_pose_;
  geometry_msgs::msg::TwistStamped desired_twist_;

  bool checkGoalCondition() {
    if (!localization_flag_) {
      return false;
    }

    if (fabs(feedback_.actual_distance_to_next_waypoint) < params_.follow_path_threshold) {
      path_ids_remaining_.erase(path_ids_remaining_.begin());
      if (path_ids_remaining_.empty()) {
        return true;
      }
      feedback_.next_waypoint_id    = path_ids_remaining_.front();
      feedback_.remaining_waypoints = path_ids_remaining_.size();
      updateDesiredPose(goal_, path_ids_remaining_[0]);
    }
    return false;
  }

};  // Plugin class
}  // namespace follow_path_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugin_position::Plugin, follow_path_base::FollowPathBase)
