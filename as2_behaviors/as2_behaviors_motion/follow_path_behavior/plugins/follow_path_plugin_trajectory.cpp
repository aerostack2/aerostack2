/*!*******************************************************************************************
 *  \file       follow_path_plugin_trajectory.cpp
 *  \brief      This file contains the implementation of the go to behavior trajectory plugin
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

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "follow_path_behavior/follow_path_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace follow_path_plugin_trajectory {
class Plugin : public follow_path_base::FollowPathBase {
  using TrajectoryGeneratorAction     = as2_msgs::action::GeneratePolynomialTrajectory;
  using GoalHandleTrajectoryGenerator = rclcpp_action::ClientGoalHandle<TrajectoryGeneratorAction>;

public:
  void ownInit() {
    traj_gen_client_ = rclcpp_action::create_client<TrajectoryGeneratorAction>(
        node_ptr_, as2_names::actions::behaviors::trajectorygenerator);

    traj_gen_pause_client_ =
        std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
            as2_names::actions::behaviors::trajectorygenerator + "/_behavior/pause", node_ptr_);

    traj_gen_resume_client_ =
        std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
            as2_names::actions::behaviors::trajectorygenerator + "/_behavior/resume", node_ptr_);

    traj_gen_goal_options_.feedback_callback =
        std::bind(&Plugin::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    traj_gen_goal_options_.result_callback =
        std::bind(&Plugin::result_callback, this, std::placeholders::_1);
  }

  bool own_activate(as2_msgs::action::FollowPath::Goal &_goal) override {
    if (!traj_gen_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Trajectory generator action server not available");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator action server available");

    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
        followPathGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f",
                traj_generator_goal.max_speed);

    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path to positions:");
    for (auto &point : traj_generator_goal.path) {
      RCLCPP_INFO(node_ptr_->get_logger(), "  x: %f, y: %f, z: %f", point.pose.position.x,
                  point.pose.position.y, point.pose.position.z);
    }

    traj_gen_goal_handle_future_ =
        traj_gen_client_->async_send_goal(traj_generator_goal, traj_gen_goal_options_);

    if (!traj_gen_goal_handle_future_.valid()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path accepted");
    return true;
  }

  bool own_modify(as2_msgs::action::FollowPath::Goal &_goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path modified");
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
        followPathGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f",
                traj_generator_goal.max_speed);

    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path to positions:");
    for (auto &point : traj_generator_goal.path) {
      RCLCPP_INFO(node_ptr_->get_logger(), "  x: %f, y: %f, z: %f", point.pose.position.x,
                  point.pose.position.y, point.pose.position.z);
    }

    // TODO: call trajectory generator modify service with new goal
    RCLCPP_ERROR(node_ptr_->get_logger(), "Follow path modify not implemented yet");

    return false;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path cancel");
    // TODO: cancel trajectory generator
    traj_gen_client_->async_cancel_goal(traj_gen_goal_handle_future_.get());
    return false;
  }

  bool own_pause(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path paused");
    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;

    auto out = traj_gen_pause_client_->sendRequest(req, resp, 3);
    if (out && resp.success) return true;
    return false;
  }

  bool own_resume(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path resumed");
    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;

    auto out = traj_gen_resume_client_->sendRequest(req, resp, 3);
    if (out && resp.success) return true;
    return false;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path end");
    sendHover();
    traj_gen_result_received_ = false;
    traj_gen_goal_accepted_   = false;
    traj_gen_result_          = false;
    return;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (!traj_gen_goal_accepted_) {
      if (traj_gen_goal_handle_future_.valid() &&
          traj_gen_goal_handle_future_.wait_for(std::chrono::seconds(0)) ==
              std::future_status::ready) {
        auto result = traj_gen_goal_handle_future_.get();
        if (result) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator goal accepted");
          traj_gen_goal_accepted_ = true;
        } else {
          RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator goal accepted");
          result_.follow_path_success = false;
          return as2_behavior::ExecutionStatus::FAILURE;
        }
      } else {
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Waiting for trajectory generator goal to be accepted");
        return as2_behavior::ExecutionStatus::RUNNING;
      }
    }

    if (traj_gen_result_received_) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator result received: %d",
                  traj_gen_result_);
      result_.follow_path_success = traj_gen_result_;
      if (traj_gen_result_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path successful");
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path failed");
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    }

    // Waiting for result
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "Waiting for trajectory generator result");
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  Eigen::Vector3d getTargetPosition() override {
    return Eigen::Vector3d(desired_pose_.position.x, desired_pose_.position.y,
                           desired_pose_.position.z);
  }

  void feedback_callback(
      GoalHandleTrajectoryGenerator::SharedPtr,
      const std::shared_ptr<const TrajectoryGeneratorAction::Feedback> feedback) {
    if (feedback->next_waypoint_id != feedback_.next_waypoint_id) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Next waypoint id: %s",
                  feedback->next_waypoint_id.c_str());
      RCLCPP_INFO(node_ptr_->get_logger(), "Remaining waypoints: %d",
                  feedback->remaining_waypoints);
      // Update target position
      for (auto waypoint : goal_.path) {
        if (waypoint.id == feedback->next_waypoint_id) {
          desired_pose_ = waypoint.pose;
          break;
        }
      }
    }

    feedback_.remaining_waypoints = feedback->remaining_waypoints;
    feedback_.next_waypoint_id    = feedback->next_waypoint_id;
    return;
  }

  void result_callback(const GoalHandleTrajectoryGenerator::WrappedResult &result) {
    traj_gen_result_received_ = true;
    traj_gen_result_          = result.result->trajectory_generator_success;
    return;
  }

private:
  std::shared_ptr<rclcpp_action::Client<TrajectoryGeneratorAction>> traj_gen_client_      = nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr traj_gen_pause_client_ = nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr traj_gen_resume_client_ =
      nullptr;
  rclcpp_action::Client<TrajectoryGeneratorAction>::SendGoalOptions traj_gen_goal_options_;
  std::shared_future<GoalHandleTrajectoryGenerator::SharedPtr> traj_gen_goal_handle_future_;

  bool traj_gen_goal_accepted_   = false;
  bool traj_gen_result_received_ = false;
  bool traj_gen_result_          = false;

  geometry_msgs::msg::Pose desired_pose_;

private:
  as2_msgs::action::GeneratePolynomialTrajectory::Goal followPathGoalToTrajectoryGeneratorGoal(
      const as2_msgs::action::FollowPath::Goal &_goal) {
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal;

    traj_generator_goal.header    = _goal.header;
    traj_generator_goal.yaw       = _goal.yaw;
    traj_generator_goal.max_speed = _goal.max_speed;
    traj_generator_goal.path      = _goal.path;

    return traj_generator_goal;
  }

};  // Plugin class
}  // namespace follow_path_plugin_trajectory

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugin_trajectory::Plugin, follow_path_base::FollowPathBase)
