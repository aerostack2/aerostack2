/*!*******************************************************************************************
 *  \file       land_plugin_trajectory.cpp
 *  \brief      This file contains the implementation of the land behavior trajectory plugin
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

#include "land_behavior/land_base.hpp"

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace land_plugin_trajectory {
class Plugin : public land_base::LandBase {
  using TrajectoryGeneratorAction     = as2_msgs::action::GeneratePolynomialTrajectory;
  using GoalHandleTrajectoryGenerator = rclcpp_action::ClientGoalHandle<TrajectoryGeneratorAction>;

public:
  void ownInit() {
    node_ptr_->declare_parameter<double>("land_speed_condition_percentage");
    node_ptr_->get_parameter("land_speed_condition_percentage", land_speed_condition_percentage_);
    node_ptr_->declare_parameter<double>("land_speed_condition_height");
    node_ptr_->get_parameter("land_speed_condition_height", land_speed_condition_height_);
    node_ptr_->declare_parameter<double>("land_trajectory_height");
    node_ptr_->get_parameter("land_trajectory_height", land_height_);

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

  bool own_activate(as2_msgs::action::Land::Goal &_goal) override {
    if (!traj_gen_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Trajectory generator action server not available");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator action server available");

    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
        landGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(node_ptr_->get_logger(), "Land to position: %f, %f, %f",
                traj_generator_goal.path[0].pose.position.x,
                traj_generator_goal.path[0].pose.position.y,
                traj_generator_goal.path[0].pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with angle mode: %d", traj_generator_goal.yaw.mode);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", traj_generator_goal.max_speed);

    traj_gen_goal_handle_future_ =
        traj_gen_client_->async_send_goal(traj_generator_goal, traj_gen_goal_options_);

    if (!traj_gen_goal_handle_future_.valid()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Land accepted");
    return true;
  }

  bool own_modify(as2_msgs::action::Land::Goal &_goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land modified");
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
        landGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(node_ptr_->get_logger(), "Land to position: %f, %f, %f",
                traj_generator_goal.path[0].pose.position.x,
                traj_generator_goal.path[0].pose.position.y,
                traj_generator_goal.path[0].pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with angle mode: %d", traj_generator_goal.yaw.mode);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", traj_generator_goal.max_speed);
    return false;
  }

  bool own_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land cancel");
    traj_gen_client_->async_cancel_goal(traj_gen_goal_handle_future_.get());
    return false;
  }

  bool own_pause(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land paused");
    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;

    auto out = traj_gen_pause_client_->sendRequest(req, resp, 3);
    if (out && resp.success) return true;
    return false;
  }

  bool own_resume(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land resumed");
    time_ = node_ptr_->now();

    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;

    auto out = traj_gen_resume_client_->sendRequest(req, resp, 3);
    if (out && resp.success) return true;
    return false;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land end");
    traj_gen_client_->async_cancel_goal(traj_gen_goal_handle_future_.get());
    if (state != as2_behavior::ExecutionStatus::SUCCESS) {
      sendHover();
    }
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
          result_.land_success = false;
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
      result_.land_success = traj_gen_result_;
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "Land failed because trajectory generator finished before land end");
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    if (checkGoalCondition()) {
      result_.land_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void feedback_callback(
      GoalHandleTrajectoryGenerator::SharedPtr,
      const std::shared_ptr<const TrajectoryGeneratorAction::Feedback> feedback) {
    traj_gen_feedback_ = *feedback;
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
  TrajectoryGeneratorAction::Feedback traj_gen_feedback_;

  bool traj_gen_goal_accepted_   = false;
  bool traj_gen_result_received_ = false;
  bool traj_gen_result_          = false;

  rclcpp::Time time_;
  double land_speed_condition_percentage_;
  double land_speed_condition_height_;
  float speed_condition_;
  int time_condition_ = 1;
  float initial_height_;
  float land_height_ = -10.0f;

  bool checkGoalCondition() {
    if (initial_height_ - actual_pose_.pose.position.z > land_speed_condition_height_ &&
        fabs(feedback_.actual_land_speed) < fabs(speed_condition_)) {
      if ((node_ptr_->now() - this->time_).seconds() > time_condition_) {
        return true;
      }
    } else {
      time_ = node_ptr_->now();
    }
    return false;
  }

private:
  as2_msgs::action::GeneratePolynomialTrajectory::Goal landGoalToTrajectoryGeneratorGoal(
      const as2_msgs::action::Land::Goal &_goal) {
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal;

    std_msgs::msg::Header header;
    header.frame_id            = "earth";
    header.stamp               = node_ptr_->now();
    traj_generator_goal.header = header;

    as2_msgs::msg::YawMode yaw_mode;
    yaw_mode.mode           = as2_msgs::msg::YawMode::KEEP_YAW;
    traj_generator_goal.yaw = yaw_mode;

    traj_generator_goal.max_speed = std::abs(_goal.land_speed);

    as2_msgs::msg::PoseWithID land_pose;
    land_pose.id              = "land_point";
    land_pose.pose.position.x = actual_pose_.pose.position.x;
    land_pose.pose.position.y = actual_pose_.pose.position.y;
    land_pose.pose.position.z = land_height_;

    traj_generator_goal.path.push_back(land_pose);

    time_            = node_ptr_->now();
    speed_condition_ = _goal.land_speed * land_speed_condition_percentage_;
    initial_height_  = actual_pose_.pose.position.z;

    return traj_generator_goal;
  }

};  // Plugin class
}  // namespace land_plugin_trajectory

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(land_plugin_trajectory::Plugin, land_base::LandBase)