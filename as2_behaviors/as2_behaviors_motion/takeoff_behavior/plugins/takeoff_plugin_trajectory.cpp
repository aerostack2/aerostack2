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

/**
 * @file takeoff_trajectory.cpp
 *
 * This file contains the implementation of the take off behavior trajectory plugin
 *
 * @authors Rafael Perez-Segui
 */

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "takeoff_behavior/takeoff_base.hpp"

namespace takeoff_plugin_trajectory
{

class Plugin : public takeoff_base::TakeoffBase
{
  using TrajectoryGeneratorAction = as2_msgs::action::GeneratePolynomialTrajectory;
  using GoalHandleTrajectoryGenerator = rclcpp_action::ClientGoalHandle<TrajectoryGeneratorAction>;

public:
  void ownInit()
  {
    traj_gen_client_ = rclcpp_action::create_client<TrajectoryGeneratorAction>(
      node_ptr_, as2_names::actions::behaviors::trajectorygenerator);

    traj_gen_goal_options_.feedback_callback =
      std::bind(&Plugin::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    traj_gen_goal_options_.result_callback =
      std::bind(&Plugin::result_callback, this, std::placeholders::_1);
  }

  bool own_activate(as2_msgs::action::Takeoff::Goal & _goal) override
  {
    if (!traj_gen_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Trajectory generator action server not available");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator action server available");

    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
      takeoffGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(
      node_ptr_->get_logger(), "Takeoff to position: %f, %f, %f",
      traj_generator_goal.path[0].pose.position.x,
      traj_generator_goal.path[0].pose.position.y,
      traj_generator_goal.path[0].pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with angle: %f", traj_generator_goal.yaw.angle);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with speed: %f", traj_generator_goal.max_speed);

    traj_gen_goal_handle_future_ =
      traj_gen_client_->async_send_goal(traj_generator_goal, traj_gen_goal_options_);

    if (!traj_gen_goal_handle_future_.valid()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff accepted");
    return true;
  }

  bool own_modify(as2_msgs::action::Takeoff::Goal & _goal) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff modified");
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal =
      takeoffGoalToTrajectoryGeneratorGoal(_goal);

    RCLCPP_INFO(
      node_ptr_->get_logger(), "Takeoff to position: %f, %f, %f",
      traj_generator_goal.path[0].pose.position.x,
      traj_generator_goal.path[0].pose.position.y,
      traj_generator_goal.path[0].pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with angle: %f", traj_generator_goal.yaw.angle);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with speed: %f", traj_generator_goal.max_speed);

    // TODO(RPS98): call trajectory generator modify service with new goal
    RCLCPP_ERROR(node_ptr_->get_logger(), "Takeoff modify not implemented yet");

    return false;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff cancel");
    // TODO(RPS98): cancel trajectory generator
    RCLCPP_ERROR(node_ptr_->get_logger(), "Takeoff cancel not implemented yet");
    traj_gen_client_->async_cancel_goal(traj_gen_goal_handle_future_.get());
    return false;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
    traj_gen_goal_accepted_ = false;
    traj_gen_result_received_ = false;
    traj_gen_result_ = false;
    return;
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    if (!traj_gen_goal_accepted_) {
      if (traj_gen_goal_handle_future_.valid() &&
        traj_gen_goal_handle_future_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        auto result = traj_gen_goal_handle_future_.get();
        if (!result) {
          // RCLCPP_WARN(node_ptr_->get_logger(), "Trajectory generator goal not accepted");
          RCLCPP_ERROR(node_ptr_->get_logger(), "Trajectory generator goal not accepted");
          result_.takeoff_success = false;
          return as2_behavior::ExecutionStatus::FAILURE;
        }
        RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator goal accepted");
        traj_gen_goal_accepted_ = true;
      } else {
        auto & clk = *node_ptr_->get_clock();
        RCLCPP_INFO_THROTTLE(
          node_ptr_->get_logger(), clk, 5000,
          "Waiting for trajectory generator goal to be accepted");
        return as2_behavior::ExecutionStatus::RUNNING;
      }
    }

    if (traj_gen_result_received_) {
      result_.takeoff_success = traj_gen_result_;
      if (traj_gen_result_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff successful");
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff failed");
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    }

    // Waiting for result
    auto & clk = *node_ptr_->get_clock();
    RCLCPP_INFO_THROTTLE(
      node_ptr_->get_logger(), clk, 5000,
      "Waiting for trajectory generator result");
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void feedback_callback(
    GoalHandleTrajectoryGenerator::SharedPtr,
    const std::shared_ptr<const TrajectoryGeneratorAction::Feedback> feedback)
  {
    traj_gen_feedback_ = *feedback;
    return;
  }

  void result_callback(const GoalHandleTrajectoryGenerator::WrappedResult & result)
  {
    traj_gen_result_received_ = true;
    traj_gen_result_ = result.result->trajectory_generator_success;
    return;
  }

private:
  std::shared_ptr<rclcpp_action::Client<TrajectoryGeneratorAction>> traj_gen_client_ = nullptr;
  rclcpp_action::Client<TrajectoryGeneratorAction>::SendGoalOptions traj_gen_goal_options_;
  std::shared_future<GoalHandleTrajectoryGenerator::SharedPtr> traj_gen_goal_handle_future_;
  TrajectoryGeneratorAction::Feedback traj_gen_feedback_;

  bool traj_gen_goal_accepted_ = false;
  bool traj_gen_result_received_ = false;
  bool traj_gen_result_ = false;

private:
  as2_msgs::action::GeneratePolynomialTrajectory::Goal takeoffGoalToTrajectoryGeneratorGoal(
    const as2_msgs::action::Takeoff::Goal & _goal)
  {
    as2_msgs::action::GeneratePolynomialTrajectory::Goal traj_generator_goal;

    traj_generator_goal.header.stamp = node_ptr_->now();
    traj_generator_goal.header.frame_id = "earth";

    traj_generator_goal.yaw.mode = as2_msgs::msg::YawMode::FIXED_YAW;
    traj_generator_goal.yaw.angle = as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);

    traj_generator_goal.max_speed = _goal.takeoff_speed;

    as2_msgs::msg::PoseWithID takeoff_pose;
    takeoff_pose.id = "takeoff_point";
    takeoff_pose.pose.position.x = actual_pose_.pose.position.x;
    takeoff_pose.pose.position.y = actual_pose_.pose.position.y;
    takeoff_pose.pose.position.z = actual_pose_.pose.position.z + _goal.takeoff_height;

    traj_generator_goal.path.push_back(takeoff_pose);

    return traj_generator_goal;
  }
};  // Plugin class
}  // namespace takeoff_plugin_trajectory

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_trajectory::Plugin, takeoff_base::TakeoffBase)
