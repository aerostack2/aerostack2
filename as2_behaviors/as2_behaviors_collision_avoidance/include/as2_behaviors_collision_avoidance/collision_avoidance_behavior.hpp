// Copyright 2026 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       collision_avoidance_behavior.hpp
 *  \brief      Collision avoidance behavior node class
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

 #pragma once

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <as2_behavior/behavior_server.hpp>
#include <as2_msgs/action/collision_avoidance.hpp>
#include <as2_msgs/action/go_to_waypoint.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <as2_ca/ca_gateway_client.hpp>
#include <as2_core/state_interface.hpp>
#include <as2_core/names/topics.hpp>

#include "as2_core/synchronous_service_client.hpp"
#include "as2_behaviors_collision_avoidance/collision_avoidance_base.hpp"

namespace collision_avoidance_behavior
{

using CollisionAvoidanceAction = as2_msgs::action::CollisionAvoidance;

class CollisionAvoidanceBehavior
  : public as2_behavior::BehaviorServer<CollisionAvoidanceAction>
{
public:
  explicit CollisionAvoidanceBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CollisionAvoidanceBehavior() override = default;

private:
  bool on_activate(std::shared_ptr<const CollisionAvoidanceAction::Goal> goal) override;
  bool on_modify(std::shared_ptr<const CollisionAvoidanceAction::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & msg) override;
  bool on_pause(const std::shared_ptr<std::string> & msg) override;
  bool on_resume(const std::shared_ptr<std::string> & msg) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const CollisionAvoidanceAction::Goal> & goal,
    std::shared_ptr<CollisionAvoidanceAction::Feedback> & feedback,
    std::shared_ptr<CollisionAvoidanceAction::Result> & result) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

  // Plugin
  std::unique_ptr<pluginlib::ClassLoader<collision_avoidance_base::CollisionAvoidanceBase>>
  loader_;
  std::shared_ptr<collision_avoidance_base::CollisionAvoidanceBase> plugin_;

  // as2_ca client
  std::shared_ptr<as2_ca::CAGatewayClient> ca_client_;

  // State interface — subscribes to self_localization/pose and self_localization/twist.
  // Used in on_activate to prepend the drone's current position to the path so the
  // lock covers the full segment from current pose to the goal.
  StateInterface state_interface_;

  // GoToWaypoint action client for internal motion orchestration
  rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SharedPtr go_to_client_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr goto_path_pause_client_ =
    nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr goto_path_resume_client_ =
    nullptr;

  std::string own_id_;
  std::string current_plugin_name_;
  uint32_t req_id_counter_{0};
  bool released_{false};

  // Stored at activation for lock re-acquisition after a pause/resume cycle
  std::vector<as2_msgs::msg::PoseStampedWithID> stored_path_;
  double stored_safety_dist_{0.0};

  // Motion orchestration state
  enum MotionPhase { IDLE_PHASE, REQUESTING_LOCK, HOLDING_LOCK, EXECUTING_MOTION, RELEASING_LOCK };
  MotionPhase current_phase_{IDLE_PHASE};
  bool motion_rejected_{false};
  bool motion_succeeded_{false};
  bool motion_aborted_{false};
  bool go_to_paused_{false};  // true when GoTo is paused, resume it once lock is re-acquired

  // Stored so on_pause / on_deactivate can cancel the in-flight GoTo
  rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr
    go_to_goal_handle_{nullptr};

  // GoToWaypoint callbacks (set flags checked in on_run)
  void go_to_response_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr & goal_handle);
  void go_to_result_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult & result);

  bool pause_go_to();
  bool resume_go_to();

  bool load_plugin(const std::string & plugin_name);
};

}  // namespace collision_avoidance_behavior
