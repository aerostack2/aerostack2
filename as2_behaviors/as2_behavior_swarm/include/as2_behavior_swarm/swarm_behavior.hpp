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
 *  \file       swarm_behavior.hpp
 *  \brief      Aerostack2 swarm_behavior header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

#ifndef AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_
#define AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <drone_swarm.hpp>
#include <swarm_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "as2_behavior_swarm_msgs/action/swarm.hpp"
#include "as2_behavior_swarm_msgs/srv/start_swarm.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SwarmBehavior
  : public as2_behavior::BehaviorServer<as2_behavior_swarm_msgs::action::Swarm>
{
public:
  SwarmBehavior();
  ~SwarmBehavior() {}


  geometry_msgs::msg::PoseStamped initial_centroid_;    // storage de original centroid pose
  std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
  goal_future_handles_;

private:
  as2_behavior_swarm_msgs::action::Swarm::Goal goal_;
  as2_behavior_swarm_msgs::action::Swarm::Feedback feedback_;
  rclcpp::Service<as2_behavior_swarm_msgs::srv::StartSwarm>::SharedPtr service_start_;
  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::TimerBase::SharedPtr timer2_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  std::unordered_map<std::string, std::shared_ptr<DroneSwarm>> drones_;
  std::string swarm_base_link_frame_id_;
  std::shared_ptr<as2::tf::TfHandler> swarm_tf_handler_;
  std::chrono::nanoseconds tf_timeout;
  std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_;
  std::vector<std::string> drones_names_;
  bool start_behavior = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  // Trayectory Generator
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
  trajectory_generator_;
  // Trayectory Command
  as2_msgs::msg::TrajectorySetpoints trajectory_command_;

  rclcpp::Duration eval_time_ = rclcpp::Duration(0, 0);
  rclcpp::Time time_zero_;
  bool first_run_ = true;

  double current_yaw_;

public:
  bool process_goal(
    std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal,
    as2_behavior_swarm_msgs::action::Swarm::Goal & new_goal);
  bool on_activate(
    std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
    std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Feedback> & feedback_msg,
    std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Result> & result_msg) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  void initDrones(
    geometry_msgs::msg::PoseStamped centroid,
    std::vector<std::string> drones);
  as2_behavior::ExecutionStatus monitoring(
    const std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
    goal_future_handles);
  // Callbacks
  void timerCallback();
  // void timerCallback2();
  void startBehavior(
    const std::shared_ptr<as2_behavior_swarm_msgs::srv::StartSwarm::Request> request,
    const std::shared_ptr<as2_behavior_swarm_msgs::srv::StartSwarm::Response> response);

  bool evaluateTrajectory(double eval_time);
  double computeYawAnglePathFacing(
    double vx, double vy);
  bool rotateYaw(
    const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal);
  bool regenerateTrajectory();
};


#endif  // AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_
