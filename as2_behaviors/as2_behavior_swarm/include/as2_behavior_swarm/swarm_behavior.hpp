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

#ifndef AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_
#define AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
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
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

class SwarmBehavior
  : public as2_behavior::BehaviorServer<as2_behavior_swarm_msgs::action::Swarm>
{
public:
  SwarmBehavior();
  ~SwarmBehavior() {}

  std::shared_ptr<geometry_msgs::msg::PoseStamped>  new_centroid_;
  geometry_msgs::msg::PoseStamped centroid_;
  std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
  goal_future_handles_;
  

private:

  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  std::unordered_map<std::string, std::shared_ptr<DroneSwarm>> drones_;
  std::string swarm_base_link_frame_id_;  
  std::shared_ptr<as2::tf::TfHandler> swarm_tf_handler_;
  std::chrono::nanoseconds tf_timeout;
  geometry_msgs::msg::TransformStamped transform;
  std::vector<std::string> drones_names_ = {"drone0", "drone1"};
  // Trayectory Generator
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
  trajectory_generator_;
    // Command
  as2_msgs::msg::TrajectorySetpoints trajectory_command_;
  // numero de puntos que queremos muestrear
  int sampling_n_=1;
  // tiempo en que se va a avealuar la trayectoria
  //esta variable se va a ir actualizando en el on_run como el tiempo que ha ido transcurriendo.
  rclcpp::Duration eval_time_ = rclcpp::Duration(0, 0); // inizializas el tiempo a 0s y 0ns.
  double sampling_dt_ = 0.01; // diferencial de tiempo
  rclcpp::Time time_zero_;
  bool first_run_ = false;


public:
  bool process_goal(
    std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal,
    as2_behavior_swarm_msgs::action::Swarm::Goal & new_goal);
  bool on_activate(
    std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> goal);
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_behavior_swarm_msgs::action::Swarm::Goal> & goal,
    std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Feedback> & feedback_msg,
    std::shared_ptr<as2_behavior_swarm_msgs::action::Swarm::Result> & result_msg);
  // on_modify
  // on_deactivate
  // on_pause
  // on_resume
  // on_execution_end

private:
  void init_drones(
    geometry_msgs::msg::PoseStamped centroid,
    std::vector<std::string> drones);
  as2_behavior::ExecutionStatus monitoring(
    const std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowReference>>>
    goal_future_handles);
  void update_pose( std::shared_ptr<const geometry_msgs::msg::PoseStamped> centroid, std::shared_ptr<geometry_msgs::msg::PoseStamped> & update_centroid);
  // cogemos el centroide vemos la posicion donde esta y calculando almacenamos la nueva desired pose en update_centroid, en el timer_callback despues de enviar la nueva posición del tf, actalizamos el centroide a la posicion actual
  // Callbacks
  void timer_callback();
  void setup();
  bool evaluateTrajectory(double eval_time);

};


#endif  // AS2_BEHAVIOR_SWARM__SWARM_BEHAVIOR_HPP_
