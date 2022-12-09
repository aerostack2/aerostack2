/*!*******************************************************************************************
 *  \file       dynamic_trajectory_generator.hpp
 *  \brief      dynamic_trajectory_generator header file.
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

#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_motion_reference_handlers/trajectory_motion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define WAYPOINTS_TOPIC "motion_reference/waypoints"
#define PATH_DEBUG_TOPIC "debug/traj_generated"
#define REF_TRAJ_TOPIC "debug/ref_traj_point"

class TrajectoryGenerator : public as2::Node {
 public:
  TrajectoryGenerator();
  ~TrajectoryGenerator(){};
  void setup();
  void run();

 private:
  /** Services **/
  rclcpp::Service<as2_msgs::srv::SendTrajectoryWaypoints>::SharedPtr
      set_trajectory_waypoints_srv_;
  rclcpp::Service<as2_msgs::srv::SendTrajectoryWaypoints>::SharedPtr
      add_trajectory_waypoints_srv_;
  rclcpp::Service<as2_msgs::srv::SetSpeed>::SharedPtr set_speed_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_node_srv_;

  /** Subscriptions **/
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr
      mod_waypoint_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr
      waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  /** Publishers **/
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_point_pub;
  rclcpp::Publisher<as2_msgs::msg::TrajGenInfo>::SharedPtr traj_gen_info_pub_;
  /** Motion Handler **/
  as2::motionReferenceHandlers::TrajectoryMotion motion_handler;

  as2::tf::TfHandler tf_handler_;
  bool evaluate_trajectory_ = false;
  bool has_odom_ = false;

  // dynamic_traj_generator::DynamicTrajectory trajectory_generator_;
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
      trajectory_generator_;

  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  int yaw_mode_ = 0;
  float begin_traj_yaw_ = 0.0f;
  std::vector<double> v_positions_;
  std::vector<double> v_velocities_;
  std::vector<double> v_accelerations_;

  dynamic_traj_generator::References references_;
  geometry_msgs::msg::PoseStamped current_state_pose_;
  geometry_msgs::msg::TwistStamped current_state_twist_;
  as2_msgs::msg::TrajGenInfo traj_gen_info_msg_;

  float prev_vx_ = references_.velocity.x();
  float prev_vy_ = references_.velocity.y();
  bool has_prev_v_ = false;

  std::thread plot_thread_;

  bool first_time_ = true;
  rclcpp::Time time_zero_;
  rclcpp::Duration eval_time_;
  bool publish_trajectory_ = false;

  bool evaluateTrajectory(double _eval_time);
  void updateState();

  /** Publish **/
  void publishTrajectory();

  /** Services Callbacks **/
  void setTrajectoryWaypointsSrvCall(
      const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request>
          _request,
      std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response>
          _response);
  void addTrajectoryWaypointsSrvCall(
      const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request>
          _request,
      std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response>
          _response);
  void setSpeedSrvCall(
      const std::shared_ptr<as2_msgs::srv::SetSpeed::Request> _request,
      std::shared_ptr<as2_msgs::srv::SetSpeed::Response> _response);

  void runNodeSrvCall(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> _response);
  /** Topic Callbacks **/
  void modifyWaypointCallback(
      const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg);
  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void waypointsCallback(
      const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg);

  void yawCallback(const std_msgs::msg::Float32::SharedPtr _msg);

  bool has_yaw_from_topic_ = false;
  float yaw_from_topic_ = 0.0f;

  /** Debug functions **/
  void plotTrajectory();
  void plotTrajectoryThread();
  void plotRefTrajPoint();
  void publishTrajGenInfo();
  void stop() {
    if (plot_thread_.joinable()) {
      plot_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Plot thread joined");
    }
    trajectory_generator_.reset();
    traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::STOPPED;
    RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator stopped");
  };
};

/** Auxiliar Functions **/
double extractYawFromQuat(const geometry_msgs::msg::Quaternion &quat);

void generateDynamicPoint(
    const as2_msgs::msg::PoseStampedWithID &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(
    const geometry_msgs::msg::PoseStamped &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(
    const nav_msgs::msg::Odometry &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);

#endif  // TRAJECTORY_GENERATOR_HPP_
