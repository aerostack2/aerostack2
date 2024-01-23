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

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_motion_reference_handlers/trajectory_motion.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define PATH_DEBUG_TOPIC "debug/traj_generated"
#define REF_TRAJ_TOPIC "debug/ref_traj_point"

class DynamicPolynomialTrajectoryGenerator
    : public as2_behavior::BehaviorServer<
          as2_msgs::action::GeneratePolynomialTrajectory> {
 public:
  DynamicPolynomialTrajectoryGenerator(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DynamicPolynomialTrajectoryGenerator(){};

 private:
  /** Subscriptions **/
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;

  // For faster waypoint modified
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr
      mod_waypoint_sub_;

  /** Dynamic trajectory generator library */
  // dynamic_traj_generator::DynamicTrajectory trajectory_generator_;
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
      trajectory_generator_;

  /** Handlers **/
  as2::motionReferenceHandlers::HoverMotion hover_motion_handler_;
  as2::motionReferenceHandlers::TrajectoryMotion trajectory_motion_handler_;
  as2::tf::TfHandler tf_handler_;

  /** Parameters */
  // Parameters
  std::string base_link_frame_id_;
  std::string desired_frame_id_;

  // Behavior action parameters
  as2_msgs::msg::YawMode yaw_mode_;
  as2_msgs::action::GeneratePolynomialTrajectory::Goal goal_;
  as2_msgs::action::GeneratePolynomialTrajectory::Feedback feedback_;
  as2_msgs::action::GeneratePolynomialTrajectory::Result result_;

  // Yaw from topic
  bool has_yaw_from_topic_ = false;
  float yaw_from_topic_ = 0.0f;

  // Initial yaw
  float init_yaw_angle_ = 0.0f;

  // State
  Eigen::Vector3d current_position_;
  double current_yaw_;

  // Command
  double yaw_angle_ = 0.0;
  dynamic_traj_generator::References traj_command_;

  // Trajectory generator
  rclcpp::Time time_zero_;
  bool first_run_ = false;
  bool has_odom_ = false;

  // Debug
  bool enable_debug_ = true;
  std::thread plot_thread_;

 private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<
                   const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
                       goal) override;

  bool on_modify(std::shared_ptr<
                 const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
                     goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> &message) override;

  bool on_pause(const std::shared_ptr<std::string> &message) override;

  bool on_resume(const std::shared_ptr<std::string> &message) override;

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<
          const as2_msgs::action::GeneratePolynomialTrajectory::Goal> &goal,
      std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Feedback>
          &feedback_msg,
      std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Result>
          &result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus &state) override;

  /** Topic Callbacks **/
  void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void yawCallback(const std_msgs::msg::Float32::SharedPtr _msg);

  // For faster waypoint modified
  void modifyWaypointCallback(
      const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg);

  /** Trajectory generator functions */
  void setup();
  bool goalToDynamicWaypoint(
      std::shared_ptr<
          const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
          goal,
      dynamic_traj_generator::DynamicWaypoint::Vector &waypoints);
  bool evaluateTrajectory(double _eval_time);
  double computeYawAnglePathFacing();

 private:
  /** For debuging **/

  /** Debug publishers **/
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_point_pub;

  /** Debug functions **/
  void plotTrajectory();
  void plotTrajectoryThread();
  void plotRefTrajPoint();
};

/** Auxiliar Functions **/

void generateDynamicPoint(
    const as2_msgs::msg::PoseWithID &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);

#endif  // TRAJECTORY_GENERATOR_HPP_
