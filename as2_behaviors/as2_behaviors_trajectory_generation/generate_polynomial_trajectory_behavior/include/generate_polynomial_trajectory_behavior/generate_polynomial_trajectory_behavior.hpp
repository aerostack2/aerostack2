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
 * @file generate_polynomial_trajectory_behavior.hpp
 *
 * @brief Class definition for the GeneratePolynomialTrajectoryBehavior class.
 *
 * @author Miguel Fernández Cortizas
 *         Pedro Arias Pérez
 *         David Pérez Saura
 *         Rafael Pérez Seguí
 */

#ifndef GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_
#define GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <string>
#include <memory>

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

#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class DynamicPolynomialTrajectoryGenerator
  : public as2_behavior::BehaviorServer<
    as2_msgs::action::GeneratePolynomialTrajectory>
{
public:
  DynamicPolynomialTrajectoryGenerator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamicPolynomialTrajectoryGenerator() {}

private:
  /** Subscriptions **/
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;

  // For faster waypoint modified
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr
    mod_waypoint_sub_;

  /** Timer**/
  rclcpp::TimerBase::SharedPtr timer_update_frame_;
  double frequency_update_frame_ = 0.0;

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
  std::string map_frame_id_;
  int sampling_n_ = 1;
  double sampling_dt_ = 0.0;
  int path_lenght_ = 0;
  float yaw_threshold_ = 0;
  float transform_threshold_ = 1.0;
  double yaw_speed_threshold_ = 2.0;
  double wp_close_threshold_ = 0.0;

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
  geometry_msgs::msg::TransformStamped current_map_to_odom_transform_;
  geometry_msgs::msg::TransformStamped last_map_to_odom_transform_;
  double current_yaw_;

  // Command
  as2_msgs::msg::TrajectorySetpoints trajectory_command_;

  // Trajectory generator
  rclcpp::Duration eval_time_ = rclcpp::Duration(0, 0);
  rclcpp::Duration eval_time_yaw_ = rclcpp::Duration(0, 0);
  rclcpp::Time time_zero_;
  rclcpp::Time time_zero_yaw_;
  bool first_run_ = false;
  bool has_odom_ = false;
  dynamic_traj_generator::DynamicWaypoint::Deque waypoints_to_set_;
  std::optional<rclcpp::Time> time_debug_;

  // Debug
  bool enable_debug_ = false;
  std::thread plot_thread_;

private:
  /** As2 Behavior methods **/
  bool on_activate(
    std::shared_ptr<
      const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
    goal) override;

  bool on_modify(
    std::shared_ptr<
      const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
    goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<
      const as2_msgs::action::GeneratePolynomialTrajectory::Goal> & goal,
    std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Feedback>
    & feedback_msg,
    std::shared_ptr<as2_msgs::action::GeneratePolynomialTrajectory::Result>
    & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  /**
  * @brief Callback to check the errors between frames and update the frame offset
  */
  void timerUpdateFrameCallback();

  /** Topic Callbacks **/
  void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void yawCallback(const std_msgs::msg::Float32::SharedPtr _msg);

  // For faster waypoint modified
  void modifyWaypointCallback(
    const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr _msg);

  /** Trajectory generator functions */
  void setup();
  bool goalToDynamicWaypoint(
    std::shared_ptr<
      const as2_msgs::action::GeneratePolynomialTrajectory::Goal>
    goal,
    dynamic_traj_generator::DynamicWaypoint::Deque & waypoints);
  bool evaluateTrajectory(double eval_time);
  bool evaluateSetpoint(
    double eval_time,
    as2_msgs::msg::TrajectoryPoint & trajectory_command,
    bool current_setpoint = true);

  /**
   * @brief update the trajectory waypoint and waypoint_to_set_queue with the frame offset
   * @param goal the goal of the action
   * @return bool Return false if the transform between the map and the desired frame is not available and true otherwise
   */
  bool updateFrame(
    const as2_msgs::action::GeneratePolynomialTrajectory::Goal &
    goal);

  double computeYawAnglePathFacing(double vx, double vy);

  /**
  * @brief Compute the Yaw angle to face the next reference point
  * @return double Current yaw angle if the distance to the next reference point is less than the yaw_threshold_ or the angle to face the next reference point otherwise
  */
  double computeYawFaceReference();

  /**
* @brief Compute the error frames between the map and the desired frame
* @return bool Return true if the frame offset is bigger than the transform_threshold_ and false otherwise
*/
  bool computeErrorFrames();

  /** For debuging **/

  /** Debug publishers **/
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_ref_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_end_ref_point_pub_;

  /** Debug functions **/
  void plotTrajectory();
  void plotTrajectoryThread();
  void plotRefTrajPoint();
};

/** Auxiliar Functions **/

void generateDynamicPoint(
  const as2_msgs::msg::PoseStampedWithID & msg,
  dynamic_traj_generator::DynamicWaypoint & dynamic_point);

#endif  // GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_
