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
 *  \file       path_planner_behavior.hpp
 *  \brief      path_planner_behavior header file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_
#define AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pluginlib/class_loader.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_srvs/srv/trigger.hpp"
#include "as2_core/synchronous_service_client.hpp"

#include "as2_msgs/action/follow_path.hpp"
#include "as2_behaviors_path_planning/path_planner_plugin_base.hpp"

class PathPlannerBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>
{
public:
  explicit PathPlannerBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathPlannerBehavior() {}

private:
  // Behavior action parameters
  as2_msgs::msg::YawMode yaw_mode_;
  as2_msgs::action::NavigateToPoint::Goal goal_;
  as2_msgs::action::NavigateToPoint::Feedback feedback_;
  as2_msgs::action::NavigateToPoint::Result result_;

  // Planner variables
  bool enable_visualization_ = false;
  bool use_path_optimizer_ = false;
  geometry_msgs::msg::PoseStamped drone_pose_;
  double safety_distance_ = 1.0;  // aprox drone size [m]
  std::vector<geometry_msgs::msg::Point> path_;

  bool navigation_aborted_ = false;
  std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> follow_path_feedback_;
  bool follow_path_rejected_ = false;
  bool follow_path_succeeded_ = false;

private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> & goal,
    std::shared_ptr<as2_msgs::action::NavigateToPoint::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::NavigateToPoint::Result> & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  /** Path Planner Behavior plugin **/
  std::shared_ptr<pluginlib::ClassLoader<as2_behaviors_path_planning::PluginBase>> loader_;
  std::shared_ptr<as2_behaviors_path_planning::PluginBase> path_planner_plugin_;

  /* Other ROS 2 interfaces */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
  // TODO(pariaspe): where to place visualization publisher. In plugin or here?
  // For path returned by plugin probably here
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  rclcpp_action::Client<as2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_pause_client_ =
    nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_resume_client_ =
    nullptr;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  void drone_pose_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // FollowPath Action Client
  void follow_path_response_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr & goal_handle);
  void follow_path_feedback_cbk(
    rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr goal_handle,
    const std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback);
  void follow_path_result_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::WrappedResult & result);
};

#endif  // AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_
