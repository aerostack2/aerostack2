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
 *  \file       path_planner_behavior.cpp
 *  \brief      path_planner_behavior implementation file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#include "as2_behaviors_path_planning/path_planner_behavior.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

PathPlannerBehavior::PathPlannerBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>("path_planner", options)
{
  this->declare_parameter("enable_visualization", false);
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  this->declare_parameter("use_path_optimizer", false);
  use_path_optimizer_ = this->get_parameter("use_path_optimizer").as_bool();

  // TODO(pariaspe): move to action_goal
  this->declare_parameter("safety_distance", 1.0);  // aprox drone size [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Loading plugin
  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_behaviors_path_planning::PluginBase>>(
    "as2_behaviors_path_planning", "as2_behaviors_path_planning::PluginBase");
  try {
    path_planner_plugin_ = loader_->createSharedInstance("a_star::Plugin");
    path_planner_plugin_->initialize(this, tf_buffer_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load. Error: %s", ex.what());
    this->~PathPlannerBehavior();
  }

  // TODO(pariaspe): use as2_names
  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", as2_names::topics::self_localization::qos,
    std::bind(&PathPlannerBehavior::drone_pose_cbk, this, std::placeholders::_1));

  if (enable_visualization_) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
  }

  follow_path_client_ = rclcpp_action::create_client<as2_msgs::action::FollowPath>(
    this, as2_names::actions::behaviors::followpath);

  // TODO(pariaspe): modify follow_path interfaces
  follow_path_pause_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::followpath) + "/_behavior/pause", this);

  follow_path_resume_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::followpath) + "/_behavior/resume", this);
}

bool PathPlannerBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
  bool ret = path_planner_plugin_->on_activate(drone_pose_, *goal);
  if (!ret) {
    return false;
  }


  // Call Follow Path behavior
  if (!this->follow_path_client_->wait_for_action_server(
      std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Follow Path Action server not available after waiting. Aborting navigation.");
    return false;
  }

  path_ = path_planner_plugin_->path_;

  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw = goal->yaw;
  goal_msg.max_speed = goal->navigation_speed;
  int i = 0;
  for (auto & p : path_) {
    as2_msgs::msg::PoseWithID pid = as2_msgs::msg::PoseWithID();
    pid.id = std::to_string(i);
    pid.pose.position = p;
    pid.pose.position.z = 1.0;
    goal_msg.path.push_back(pid);
    i++;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath behavior");

  auto send_goal_options = rclcpp_action::Client<as2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &PathPlannerBehavior::follow_path_response_cbk, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &PathPlannerBehavior::follow_path_feedback_cbk, this, std::placeholders::_1,
    std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&PathPlannerBehavior::follow_path_result_cbk, this, std::placeholders::_1);
  follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  return true;
}

bool PathPlannerBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
  RCLCPP_WARN(this->get_logger(), "Modify not implemented");
  return false;
}

bool PathPlannerBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // Cancel only the goal started from navigation. Behaviors only accepts
  // one goal simultaneously, don't have to worry about
  follow_path_client_->async_cancel_all_goals();
  navigation_aborted_ = true;
  return true;
}

bool PathPlannerBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool paused = follow_path_pause_client_->sendRequest(req, res, 3);
  return paused && res.success;
}

bool PathPlannerBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool resumed = follow_path_resume_client_->sendRequest(req, res, 3);
  return resumed && res.success;
}

void PathPlannerBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  std::string state_str;
  switch (state) {
    case as2_behavior::ExecutionStatus::SUCCESS:
      state_str = "SUCCEEDED";
      break;
    case as2_behavior::ExecutionStatus::ABORTED:
      state_str = "ABORTED";
      break;
    case as2_behavior::ExecutionStatus::RUNNING:
      state_str = "RUNNING";
      break;
    case as2_behavior::ExecutionStatus::FAILURE:
      state_str = "FAILED";
      break;
    default:
      state_str = "UNKNOWN";
      break;
  }
  RCLCPP_INFO(this->get_logger(), "Execution ended with state: %s", state_str.c_str());
}

as2_behavior::ExecutionStatus PathPlannerBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> & goal,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Result> & result_msg)
{
  if (follow_path_rejected_ || navigation_aborted_) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // TODO(pariaspe): current feedback is just a template
  if (!follow_path_feedback_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for feedback from FollowPath behavior");
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  feedback_msg->current_pose = drone_pose_;
  feedback_msg->current_speed.twist.linear.x = follow_path_feedback_->actual_speed;
  feedback_msg->distance_remaining = follow_path_feedback_->actual_distance_to_next_waypoint;
  // feedback_msg->estimated_time_remaining = -1;
  // feedback_msg->navigation_time = -1;

  if (follow_path_succeeded_) {
    result_msg->success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PathPlannerBehavior::drone_pose_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  drone_pose_ = *(msg);
}

void PathPlannerBehavior::follow_path_response_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowPath was rejected by behavior server. Aborting navigation.");
    follow_path_rejected_ = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "FollowPath accepted, flying to point.");
  }
}

void PathPlannerBehavior::follow_path_feedback_cbk(
  rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr goal_handle,
  const std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback)
{
  if (navigation_aborted_) {
    // cancel follow path too
    follow_path_client_->async_cancel_goal(goal_handle);
    return;
  }

  follow_path_feedback_ = feedback;
}

void PathPlannerBehavior::follow_path_result_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "FollowPath was aborted. Aborting navigation.");
      navigation_aborted_ = true;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "FollowPath was canceled. Cancelling navigation");
      navigation_aborted_ = true;
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code from FollowPath. Aborting navigation.");
      navigation_aborted_ = true;
      return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Follow Path succeeded. Goal point reached. Navigation succeeded.");
  follow_path_succeeded_ = true;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlannerBehavior)
