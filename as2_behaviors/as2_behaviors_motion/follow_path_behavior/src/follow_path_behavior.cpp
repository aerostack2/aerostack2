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
 * @file follow_path_behavior.cpp
 *
 * follow_path_behavior file
 *
 * @authors Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#include "follow_path_behavior/follow_path_behavior.hpp"

FollowPathBehavior::FollowPathBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::FollowPath>(
    as2_names::actions::behaviors::followpath,
    options)
{
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~FollowPathBehavior();
  }
  try {
    this->declare_parameter<double>("follow_path_speed");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <follow_path_speed> not defined or "
      "malformed: %s",
      e.what());
    this->~FollowPathBehavior();
  }
  try {
    this->declare_parameter<double>("follow_path_threshold");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <follow_path_threshold> not defined or malformed: %s", e.what());
    this->~FollowPathBehavior();
  }

  loader_ = std::make_shared<pluginlib::ClassLoader<follow_path_base::FollowPathBase>>(
    "as2_behaviors_motion", "follow_path_base::FollowPathBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    follow_path_plugin_ = loader_->createSharedInstance(plugin_name);

    follow_path_base::follow_path_plugin_params params;
    params.follow_path_speed = this->get_parameter("follow_path_speed").as_double();
    params.follow_path_threshold = this->get_parameter("follow_path_threshold").as_double();

    follow_path_plugin_->initialize(this, tf_handler_, params);

    RCLCPP_INFO(this->get_logger(), "FOLLOW PATH PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
      ex.what());
    this->~FollowPathBehavior();
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&FollowPathBehavior::platform_info_callback, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&FollowPathBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "FollowPath Behavior ready!");
}

FollowPathBehavior::~FollowPathBehavior() {}

void FollowPathBehavior::state_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    auto [pose_msg, twist_msg] =
      tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
    follow_path_plugin_->state_callback(pose_msg, twist_msg);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void FollowPathBehavior::platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  follow_path_plugin_->platform_info_callback(msg);
  return;
}

bool FollowPathBehavior::process_goal(
  std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal,
  as2_msgs::action::FollowPath::Goal & new_goal)
{
  if (goal->header.frame_id == "") {
    RCLCPP_ERROR(this->get_logger(), "Path frame_id is empty");
    return false;
  }

  if (goal->path.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Path is empty");
    return false;
  }

  if (goal->header.frame_id != "earth") {
    std::vector<as2_msgs::msg::PoseWithID> path_converted;
    path_converted.reserve(goal->path.size());

    geometry_msgs::msg::PoseStamped pose_msg;

    for (as2_msgs::msg::PoseWithID waypoint : goal->path) {
      pose_msg.pose = waypoint.pose;
      pose_msg.header = goal->header;
      if (!tf_handler_->tryConvert(pose_msg, "earth")) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath: can not get waypoint in earth frame");
        return false;
      }
      waypoint.pose = pose_msg.pose;
      path_converted.push_back(waypoint);
    }
    new_goal.header.frame_id = "earth";
    new_goal.path = path_converted;
  }

  new_goal.max_speed = (goal->max_speed != 0.0f) ?
    goal->max_speed :
    this->get_parameter("follow_path_speed").as_double();

  return true;
}

bool FollowPathBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal)
{
  as2_msgs::action::FollowPath::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return follow_path_plugin_->on_activate(
    std::make_shared<const as2_msgs::action::FollowPath::Goal>(new_goal));
}

bool FollowPathBehavior::on_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal)
{
  as2_msgs::action::FollowPath::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return follow_path_plugin_->on_modify(
    std::make_shared<const as2_msgs::action::FollowPath::Goal>(new_goal));
}

bool FollowPathBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return follow_path_plugin_->on_deactivate(message);
}

bool FollowPathBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return follow_path_plugin_->on_pause(message);
}

bool FollowPathBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return follow_path_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus FollowPathBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> & goal,
  std::shared_ptr<as2_msgs::action::FollowPath::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::FollowPath::Result> & result_msg)
{
  return follow_path_plugin_->on_run(goal, feedback_msg, result_msg);
}

void FollowPathBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  return follow_path_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(FollowPathBehavior)
