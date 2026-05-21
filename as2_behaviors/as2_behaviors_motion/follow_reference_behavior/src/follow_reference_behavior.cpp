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
 * @file follow_reference_behavior.cpp
 *
 * Pluginlib-based wrapper for follow_reference plugins.
 *
 * @authors Rafael Perez-Segui
 *          Pedro Arias Pérez
 *          Javier Melero Deza
 */

#include "follow_reference_behavior/follow_reference_behavior.hpp"

FollowReferenceBehavior::FollowReferenceBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::FollowReference>(
    as2_names::actions::behaviors::followreference,
    options)
{
  try {
    this->declare_parameter<std::string>("plugin_name");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <plugin_name> not defined or malformed: %s", e.what());
    this->~FollowReferenceBehavior();
  }

  loader_ = std::make_shared<
    pluginlib::ClassLoader<follow_reference_base::FollowReferenceBase>>(
    "as2_behaviors_motion",
    "follow_reference_base::FollowReferenceBase");

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  try {
    std::string plugin_name = this->get_parameter("plugin_name").as_string();
    plugin_name += "::Plugin";
    follow_reference_plugin_ = loader_->createSharedInstance(plugin_name);

    // The base reads its own parameters (max speeds) from the node inside
    // initialize(), so the wrapper does not pass them by construction.
    follow_reference_plugin_->initialize(this, tf_handler_);

    RCLCPP_INFO(
      this->get_logger(),
      "FOLLOW REFERENCE BEHAVIOR PLUGIN LOADED: %s", plugin_name.c_str());
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The plugin failed to load for some reason. Error: %s\n", ex.what());
    this->~FollowReferenceBehavior();
  }

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(
      &FollowReferenceBehavior::platform_info_callback, this,
      std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist,
    as2_names::topics::self_localization::qos,
    std::bind(
      &FollowReferenceBehavior::state_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "FollowReference Behavior ready!");
}

FollowReferenceBehavior::~FollowReferenceBehavior() {}

void FollowReferenceBehavior::state_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  try {
    auto [pose_msg, twist_msg] =
      tf_handler_->getState(*_twist_msg, "earth", "earth", base_link_frame_id_);
    follow_reference_plugin_->state_callback(pose_msg, twist_msg);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void FollowReferenceBehavior::platform_info_callback(
  const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  follow_reference_plugin_->platform_info_callback(msg);
  return;
}

bool FollowReferenceBehavior::process_goal(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal,
  as2_msgs::action::FollowReference::Goal & new_goal)
{
  if (goal->target_pose.header.frame_id == "") {
    RCLCPP_ERROR(this->get_logger(), "Target pose frame_id is empty");
    return false;
  }

  // Resolve the target frame to its current representation (no-op when the
  // frame is already valid, but validates the lookup).
  if (!tf_handler_->tryConvert(
      new_goal.target_pose, goal->target_pose.header.frame_id))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "FollowReferenceBehavior: can not get target position in the desired "
      "frame");
    return false;
  }

  new_goal.max_speed_x = (goal->max_speed_x != 0.0f) ?
    goal->max_speed_x :
    this->get_parameter("follow_reference_max_speed_x").as_double();
  new_goal.max_speed_y = (goal->max_speed_y != 0.0f) ?
    goal->max_speed_y :
    this->get_parameter("follow_reference_max_speed_y").as_double();
  new_goal.max_speed_z = (goal->max_speed_z != 0.0f) ?
    goal->max_speed_z :
    this->get_parameter("follow_reference_max_speed_z").as_double();

  return true;
}

bool FollowReferenceBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
{
  as2_msgs::action::FollowReference::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return follow_reference_plugin_->on_activate(
    std::make_shared<const as2_msgs::action::FollowReference::Goal>(new_goal));
}

bool FollowReferenceBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
{
  as2_msgs::action::FollowReference::Goal new_goal = *goal;
  if (!process_goal(goal, new_goal)) {
    return false;
  }
  return follow_reference_plugin_->on_modify(
    std::make_shared<const as2_msgs::action::FollowReference::Goal>(new_goal));
}

bool FollowReferenceBehavior::on_deactivate(
  const std::shared_ptr<std::string> & message)
{
  return follow_reference_plugin_->on_deactivate(message);
}

bool FollowReferenceBehavior::on_pause(
  const std::shared_ptr<std::string> & message)
{
  return follow_reference_plugin_->on_pause(message);
}

bool FollowReferenceBehavior::on_resume(
  const std::shared_ptr<std::string> & message)
{
  return follow_reference_plugin_->on_resume(message);
}

as2_behavior::ExecutionStatus FollowReferenceBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> & goal,
  std::shared_ptr<as2_msgs::action::FollowReference::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::FollowReference::Result> & result_msg)
{
  return follow_reference_plugin_->on_run(goal, feedback_msg, result_msg);
}

void FollowReferenceBehavior::on_execution_end(
  const as2_behavior::ExecutionStatus & state)
{
  return follow_reference_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(FollowReferenceBehavior)
