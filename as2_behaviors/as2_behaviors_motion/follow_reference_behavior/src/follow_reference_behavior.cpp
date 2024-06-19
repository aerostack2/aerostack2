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
 * @file follow_reference_behavior.hpp
 *
 * follow_reference_behavior header file
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
    this->declare_parameter<double>("follow_reference_max_speed_x");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <follow_reference_max_speed_x> not defined or "
      "malformed: %s",
      e.what());
    this->~FollowReferenceBehavior();
  }

  try {
    this->declare_parameter<double>("follow_reference_max_speed_y");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <follow_reference_max_speed_y> not defined or "
      "malformed: %s",
      e.what());
    this->~FollowReferenceBehavior();
  }

  try {
    this->declare_parameter<double>("follow_reference_max_speed_z");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <follow_reference_max_speed_z> not defined or "
      "malformed: %s",
      e.what());
    this->~FollowReferenceBehavior();
  }

  try {
    this->declare_parameter<double>("tf_timeout_threshold");
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Launch argument <tf_timeout_threshold> not defined or "
      "malformed: %s",
      e.what());
    this->~FollowReferenceBehavior();
  }

  position_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::PositionMotion>(this);

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  tf_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(this->get_parameter("tf_timeout_threshold").as_double()));

  hover_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(this);

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
    std::bind(&FollowReferenceBehavior::state_callback, this, std::placeholders::_1));

  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info, as2_names::topics::platform::qos,
    std::bind(&FollowReferenceBehavior::platform_info_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "FollowReference Behavior ready!");
}

FollowReferenceBehavior::~FollowReferenceBehavior() {}

void FollowReferenceBehavior::state_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg)
{
  actual_twist = *_twist_msg;
  localization_flag_ = true;
  if (getState()) {
    computeYaw(
      goal_.yaw.mode, goal_.target_pose.point, actual_pose_.pose.position,
      goal_.yaw.angle);
  }
}

void FollowReferenceBehavior::platform_info_callback(
  const as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  platform_state_ = msg->status.state;
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

  if (!tf_handler_->tryConvert(
      new_goal.target_pose, goal->target_pose.header.frame_id,
      tf_timeout))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "FollowReferenceBehavior: can not get target position in the desired frame");
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
  goal_ = *goal;

  if (!process_goal(goal, goal_)) {
    return false;
  }

  if (!getState()) {
    return false;
  }

  if (!checkGoal(goal_)) {
    return false;
  }

  if (!computeYaw(
      goal_.yaw.mode, goal_.target_pose.point, actual_pose_.pose.position,
      goal_.yaw.angle))
  {
    return false;
  }

  return true;
}

bool FollowReferenceBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
{
  goal_ = *goal;

  if (!getState()) {
    return false;
  }

  if (!computeYaw(
      goal_.yaw.mode, goal_.target_pose.point, actual_pose_.pose.position,
      goal_.yaw.angle))
  {
    return false;
  }

  return true;
}

bool FollowReferenceBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "FollowReference Stopped");
  // Leave the drone in the last position
  goal_.target_pose.header.frame_id = "";
  sendHover();
  return true;
}

bool FollowReferenceBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "FollowReference Paused");
  sendHover();

  return true;
}

bool FollowReferenceBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "FollowReference Resumed");
  return true;
}

as2_behavior::ExecutionStatus FollowReferenceBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> & goal,
  std::shared_ptr<as2_msgs::action::FollowReference::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::FollowReference::Result> & result_msg)
{
  feedback_msg = std::make_shared<as2_msgs::action::FollowReference::Feedback>(feedback_);
  result_msg = std::make_shared<as2_msgs::action::FollowReference::Result>(result_);
  if (!position_motion_handler_->sendPositionCommandWithYawAngle(
      goal_.target_pose.header.frame_id, goal_.target_pose.point.x, goal_.target_pose.point.y,
      goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed_x, goal_.max_speed_y,
      goal_.max_speed_z))
  {
    RCLCPP_ERROR(this->get_logger(), "FOLLOW REFERENCE: Error sending position command");
    result_.follow_reference_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }
  result_.follow_reference_success = true;
  return as2_behavior::ExecutionStatus::RUNNING;
}

void FollowReferenceBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  return;
}

inline void FollowReferenceBehavior::sendHover()
{
  hover_motion_handler_->sendHover();
  return;
}

inline float FollowReferenceBehavior::getActualYaw()
{
  return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
}

bool FollowReferenceBehavior::getState()
{
  if (goal_.target_pose.header.frame_id != "") {
    try {
      auto [pose_msg, twist_msg] =
        tf_handler_->getState(
        actual_twist, "earth", goal_.target_pose.header.frame_id,
        base_link_frame_id_, tf_timeout);
      actual_pose_ = pose_msg;
      feedback_.actual_speed = Eigen::Vector3d(
        twist_msg.twist.linear.x, twist_msg.twist.linear.y,
        twist_msg.twist.linear.z)
        .norm();

      feedback_.actual_distance_to_goal =
        (Eigen::Vector3d(
          actual_pose_.pose.position.x, actual_pose_.pose.position.y,
          actual_pose_.pose.position.z) -
        Eigen::Vector3d(
          goal_.target_pose.point.x, goal_.target_pose.point.y,
          goal_.target_pose.point.z))
        .norm();
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
    return true;
  }
  return false;
}

bool FollowReferenceBehavior::computeYaw(
  const uint8_t yaw_mode,
  const geometry_msgs::msg::Point & target,
  const geometry_msgs::msg::Point & actual,
  float & yaw)
{
  switch (yaw_mode) {
    case as2_msgs::msg::YawMode::PATH_FACING: {
        Eigen::Vector2d diff(target.x - actual.x, target.y - actual.y);
        if (diff.norm() < 0.1) {
          RCLCPP_WARN(
            this->get_logger(),
            "Goal is too close to the current position in the plane, setting yaw_mode to "
            "KEEP_YAW");
          yaw = getActualYaw();
        } else {
          yaw = as2::frame::getVector2DAngle(diff.x(), diff.y());
        }
      } break;
    case as2_msgs::msg::YawMode::YAW_TO_FRAME:
      yaw = std::atan2(actual.y, actual.x);
      yaw = yaw + (yaw > 0 ? -M_PI : M_PI);
      break;
    case as2_msgs::msg::YawMode::FIXED_YAW:
      RCLCPP_INFO(this->get_logger(), "Yaw mode FIXED_YAW");
      break;
    case as2_msgs::msg::YawMode::KEEP_YAW:
      RCLCPP_INFO(this->get_logger(), "Yaw mode KEEP_YAW");
      yaw = getActualYaw();
      break;
    case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
      RCLCPP_INFO(this->get_logger(), "Yaw mode YAW_FROM_TOPIC, not supported");
      return false;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Yaw mode %d not supported", yaw_mode);
      return false;
      break;
  }
  return true;
}

bool FollowReferenceBehavior::checkGoal(as2_msgs::action::FollowReference::Goal & _goal)
{
  if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
    RCLCPP_ERROR(this->get_logger(), "Behavior reject, platform is not flying");
    return false;
  }

  if (!localization_flag_) {
    RCLCPP_ERROR(this->get_logger(), "Behavior reject, there is no localization");
    return false;
  }

  return true;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(FollowReferenceBehavior)
