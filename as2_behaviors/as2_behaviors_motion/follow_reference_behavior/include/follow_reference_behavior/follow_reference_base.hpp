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
 * @file follow_reference_base.hpp
 *
 * Base class for follow_reference plugins.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef FOLLOW_REFERENCE_BEHAVIOR__FOLLOW_REFERENCE_BASE_HPP_
#define FOLLOW_REFERENCE_BEHAVIOR__FOLLOW_REFERENCE_BASE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/action/follow_reference.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"

namespace follow_reference_base
{

struct follow_reference_plugin_params
{
  double follow_reference_max_speed_x = 0.0;
  double follow_reference_max_speed_y = 0.0;
  double follow_reference_max_speed_z = 0.0;
};

class FollowReferenceBase
{
public:
  using GoalHandleFollowReference =
    rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowReference>;

  FollowReferenceBase() {}
  virtual ~FollowReferenceBase() {}

  void initialize(
    as2::Node * node_ptr,
    std::shared_ptr<as2::tf::TfHandler> tf_handler)
  {
    node_ptr_ = node_ptr;
    tf_handler_ = tf_handler;

    // Read max speed parameters from the node. The wrapper neither declares
    // nor passes them; we declare on demand here so the base owns the
    // contract with the node. Values come from the launch-loaded config.
    params_.follow_reference_max_speed_x =
      declareAndGetDouble("follow_reference_max_speed_x");
    params_.follow_reference_max_speed_y =
      declareAndGetDouble("follow_reference_max_speed_y");
    params_.follow_reference_max_speed_z =
      declareAndGetDouble("follow_reference_max_speed_z");

    hover_motion_handler_ =
      std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    this->ownInit();
  }

  void state_callback(
    geometry_msgs::msg::PoseStamped & pose_msg,
    geometry_msgs::msg::TwistStamped & twist_msg)
  {
    actual_pose_ = pose_msg;

    feedback_.actual_speed = Eigen::Vector3d(
      twist_msg.twist.linear.x, twist_msg.twist.linear.y,
      twist_msg.twist.linear.z)
      .norm();

    // Compute actual_distance_to_goal in a consistent frame. actual_pose_
    // is in "earth"; goal_.target_pose may be in any frame
    if (!goal_.target_pose.header.frame_id.empty()) {
      geometry_msgs::msg::PointStamped target = goal_.target_pose;
      target.header.stamp = node_ptr_->now();
      try {
        geometry_msgs::msg::PointStamped target_in_earth =
          tf_handler_->convert(target, "earth");
        feedback_.actual_distance_to_goal =
          (Eigen::Vector3d(
            actual_pose_.pose.position.x, actual_pose_.pose.position.y,
            actual_pose_.pose.position.z) -
          Eigen::Vector3d(
            target_in_earth.point.x, target_in_earth.point.y,
            target_in_earth.point.z))
          .norm();
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
          "FollowReference: could not transform target to earth for "
          "feedback distance: %s",
          ex.what());
      }
    }

    localization_flag_ = true;
    return;
  }

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg)
  {
    platform_state_ = msg->status.state;
    return;
  }

  bool on_activate(
    std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
  {
    as2_msgs::action::FollowReference::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) {return false;}

    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  bool on_modify(
    std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal)
  {
    as2_msgs::action::FollowReference::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) {return false;}

    if (own_modify(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  inline bool on_deactivate(const std::shared_ptr<std::string> & message)
  {
    return own_deactivate(message);
  }

  inline bool on_pause(const std::shared_ptr<std::string> & message)
  {
    return own_pause(message);
  }

  inline bool on_resume(const std::shared_ptr<std::string> & message)
  {
    return own_resume(message);
  }

  void on_execution_end(const as2_behavior::ExecutionStatus & state)
  {
    localization_flag_ = false;
    own_execution_end(state);
    return;
  }

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::FollowReference::Goal> goal,
    std::shared_ptr<as2_msgs::action::FollowReference::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::FollowReference::Result> & result_msg)
  {
    as2_behavior::ExecutionStatus status = own_run();

    feedback_msg =
      std::make_shared<as2_msgs::action::FollowReference::Feedback>(feedback_);
    result_msg =
      std::make_shared<as2_msgs::action::FollowReference::Result>(result_);
    return status;
  }

private:
  double declareAndGetDouble(const std::string & param_name)
  {
    if (!node_ptr_->has_parameter(param_name)) {
      try {
        node_ptr_->declare_parameter<double>(param_name);
      } catch (const rclcpp::ParameterTypeException & e) {
        RCLCPP_FATAL(
          node_ptr_->get_logger(),
          "Parameter <%s> not defined or malformed: %s",
          param_name.c_str(), e.what());
        throw;
      }
    }
    return node_ptr_->get_parameter(param_name).as_double();
  }

  bool processGoal(as2_msgs::action::FollowReference::Goal & _goal)
  {
    if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Behavior reject, platform is not flying");
      return false;
    }

    if (!localization_flag_) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Behavior reject, there is no localization");
      return false;
    }

    return true;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion>
  hover_motion_handler_ = nullptr;

  /* Interface with plugin */

protected:
  virtual void ownInit() {}

  virtual bool own_activate(
    as2_msgs::action::FollowReference::Goal & goal) = 0;

  virtual bool own_modify(as2_msgs::action::FollowReference::Goal & goal)
  {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Follow reference can not be modified, not implemented");
    return false;
  }

  virtual bool own_deactivate(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> & message)
  {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Follow reference can not be paused, not implemented, try to cancel it");
    return false;
  }

  virtual bool own_resume(const std::shared_ptr<std::string> & message)
  {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Follow reference can not be resumed, not implemented");
    return false;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus & state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

  inline void sendHover()
  {
    hover_motion_handler_->sendHover();
    return;
  }

protected:
  as2::Node * node_ptr_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_ = nullptr;

  as2_msgs::action::FollowReference::Goal goal_;
  as2_msgs::action::FollowReference::Feedback feedback_;
  as2_msgs::action::FollowReference::Result result_;

  int platform_state_;
  follow_reference_plugin_params params_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  bool localization_flag_;
};  // class FollowReferenceBase
}  // namespace follow_reference_base
#endif  // FOLLOW_REFERENCE_BEHAVIOR__FOLLOW_REFERENCE_BASE_HPP_
