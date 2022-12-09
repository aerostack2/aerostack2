/*!*******************************************************************************************
 *  \file       goto_base.hpp
 *  \brief      Base class for goto plugins header
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

#ifndef GOTO_BASE_HPP
#define GOTO_BASE_HPP

#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "motion_reference_handlers/hover_motion.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace goto_base {

struct goto_plugin_params {
  double default_goto_max_speed = 0.0;
  double goto_threshold         = 0.0;
};

class GotoBase {
public:
  using GoalHandleGoto = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

  GotoBase(){};
  virtual ~GotoBase(){};

  void initialize(as2::Node *node_ptr,
                  const std::shared_ptr<as2::tf::TfHandler> tf_handler,
                  goto_plugin_params &params) {
    node_ptr_             = node_ptr;
    params_               = params;
    hover_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    this->ownInit();
  }

  virtual void state_callback(geometry_msgs::msg::PoseStamped &pose_msg,
                              geometry_msgs::msg::TwistStamped &twist_msg) {
    actual_pose_ = pose_msg;

    feedback_.actual_speed = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                                             twist_msg.twist.linear.z)
                                 .norm();

    feedback_.actual_distance_to_goal =
        (Eigen::Vector3d(actual_pose_.pose.position.x, actual_pose_.pose.position.y,
                         actual_pose_.pose.position.z) -
         Eigen::Vector3d(goal_.target_pose.point.x, goal_.target_pose.point.y,
                         goal_.target_pose.point.z))
            .norm();

    distance_measured_ = true;
    return;
  }

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
    platform_state_ = msg->status.state;
    return;
  }

  virtual bool on_deactivate(const std::shared_ptr<std::string> &message) = 0;
  virtual bool on_pause(const std::shared_ptr<std::string> &message)      = 0;
  virtual bool on_resume(const std::shared_ptr<std::string> &message)     = 0;

  virtual bool on_activate(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) {
    as2_msgs::action::GoToWaypoint::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_activate(std::make_shared<as2_msgs::action::GoToWaypoint::Goal>(goal_candidate))) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  virtual bool on_modify(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) {
    as2_msgs::action::GoToWaypoint::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_modify(std::make_shared<as2_msgs::action::GoToWaypoint::Goal>(goal_candidate))) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  virtual as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal,
      std::shared_ptr<as2_msgs::action::GoToWaypoint::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::GoToWaypoint::Result> &result_msg) {
    as2_behavior::ExecutionStatus status = own_run();

    feedback_msg = std::make_shared<as2_msgs::action::GoToWaypoint::Feedback>(feedback_);
    result_msg   = std::make_shared<as2_msgs::action::GoToWaypoint::Result>(result_);
    return status;
  }

  virtual void on_excution_end(const as2_behavior::ExecutionStatus &state) {
    distance_measured_ = false;
    own_execution_end(state);
    return;
  }

protected:
  virtual void ownInit(){};

  virtual bool own_activate(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) {
    return true;
  }

  virtual as2_behavior::ExecutionStatus own_run() = 0;

  virtual bool own_modify(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) {
    return true;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus &state) = 0;

  inline void sendHover() {
    hover_motion_handler_->sendHover();
    return;
  };

  inline float getActualYaw() {
    return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
  };

protected:
  as2::Node *node_ptr_;

  as2_msgs::action::GoToWaypoint::Goal goal_;
  as2_msgs::action::GoToWaypoint::Feedback feedback_;
  as2_msgs::action::GoToWaypoint::Result result_;

  int platform_state_;
  goto_plugin_params params_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  bool distance_measured_;

  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_ = nullptr;

private:
  bool processGoal(as2_msgs::action::GoToWaypoint::Goal &_goal) {
    if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, platform is not flying");
      return false;
    }

    if (!distance_measured_) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, there is no localization");
      return false;
    }

    switch (_goal.yaw.mode) {
      case as2_msgs::msg::YawMode::PATH_FACING: {
        Eigen::Vector2d diff(_goal.target_pose.point.x - actual_pose_.pose.position.x,
                             _goal.target_pose.point.y - actual_pose_.pose.position.y);
        if (diff.norm() < 0.1) {
          RCLCPP_WARN(node_ptr_->get_logger(),
                      "Goal is too close to the current position in the plane, setting yaw_mode to "
                      "KEEP_YAW");
          _goal.yaw.angle = getActualYaw();
        } else {
          _goal.yaw.angle = as2::frame::getVector2DAngle(diff.x(), diff.y());
        }
        break;
      }
      case as2_msgs::msg::YawMode::FIXED_YAW:
        break;
      case as2_msgs::msg::YawMode::KEEP_YAW:
        _goal.yaw.angle = getActualYaw();
        break;
      case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
      default:
        RCLCPP_ERROR(node_ptr_->get_logger(), "Yaw mode not supported");
        return false;
        break;
    }
    return true;
  }

};  // class GotoBase
}  // namespace goto_base
#endif  // GOTO_BASE_HPP
