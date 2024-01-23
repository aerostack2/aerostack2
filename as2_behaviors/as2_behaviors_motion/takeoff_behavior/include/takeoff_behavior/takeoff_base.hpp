/*!*******************************************************************************************
 *  \file       takeoff_base.hpp
 *  \brief      Base class for takeoff plugins
 *  \authors    Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
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

#ifndef TAKEOFF_BASE_HPP
#define TAKEOFF_BASE_HPP

#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/action/takeoff.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace takeoff_base {

struct takeoff_plugin_params {
  double takeoff_height       = 0.0;
  double takeoff_speed        = 0.0;
  double takeoff_threshold    = 0.0;
  double tf_timeout_threshold = 0.0;
};

class TakeoffBase {
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::Takeoff>;

  TakeoffBase(){};
  virtual ~TakeoffBase(){};

  void initialize(as2::Node *node_ptr,
                  std::shared_ptr<as2::tf::TfHandler> tf_handler,
                  takeoff_plugin_params &params) {
    node_ptr_             = node_ptr;
    tf_handler            = tf_handler;
    params_               = params;
    hover_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    this->ownInit();
  }

  virtual void state_callback(geometry_msgs::msg::PoseStamped &pose_msg,
                              geometry_msgs::msg::TwistStamped &twist_msg) {
    actual_pose_ = pose_msg;

    feedback_.actual_takeoff_height = actual_pose_.pose.position.z;
    feedback_.actual_takeoff_speed  = twist_msg.twist.linear.z;

    localization_flag_ = true;
    return;
  }

  bool on_activate(std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal) {
    as2_msgs::action::Takeoff::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  bool on_modify(std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal) {
    as2_msgs::action::Takeoff::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_modify(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  inline bool on_deactivate(const std::shared_ptr<std::string> &message) {
    return own_deactivate(message);
  }

  inline bool on_pause(const std::shared_ptr<std::string> &message) { return own_pause(message); }

  inline bool on_resume(const std::shared_ptr<std::string> &message) { return own_resume(message); }

  void on_execution_end(const as2_behavior::ExecutionStatus &state) {
    localization_flag_ = false;
    own_execution_end(state);
    return;
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal,
      std::shared_ptr<as2_msgs::action::Takeoff::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::Takeoff::Result> &result_msg) {
    as2_behavior::ExecutionStatus status = own_run();

    feedback_msg = std::make_shared<as2_msgs::action::Takeoff::Feedback>(feedback_);
    result_msg   = std::make_shared<as2_msgs::action::Takeoff::Result>(result_);
    return status;
  }

private:
  bool processGoal(as2_msgs::action::Takeoff::Goal &_goal) {
    if (!localization_flag_) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, there is no localization");
      return false;
    }
    return true;
  }

private:
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_ = nullptr;

  /* Interface with plugin */

protected:
  virtual void ownInit(){};

  virtual bool own_activate(as2_msgs::action::Takeoff::Goal &goal) = 0;

  virtual bool own_modify(as2_msgs::action::Takeoff::Goal &goal) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be modified, not implemented");
    return false;
  }

  virtual bool own_deactivate(const std::shared_ptr<std::string> &message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Takeoff can not be paused, not implemented, try to cancel it");
    return false;
  }

  virtual bool own_resume(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be resumed, not implemented");
    return false;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus &state) = 0;
  virtual as2_behavior::ExecutionStatus own_run()                            = 0;

  inline void sendHover() {
    hover_motion_handler_->sendHover();
    return;
  };

protected:
  as2::Node *node_ptr_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler = nullptr;

  as2_msgs::action::Takeoff::Goal goal_;
  as2_msgs::action::Takeoff::Feedback feedback_;
  as2_msgs::action::Takeoff::Result result_;

  takeoff_plugin_params params_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  bool localization_flag_;

};  // class TakeoffBase
}  // namespace takeoff_base
#endif  // TAKEOFF_BASE_HPP
