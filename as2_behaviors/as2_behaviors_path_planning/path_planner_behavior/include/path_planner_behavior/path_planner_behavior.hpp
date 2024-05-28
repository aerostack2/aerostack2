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
 ********************************************************************************/

#ifndef PATH_PLANNER_BEHAVIOR__PATH_PLANNER_BEHAVIOR_HPP_
#define PATH_PLANNER_BEHAVIOR__PATH_PLANNER_BEHAVIOR_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"


class PathPlannerBehavior
  : public as2_behavior::BehaviorServer<
    as2_msgs::action::NavigateToPoint>
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
};

#endif  // PATH_PLANNER_BEHAVIOR__PATH_PLANNER_BEHAVIOR_HPP_
