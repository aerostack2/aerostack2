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
 ********************************************************************************/

#include "path_planner_behavior.hpp"

PathPlannerBehavior::PathPlannerBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>("path_planner", options)
{
}

bool PathPlannerBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
}

bool PathPlannerBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
}

bool PathPlannerBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
}

bool PathPlannerBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
}

bool PathPlannerBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
}

void PathPlannerBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
}

as2_behavior::ExecutionStatus PathPlannerBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> & goal,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Result> & result_msg)
{
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlannerBehavior)
