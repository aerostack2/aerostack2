// Copyright 2025 Universidad Politécnica de Madrid
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
 *  \file       gripper_behavior.cpp
 *  \brief      gripper_behavior implementation file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

 #include "as2_behaviors_gripper/gripper_handler_behavior.hpp"
 #include "as2_core/names/actions.hpp"
 #include "as2_core/names/topics.hpp"

GripperHandlerBehavior::GripperHandlerBehavior()
: as2_behavior::BehaviorServer<as2_msgs::action::GripperHandler>("GripperHandlerBehavior")
{
  std::string plugin_name = this->declare_parameter<std::string>(
    "plugin_name",
    "gripper_handler_plugin_gazebo");
  plugin_name += "::Plugin";
  loader_ = std::make_shared<pluginlib::ClassLoader<gripper_handler_base::PluginBase>>(
    "as2_behaviors_gripper",
    "gripper_handler_base::PluginBase");
  gripper_handler_plugin_ = loader_->createSharedInstance(plugin_name);
  gripper_handler_plugin_->initialize(this);
  RCLCPP_INFO(this->get_logger(), "GRIPPER HANDLER PLUGIN LOADED: %s", plugin_name.c_str());
}

bool GripperHandlerBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal)
{
  as2_msgs::action::GripperHandler::Goal new_goal = *goal;
  RCLCPP_INFO(
    this->get_logger(),
    "GripperHandlerBehavior on_activate");
  return gripper_handler_plugin_->on_activate(
    std::make_shared<const as2_msgs::action::GripperHandler::Goal>(new_goal));
}

bool GripperHandlerBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal)
{
  as2_msgs::action::GripperHandler::Goal new_goal = *goal;

  return gripper_handler_plugin_->on_modify(
    std::make_shared<const as2_msgs::action::GripperHandler::Goal>(new_goal));
}

bool GripperHandlerBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  return gripper_handler_plugin_->on_deactivate(message);
  return true;
}

bool GripperHandlerBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  return gripper_handler_plugin_->on_pause(message);
  return true;
}

bool GripperHandlerBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  return gripper_handler_plugin_->on_resume(message);
  return true;
}

as2_behavior::ExecutionStatus GripperHandlerBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> & goal,
  std::shared_ptr<as2_msgs::action::GripperHandler::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::GripperHandler::Result> & result_msg)
{
  return gripper_handler_plugin_->on_run(goal, feedback_msg, result_msg);
  return as2_behavior::ExecutionStatus::RUNNING;
}

void GripperHandlerBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  return gripper_handler_plugin_->on_execution_end(state);
}
