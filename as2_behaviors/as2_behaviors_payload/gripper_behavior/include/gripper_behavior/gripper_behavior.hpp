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
 *  \file       gripper_handler_behavior.hpp
 *  \brief      gripper_handler_behavior header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_PAYLOAD__GRIPPER_BEHAVIOR_HPP_
#define AS2_BEHAVIORS_PAYLOAD__GRIPPER_BEHAVIOR_HPP_

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pluginlib/class_loader.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/gripper_handler.hpp"
#include "gripper_behavior/gripper_behavior_plugin_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace gripper_behavior
{

class GripperHandlerBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::GripperHandler>
{
public:
  explicit GripperHandlerBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GripperHandlerBehavior() {}

private:
// Behavior action parameters
  as2_msgs::action::GripperHandler::Goal goal_;
  as2_msgs::action::GripperHandler::Result result_;
  as2_msgs::action::GripperHandler::Feedback feedback_;

private:
  /** As2 Behavior methods **/
  bool on_activate(
    std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal)
  override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> & goal,
    std::shared_ptr<as2_msgs::action::GripperHandler::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::GripperHandler::Result> & result_msg) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  /** Gripper handler Behavior plugin **/
  std::filesystem::path plugin_name_;
  std::shared_ptr<pluginlib::ClassLoader<gripper_behavior_plugin_base::GripperBase>> loader_;
  std::shared_ptr<gripper_behavior_plugin_base::GripperBase> gripper_handler_plugin_;
  /* Other ROS 2 interfaces */
};

}  // namespace gripper_behavior

#endif  // AS2_BEHAVIORS_PAYLOAD__GRIPPER_BEHAVIOR_HPP_
