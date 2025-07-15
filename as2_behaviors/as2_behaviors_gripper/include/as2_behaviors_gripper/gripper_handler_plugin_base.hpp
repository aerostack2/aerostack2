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
 *  \file       gripper_handler_plugin_base.hpp
 *  \brief      gripper_handler_plugin_base header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

 #ifndef AS2_BEHAVIORS_GRIPPER__GRIPPER_HANDLER_PLUGIN_BASE_HPP_
 #define AS2_BEHAVIORS_GRIPPER__GRIPPER_HANDLER_PLUGIN_BASE_HPP_

 #include <memory>
 #include <vector>
 #include <string>

 #include <as2_core/node.hpp>
 #include <as2_behavior/behavior_utils.hpp>
 #include "as2_msgs/action/gripper_handler.hpp"


namespace gripper_handler_base
{
class PluginBase
{
public:
  PluginBase() {}
  virtual ~PluginBase() {}
  void initialize(
    as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
    ownInit();
  }

  bool on_activate(
    std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal)
  {
    as2_msgs::action::GripperHandler::Goal goal_candidate = *goal;
    if (own_activate(goal_candidate)) {
      goal_ = goal_candidate;
      return true;
    }
    return true;
  }
  inline bool on_deactivate(const std::shared_ptr<std::string> & message)
  {return own_deactivate(message);}
  inline bool on_modify(std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal)
  {
    as2_msgs::action::GripperHandler::Goal goal_candidate = *goal;
    return own_modify(goal_candidate);
  }
  inline bool on_pause(const std::shared_ptr<std::string> & message) {return own_pause(message);}
  inline bool on_resume(const std::shared_ptr<std::string> & message) {return own_resume(message);}

  void on_execution_end(const as2_behavior::ExecutionStatus & state)
  {
    own_execution_end(state);
    return;
  }
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::GripperHandler::Goal> goal,
    std::shared_ptr<as2_msgs::action::GripperHandler::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::GripperHandler::Result> & result_msg)
  {
    as2_behavior::ExecutionStatus status = own_run();

    feedback_msg = std::make_shared<as2_msgs::action::GripperHandler::Feedback>(feedback_);
    result_msg = std::make_shared<as2_msgs::action::GripperHandler::Result>(result_);
    return status;
  }

protected:
  virtual void ownInit() {}
  virtual bool own_activate(as2_msgs::action::GripperHandler::Goal & goal) = 0;
  virtual bool own_modify(as2_msgs::action::GripperHandler::Goal & goal) = 0;

  virtual bool own_deactivate(const std::shared_ptr<std::string> & message) = 0;

  virtual bool own_pause(const std::shared_ptr<std::string> & message) = 0;


  virtual bool own_resume(const std::shared_ptr<std::string> & message) = 0;

  virtual void own_execution_end(const as2_behavior::ExecutionStatus & state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

protected:
  as2::Node * node_ptr_;
  as2_msgs::action::GripperHandler::Goal goal_;
  as2_msgs::action::GripperHandler::Feedback feedback_;
  as2_msgs::action::GripperHandler::Result result_;
};
}   // namespace gripper_handler_base

#endif  // AS2_BEHAVIORS_GRIPPER__GRIPPER_HANDLER_PLUGIN_BASE_HPP_
