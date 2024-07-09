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
* @file behavior_server__class.hpp
*
* @brief Class definition for a behavior server
*
* @author Miguel Fernández Cortizas
*         Pedro Arias Pérez
*         David Pérez Saura
*         Rafael Pérez Seguí
*/

#ifndef AS2_BEHAVIOR__BEHAVIOR_SERVER__CLASS_HPP__
#define AS2_BEHAVIOR__BEHAVIOR_SERVER__CLASS_HPP__

#include <string>
#include <memory>
#include <as2_behavior/behavior_utils.hpp>
#include <as2_core/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

namespace as2_behavior
{

template<typename actionT>
class BehaviorServer : public as2::Node
{
protected:
//

public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<actionT>;
  std::string action_name_;

  typename rclcpp_action::Server<actionT>::SharedPtr action_server_;

public:
  void register_action();
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename actionT::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle);

  std::shared_ptr<GoalHandleAction> goal_handle_;

  as2_msgs::msg::BehaviorStatus behavior_status_;

  // TODO(CVAR): remove all the unnecessary using statements
  using BehaviorStatus = as2_msgs::msg::BehaviorStatus;
  using start_srv = typename actionT::Impl::SendGoalService;
  using modify_srv = start_srv;
  using result_srv = typename actionT::Impl::GetResultService;
  using feedback_msg = typename actionT::Impl::FeedbackMessage;
  using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
  using cancel_srv = typename actionT::Impl::CancelGoalService;

private:
  // create rclmessage formed by goal and result_msg
  typename rclcpp::Service<start_srv>::SharedPtr start_srv_;
  typename rclcpp::Service<modify_srv>::SharedPtr modify_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  typename rclcpp::Publisher<feedback_msg>::SharedPtr feedback_pub_;
  typename rclcpp::Publisher<goal_status_msg>::SharedPtr goal_status_pub_;

  rclcpp::Publisher<BehaviorStatus>::SharedPtr behavior_status_pub_;
  rclcpp::TimerBase::SharedPtr behavior_status_timer_;
  rclcpp::TimerBase::SharedPtr run_timer_;

private:
  std::string generate_name(const std::string & name);

private:
  void register_service_servers();

  void register_publishers();
  void register_timers();

  void register_run_timer();
  void cleanup_run_timer(const ExecutionStatus & status);

public:
  BehaviorServer(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // TODO(CVAR): CONVERT INTO PURE VIRTUAL FUNCTIONS
  virtual bool on_activate(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_modify(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_deactivate(const std::shared_ptr<std::string> & message);
  virtual bool on_pause(const std::shared_ptr<std::string> & message);
  virtual bool on_resume(const std::shared_ptr<std::string> & message);

  virtual void on_execution_end(const ExecutionStatus & state) {}

  virtual ExecutionStatus on_run(
    const typename std::shared_ptr<const typename actionT::Goal> & goal,
    typename std::shared_ptr<typename actionT::Feedback> & feedback_msg,
    typename std::shared_ptr<typename actionT::Result> & result_msg);

  bool activate(std::shared_ptr<const typename actionT::Goal> goal);
  void modify(std::shared_ptr<const typename actionT::Goal> goal);
  void deactivate(
    const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
    typename std_srvs::srv::Trigger::Response::SharedPtr result);
  void pause(
    const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
    typename std_srvs::srv::Trigger::Response::SharedPtr result);
  void resume(
    const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
    typename std_srvs::srv::Trigger::Response::SharedPtr result);

  void run(const typename std::shared_ptr<GoalHandleAction> & goal_handle_action);

  void timer_callback() {run(goal_handle_);}
  void publish_behavior_status();
};
}   // namespace as2_behavior

#endif  // AS2_BEHAVIOR__BEHAVIOR_SERVER__CLASS_HPP__
