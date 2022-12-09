#ifndef __BEHAVIOR_SERVER__CLASS_HPP__
#define __BEHAVIOR_SERVER__CLASS_HPP__

#include <as2_core/node.hpp>
#include <as2_behavior/behaviour_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

namespace as2_behavior {

template <typename actionT>
class BehaviorServer : public as2::Node {
protected:
public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<actionT>;
  std::string action_name_;

  typename rclcpp_action::Server<actionT>::SharedPtr action_server_;

public:
  void register_action();
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const typename actionT::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle);

  std::shared_ptr<GoalHandleAction> goal_handle_;

  as2_msgs::msg::BehaviorStatus behavior_status_;

  // TODO: remove all the unnecessary using statements
  using BehaviorStatus  = as2_msgs::msg::BehaviorStatus;
  using start_srv       = typename actionT::Impl::SendGoalService;
  using modify_srv      = start_srv;
  using result_srv      = typename actionT::Impl::GetResultService;
  using feedback_msg    = typename actionT::Impl::FeedbackMessage;
  using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
  using cancel_srv      = typename actionT::Impl::CancelGoalService;

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
  std::string generate_name(const std::string& name);

private:
  void register_service_servers();

  void register_publishers();
  void register_timers();

  void register_run_timer();
  void cleanup_run_timer(const ExecutionStatus& status);

public:
  BehaviorServer(const std::string& name);

  // TODO: CONVERT INTO PURE VIRTUAL FUNCTIONS
  virtual bool on_activate(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_modify(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_deactivate(const std::shared_ptr<std::string>& message);
  virtual bool on_pause(const std::shared_ptr<std::string>& message);
  virtual bool on_resume(const std::shared_ptr<std::string>& message);

  virtual void on_excution_end(const ExecutionStatus& state){};

  virtual ExecutionStatus on_run(const typename std::shared_ptr<const typename actionT::Goal>& goal,
                                 typename std::shared_ptr<typename actionT::Feedback>& feedback_msg,
                                 typename std::shared_ptr<typename actionT::Result>& result_msg);

  bool activate(std::shared_ptr<const typename actionT::Goal> goal);
  void modify(std::shared_ptr<const typename actionT::Goal> goal);
  void deactivate(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
                  typename std_srvs::srv::Trigger::Response::SharedPtr result);
  void pause(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
             typename std_srvs::srv::Trigger::Response::SharedPtr result);
  void resume(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
              typename std_srvs::srv::Trigger::Response::SharedPtr result);

  void run(const typename std::shared_ptr<GoalHandleAction>& goal_handle_action);

  void timer_callback() { run(goal_handle_); };
  void publish_behavior_status();
};
};  // namespace as2_behavior

#endif  // __BEHAVIOR_SERVER__CLASS_HPP__

