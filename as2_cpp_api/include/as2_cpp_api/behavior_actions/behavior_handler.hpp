#ifndef BEHAVIOR_ACTIONS__BEHAVIOR_HANDLER
#define BEHAVIOR_ACTIONS__BEHAVIOR_HANDLER

#include <stdexcept>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "as2_msgs/msg/behavior_status.hpp"

namespace as2
{
namespace as2_cpp_api
{

class BehaviorHandlerException : public std::runtime_error
{
public:
  explicit BehaviorHandlerException(const std::string & description)
  : std::runtime_error(description) {}
};

class BehaviorIsNotAvailableException : public BehaviorHandlerException
{
public:
  explicit BehaviorIsNotAvailableException(const std::string & description)
  : BehaviorHandlerException(description) {}
};

class GoalRejectedException : public BehaviorHandlerException
{
public:
  explicit GoalRejectedException(const std::string & description)
  : BehaviorHandlerException(description) {}
};

class ResultUnknownException : public BehaviorHandlerException
{
public:
  explicit ResultUnknownException(const std::string & description)
  : BehaviorHandlerException(description) {}
};

template<typename ActionMsgT>
class BehaviorHandler
{
public:
  using GoalHandleActionMsgT = typename rclcpp_action::ClientGoalHandle<ActionMsgT>;
  BehaviorHandler(const rclcpp::Node::SharedPtr & node, const std::string & behavior_name);
  ~BehaviorHandler() = default;
  // status();
  // feedback();
  // result_status();
  // result();
  // is_running();
  bool start(const typename ActionMsgT::Goal & goal_msg, bool wait_result);
  // modify(); // TODO
  bool pause();
  bool resume();
  bool stop();
  // wait_to_result();
  //__feedback_callback()
  // __status_callback();

protected:
  rclcpp::Node::SharedPtr node_;

private:
  bool waitForActionsAndServices();
  void __feedback_callback(
    typename GoalHandleActionMsgT::SharedPtr,
    const std::shared_ptr<const typename ActionMsgT::Feedback> feedback);

  typename rclcpp_action::Client<ActionMsgT>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  int status_;
  std::chrono::milliseconds wait_for_server_timeout_;
  std::string behavior_name_;
};

using namespace std::chrono_literals;

template<typename ActionMsgT>
BehaviorHandler<ActionMsgT>::BehaviorHandler(
  const rclcpp::Node::SharedPtr & node,
  const std::string & behavior_name)
: node_(node), behavior_name_(behavior_name)
{

  status_ = as2_msgs::msg::BehaviorStatus::IDLE;
  wait_for_server_timeout_ = std::chrono::milliseconds(1000);

  action_client_ = rclcpp_action::create_client<ActionMsgT>(node_, behavior_name_);

  pause_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/pause");

  resume_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/resume");

  stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/stop");

  if (!waitForActionsAndServices()) {
    throw BehaviorIsNotAvailableException(behavior_name_ + " is not available!");
  }
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::waitForActionsAndServices()
{
  bool is_ok = true;

  is_ok = is_ok && pause_client_->wait_for_service(wait_for_server_timeout_);
  is_ok = is_ok && resume_client_->wait_for_service(wait_for_server_timeout_);
  is_ok = is_ok && stop_client_->wait_for_service(wait_for_server_timeout_);
  is_ok = is_ok && action_client_->wait_for_action_server(wait_for_server_timeout_);

  return is_ok;
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::start(
  const typename ActionMsgT::Goal & goal_msg,
  bool wait_result)
{
  using namespace std::placeholders;
  auto send_goal_options = typename rclcpp_action::Client<ActionMsgT>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&BehaviorHandler<ActionMsgT>::__feedback_callback, this, _1, _2);

  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

  if (!goal_handle_future.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "%s : Failed to send goal!", behavior_name_.c_str());
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "%s : Goal is rejected by server!", behavior_name_.c_str());
    return false;
  }

  if (!wait_result) {
    return true;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client_->async_get_result(goal_handle);
  if (!result_future.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "%s : Failed to get result!", behavior_name_.c_str());
    return false;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was succeeded");
      return true;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return false;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return false;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return false;
  }

  return true;
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::pause()
{
  if (status_ != as2_msgs::msg::BehaviorStatus::RUNNING) {
    return true;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response_future = pause_client_->async_send_request(request);

  // Wait for the result.
  if (!response_future.valid()) {return false;}

  auto response = response_future.get();
  return response->success;
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::resume()
{
  if (status_ != as2_msgs::msg::BehaviorStatus::RUNNING) {
    return true;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response_future = resume_client_->async_send_request(request);

  // Wait for the result.
  if (!response_future.valid()) {return false;}

  auto response = response_future.get();
  return response->success;
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::stop()
{
  if (status_ != as2_msgs::msg::BehaviorStatus::RUNNING) {
    return true;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response_future = stop_client_->async_send_request(request);

  // Wait for the result.
  if (!response_future.valid()) {return false;}

  auto response = response_future.get();
  return response->success;
}


template<typename ActionMsgT>
void BehaviorHandler<ActionMsgT>::__feedback_callback(
  typename GoalHandleActionMsgT::SharedPtr,
  const std::shared_ptr<const typename ActionMsgT::Feedback> feedback)
{
}

} // namespace as2_cpp_api
} // namespace as2
#endif
