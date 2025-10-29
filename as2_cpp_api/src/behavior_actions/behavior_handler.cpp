#include "as2_cpp_api/behavior_actions/behavior_handler.hpp"

namespace as2
{
namespace as2_cpp_api
{

using namespace std::chrono_literals;

template<typename ActionMsgT>
BehaviorHandler<ActionMsgT>::BehaviorHandler(
  const rclcpp::Node::SharedPtr & node,
  const std::string & behavior_name)
: node_(node)
{

  status_ = as2_msgs::msg::BehaviorStatus::IDLE;
  wait_for_server_timeout_ = std::chrono::milliseconds(1000);

  action_client_ = rclcpp_action::create_client<ActionMsgT>(node_, behavior_name);

  pause_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/pause");

  resume_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/resume");

  stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
    behavior_name + "/_behavior/stop");

  if (waitForActionsAndServices()) {
    throw BehaviorIsNotAvailableException(behavior_name + " is not available!");
  }
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::waitForActionsAndServices()
{
  bool is_ok = true;

  is_ok = is_ok && pause_client_->wait_for_service(1s);
  is_ok = is_ok && resume_client_->wait_for_service(1s);
  is_ok = is_ok && stop_client_->wait_for_service(1s);
  is_ok = is_ok && action_client_->wait_for_action_server(1s);

  return is_ok;
}

template<typename ActionMsgT>
bool BehaviorHandler<ActionMsgT>::start(
  const typename ActionMsgT::Goal & goal_msg,
  bool wait_result)
{
  using namespace std::placeholders;
  auto send_goal_options = rclcpp_action::Client<ActionMsgT>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&BehaviorHandler<ActionMsgT>::__feedback_callback, this, _1, _2);

  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
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
  if (rclcpp::spin_until_future_complete(node_, response_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

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
  if (rclcpp::spin_until_future_complete(node_, response_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

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
  if (rclcpp::spin_until_future_complete(node_, response_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto response = response_future.get();
  return response->success;
}


template<typename ActionMsgT>
void BehaviorHandler<ActionMsgT>::__feedback_callback(
  typename GoalHandleActionMsgT::SharedPtr,
  const std::shared_ptr<const typename ActionMsgT::Feedback> feedback)
{
}

}
}
