#ifndef __AS2_BEHAVIOR_SERVER__IMPL_HPP__
#define __AS2_BEHAVIOR_SERVER__IMPL_HPP__
#include "as2_behavior/__detail/behavior_server__class.hpp"

namespace as2_behavior {

template <typename actionT>
BehaviorServer<actionT>::BehaviorServer(const std::string& name)
    : as2::Node(name), action_name_(name) {
  register_action();
  register_service_servers();
  register_publishers();
  register_timers();
}

template <typename actionT>
void BehaviorServer<actionT>::register_action() {
  // create a group for the action server
  auto action_server_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  this->action_server_ = rclcpp_action::create_server<actionT>(
      this, this->generate_global_name(action_name_),
      std::bind(&BehaviorServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BehaviorServer::handleCancel, this, std::placeholders::_1),
      std::bind(&BehaviorServer::handleAccepted, this, std::placeholders::_1));
};

template <typename actionT>
rclcpp_action::GoalResponse BehaviorServer<actionT>::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const typename actionT::Goal> goal) {
  RCLCPP_DEBUG(this->get_logger(), "Received goal request with UUID: %s", (char*)uuid.data());
  if (this->activate(goal)) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

template <typename actionT>
rclcpp_action::CancelResponse BehaviorServer<actionT>::handleCancel(
    const std::shared_ptr<GoalHandleAction> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Request to cancel goal received");
  std_srvs::srv::Trigger::Request::SharedPtr req =
      std::make_shared<std_srvs::srv::Trigger::Request>();
  std_srvs::srv::Trigger::Response::SharedPtr res =
      std::make_shared<std_srvs::srv::Trigger::Response>();
  deactivate(req, res);
  if (res->success) {
    return rclcpp_action::CancelResponse::ACCEPT;
  } else {
    return rclcpp_action::CancelResponse::REJECT;
  }
};

template <typename actionT>
void BehaviorServer<actionT>::handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
  goal_handle_ = goal_handle;
};

template <typename actionT>
std::string BehaviorServer<actionT>::generate_name(const std::string& name) {
  return std::string(this->get_name()) + "/_behavior/" + name;
}

template <typename actionT>
void BehaviorServer<actionT>::register_service_servers() {
  pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
      generate_name("pause"),
      std::bind(&BehaviorServer::pause, this, std::placeholders::_1, std::placeholders::_2));
  resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
      generate_name("resume"),
      std::bind(&BehaviorServer::resume, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename actionT>
void BehaviorServer<actionT>::register_publishers() {
  feedback_pub_    = this->create_publisher<feedback_msg>(generate_name("feedback"), 10);
  goal_status_pub_ = this->create_publisher<goal_status_msg>(generate_name("goal_status"), 10);
  behavior_status_pub_ =
      this->create_publisher<BehaviorStatus>(generate_name("behavior_status"), 10);
}

template <typename actionT>
void BehaviorServer<actionT>::register_timers() {
  behavior_status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&BehaviorServer::publish_behavior_status, this));
}

template <typename actionT>
void BehaviorServer<actionT>::register_run_timer() {
  run_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&BehaviorServer::timer_callback, this));
}
template <typename actionT>
void BehaviorServer<actionT>::cleanup_run_timer(const ExecutionStatus& state) {
  on_excution_end(state);
  goal_handle_.reset();
  run_timer_.reset();
}

template <typename actionT>
bool BehaviorServer<actionT>::on_activate(std::shared_ptr<const typename actionT::Goal> goal) {
  return true;
};

template <typename actionT>
bool BehaviorServer<actionT>::on_modify(std::shared_ptr<const typename actionT::Goal> goal) {
  return true;
}

template <typename actionT>
bool BehaviorServer<actionT>::on_deactivate(const std::shared_ptr<std::string>& message) {
  return true;
}
template <typename actionT>
bool BehaviorServer<actionT>::on_pause(const std::shared_ptr<std::string>& message) {
  return true;
}
template <typename actionT>
bool BehaviorServer<actionT>::on_resume(const std::shared_ptr<std::string>& message) {
  return true;
}

/* template <typename actionT>
ExecutionStatus BehaviorServer<actionT>::on_run(typename feedback_msg::SharedPtr& fb) {
  return ExecutionStatus::SUCCESS;
} */

template <typename actionT>
ExecutionStatus BehaviorServer<actionT>::on_run(
    const typename std::shared_ptr<const typename actionT::Goal>& goal,
    typename std::shared_ptr<typename actionT::Feedback>& feedback_msg,
    typename std::shared_ptr<typename actionT::Result>& result_msg) {
  return ExecutionStatus::SUCCESS;
}  // namespace as2_behavior

template <typename actionT>
bool BehaviorServer<actionT>::activate(std::shared_ptr<const typename actionT::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "START");
  if (on_activate(goal)) {
    register_run_timer();
    behavior_status_.status = BehaviorStatus::RUNNING;
    return true;
  }
  return false;
};
template <typename actionT>
void BehaviorServer<actionT>::deactivate(
    const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
    typename std_srvs::srv::Trigger::Response::SharedPtr result) {
  RCLCPP_INFO(this->get_logger(), "STOP");
  auto msg        = std::make_shared<std::string>();
  result->success = on_deactivate(msg);
  result->message = *msg;
  if (result->success) cleanup_run_timer(ExecutionStatus::ABORTED);
};
template <typename actionT>

void BehaviorServer<actionT>::modify(std::shared_ptr<const typename actionT::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "MODIFY");
  on_modify(goal);
};
template <typename actionT>
void BehaviorServer<actionT>::pause(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
                                    typename std_srvs::srv::Trigger::Response::SharedPtr result) {
  RCLCPP_INFO(this->get_logger(), "PAUSE");
  if (behavior_status_.status != BehaviorStatus::RUNNING) {
    result->success = false;
    result->message = "Behavior is not running";
    return;
  }
  auto msg        = std::make_shared<std::string>();
  result->success = on_pause(msg);
  result->message = *msg;
  if (result->success) {
    behavior_status_.status = BehaviorStatus::PAUSED;
  }
};
template <typename actionT>
void BehaviorServer<actionT>::resume(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
                                     typename std_srvs::srv::Trigger::Response::SharedPtr result) {
  RCLCPP_INFO(this->get_logger(), "RESUME");
  if (behavior_status_.status != BehaviorStatus::PAUSED) {
    result->success = false;
    result->message = "Behavior is not paused";
    return;
  }
  auto msg        = std::make_shared<std::string>();
  result->success = on_resume(msg);
  result->message = *msg;
  if (result->success) {
    behavior_status_.status = BehaviorStatus::RUNNING;
  }
};

template <typename actionT>
void BehaviorServer<actionT>::run(
    const typename std::shared_ptr<GoalHandleAction>& goal_handle_action) {
  if (behavior_status_.status != BehaviorStatus::RUNNING) {
    return;
  };
  auto goal              = goal_handle_action->get_goal();
  auto feedback          = std::make_shared<typename actionT::Feedback>();
  auto result            = std::make_shared<typename actionT::Result>();
  ExecutionStatus status = on_run(goal, feedback, result);

  switch (status) {
    case ExecutionStatus::SUCCESS: {
      RCLCPP_INFO(this->get_logger(), "SUCCESS");
      behavior_status_.status = BehaviorStatus::IDLE;
      goal_handle_->succeed(result);
    } break;
    case ExecutionStatus::RUNNING: {
      auto clk = this->get_clock();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clk, 5000, "RUNNING");
      goal_handle_action->publish_feedback(feedback);
      behavior_status_.status = BehaviorStatus::RUNNING;
    } break;
    case ExecutionStatus::FAILURE: {
      RCLCPP_INFO(this->get_logger(), "FAILURE");
      behavior_status_.status = BehaviorStatus::IDLE;
      goal_handle_->abort(result);
    } break;
    case ExecutionStatus::ABORTED: {
      RCLCPP_INFO(this->get_logger(), "ABORTED");
      behavior_status_.status = BehaviorStatus::IDLE;
      goal_handle_->abort(result);
    } break;
  }

  if (behavior_status_.status != BehaviorStatus::RUNNING) {
    cleanup_run_timer(status);
  }
}

template <typename actionT>
void BehaviorServer<actionT>::publish_behavior_status() {
  BehaviorStatus msg;
  msg.status = behavior_status_.status;
  behavior_status_pub_->publish(msg);
}
};  // namespace as2_behavior

#endif  // AS2_BEHAVIOR__BEHAVIOR_SERVER_HPP_
