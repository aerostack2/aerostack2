#ifndef __SET_ARMING_STATE_BEHAVIOR_HPP__
#define __SET_ARMING_STATE_BEHAVIOR_HPP__

#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/set_arming_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

class SetArmingStateBehavior
    : public as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState> {
public:
  SetArmingStateBehavior()
      : as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState>("set_arming_state") {
    client_ = this->create_client<std_srvs::srv::SetBool>("set_arming_state");
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;

public:
  bool on_activate(std::shared_ptr<const as2_msgs::action::SetArmingState::Goal> goal) override {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

    req->data = goal->request;
    using namespace std::chrono_literals;
    if (!client_->wait_for_service(5s)) {
      RCLCPP_INFO(get_logger(), "service not available");
      return false;
    }
    future_ = client_->async_send_request(req);
    if (!future_.valid()) {
      RCLCPP_INFO(get_logger(), "request not sent");
      return false;
    }
    return true;
  };

  bool on_modify(std::shared_ptr<const as2_msgs::action::SetArmingState::Goal> goal) override {
    RCLCPP_WARN(get_logger(), "Cannot modify a service request");
    return false;
  };

  bool on_deactivate(const std::shared_ptr<std::string>& message) override {
    *message = "Unable to deactivate InstantBehavior";
    return false;
  };
  bool on_pause(const std::shared_ptr<std::string>& message) override {
    *message = "Unable to pause InstantBehavior";
    return false;
  };

  bool on_resume(const std::shared_ptr<std::string>& message) override {
    *message = "Unable to resume InstantBehavior";
    return false;
  };

  void on_execution_end(const as2_behavior::ExecutionStatus& state) override{};

  as2_behavior::ExecutionStatus on_run(
      const typename std::shared_ptr<const as2_msgs::action::SetArmingState::Goal>& goal,
      typename std::shared_ptr<as2_msgs::action::SetArmingState::Feedback>& feedback_msg,
      typename std::shared_ptr<as2_msgs::action::SetArmingState::Result>& result_msg) override {
    if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = future_.get();
      if (result->success) {
        result_msg->success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        result_msg->success = true;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  };
};
#endif
