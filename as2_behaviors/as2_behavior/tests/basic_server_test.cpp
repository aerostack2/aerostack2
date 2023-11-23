#include <iostream>
#include "as2_behavior/behavior_server.hpp"

#include <rclcpp/rclcpp.hpp>
// #include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/takeoff.hpp"

class TakeoffServer : public as2_behavior::BehaviorServer<as2_msgs::action::Takeoff> {
public:
  TakeoffServer(const std::string& name)
      : as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>(name) {
    std::cout << "TakeoffServer constructor" << std::endl;
  }
  int i = 0;
  bool on_activate(std::shared_ptr<const typename as2_msgs::action::Takeoff::Goal> goal) override {
    std::cout << "TakeoffServer activate" << std::endl;
    i = 0;
    return true;
  }
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::Takeoff::Goal>& goal,
      std::shared_ptr<as2_msgs::action::Takeoff::Feedback>& feedback_msg,
      std::shared_ptr<as2_msgs::action::Takeoff::Result>& result_msg) override {
    feedback_msg->actual_takeoff_height = i++;
    if (i < 100) {
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    result_msg->takeoff_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::cout << "Hello World!" << std::endl;
  auto node = std::make_shared<TakeoffServer>("TakeoffBehavior");
  rclcpp::spin(node);

  return 0;
}
