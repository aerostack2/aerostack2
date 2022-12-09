#include <iostream>
#include "as2_behavior/behavior_server.hpp"

#include <rclcpp/rclcpp.hpp>
// #include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/take_off.hpp"

class TakeOffServer : public as2_behavior::BehaviorServer<as2_msgs::action::TakeOff> {
public:
  TakeOffServer(const std::string& name)
      : as2_behavior::BehaviorServer<as2_msgs::action::TakeOff>(name) {
    std::cout << "TakeOffServer constructor" << std::endl;
  }
  int i = 0;
  bool on_activate(std::shared_ptr<const typename as2_msgs::action::TakeOff::Goal> goal) override {
    std::cout << "TakeOffServer activate" << std::endl;
    i = 0;
    return true;
  }
  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::TakeOff::Goal>& goal,
      std::shared_ptr<as2_msgs::action::TakeOff::Feedback>& feedback_msg,
      std::shared_ptr<as2_msgs::action::TakeOff::Result>& result_msg) override {
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
  auto node = std::make_shared<TakeOffServer>("TakeOffBehaviour");
  rclcpp::spin(node);

  return 0;
}
