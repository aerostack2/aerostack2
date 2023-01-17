#include <rclcpp/rclcpp.hpp>
#include "as2_behaviors_platform/set_arming_state_behavior.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetArmingStateBehavior>());
  rclcpp::shutdown();
  return 0;
}
