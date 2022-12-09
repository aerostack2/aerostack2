#include <rclcpp/rclcpp.hpp>
#include "as2_basic_behaviors/set_offboard_mode_behavior.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetOffboardModeBehavior>());
  rclcpp::shutdown();
  return 0;
}
