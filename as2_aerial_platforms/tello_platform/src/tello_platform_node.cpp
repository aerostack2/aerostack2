#include "tello_platform.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto tello_node = std::make_shared<TelloPlatform>();
  rclcpp::spin(tello_node);
  tello_node->~TelloPlatform();

  tello_node.reset();
  rclcpp::shutdown();
  return 0;
}
