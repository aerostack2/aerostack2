#include <as2_core/core_functions.hpp>
#include "as2_core/core_functions.hpp"
#include "tello_platform.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto tello_node = std::make_shared<TelloPlatform>();
  // tello_node->preset_loop_frequency(200);
  // as2::spinLoop(tello_node);
  rclcpp::spin(tello_node);
  tello_node->~TelloPlatform();

  tello_node.reset();
  rclcpp::shutdown();
  return 0;
}
