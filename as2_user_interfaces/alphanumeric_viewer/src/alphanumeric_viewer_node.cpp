// "Copyright [year] <Copyright Owner>"

#include "alphanumeric_viewer.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlphanumericViewer>();
  node->preset_loop_frequency(10);  // Node frequency for run and callbacks
  // Node with only callbacks
  // as2::spinLoop(node);
  // Node with run
  as2::spinLoop(node, std::bind(&AlphanumericViewer::run, node));

  rclcpp::shutdown();
  return 0;
}
