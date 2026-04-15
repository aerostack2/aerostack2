#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "overlay_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2_camera_overlay::OverlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
