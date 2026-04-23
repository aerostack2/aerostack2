#include "overlay_node.hpp"
#include <QApplication>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  setenv("QT_QPA_PLATFORM", "offscreen", 0);
  QApplication qt_app(argc, argv);
  (void)qt_app;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2_camera_overlay::OverlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
