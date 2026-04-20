#include <cstdlib>
#include <memory>

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "overlay_node.hpp"

int main(int argc, char ** argv) {
  // Qt must be set up before rclcpp::init so that QObject constructors inside
  // rviz_common::Display work even before the display context is created.
  // The offscreen platform plugin provides a headless GUI stack with no window.
  setenv("QT_QPA_PLATFORM", "offscreen", 0 /* don't overwrite if already set */);
  QApplication qt_app(argc, argv);
  (void)qt_app;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<as2_camera_overlay::OverlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
