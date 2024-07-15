#include "laserscan_to_occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserToOccupancyGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}