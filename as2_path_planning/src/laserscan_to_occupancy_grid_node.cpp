#include "laserscan_to_occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
//  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<LaserToOccupancyGridNode>();
//  executor.add_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
