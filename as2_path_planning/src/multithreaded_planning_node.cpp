#include "frontier_allocator.hpp"
#include "path_planner.hpp"
#include "laserscan_to_occupancy_grid.hpp"
#include "map_server.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<FrontierAllocator>();
  auto node2 = std::make_shared<PathPlanner>();
  auto node3 = std::make_shared<LaserToOccupancyGridNode>();
  auto node4 = std::make_shared<MapServer>();

  executor.add_node(node);
  executor.add_node(node2);
  executor.add_node(node3);
  executor.add_node(node4);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
