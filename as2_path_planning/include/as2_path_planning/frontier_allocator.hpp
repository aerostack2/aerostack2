#ifndef FRONTIER_ALLOCATOR_HPP_
#define FRONTIER_ALLOCATOR_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

#include "utils.hpp"

class FrontierAllocator : public rclcpp::Node {
public:
  FrontierAllocator();
  ~FrontierAllocator(){};

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr last_occ_grid_;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // TODO: new custom msgs
  void
  allocateFrontierCbk(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr allocate_frontier_serv_;
};
#endif // FRONTIER_ALLOCATOR_HPP_