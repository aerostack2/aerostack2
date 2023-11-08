#include "frontier_allocator.hpp"

FrontierAllocator::FrontierAllocator() : Node("frontier_allocator") {
  occ_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "occ_grid", 1,
      std::bind(&FrontierAllocator::occGridCallback, this,
                std::placeholders::_1));
  allocate_frontier_serv_ = create_service<std_srvs::srv::Empty>(
      "allocate_frontier",
      std::bind(&FrontierAllocator::allocateFrontierCbk, this,
                std::placeholders::_1, std::placeholders::_2));
}

void FrontierAllocator::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_grid_ = msg;
};

// TODO: new custom msgs
void FrontierAllocator::allocateFrontierCbk(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response){};