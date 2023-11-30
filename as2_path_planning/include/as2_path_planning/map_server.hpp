#ifndef MAP_SERVER_HPP_
#define MAP_SERVER_HPP_

#include <as2_msgs/msg/labeled_occupancy_grid.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

#include "A_star_algorithm.hpp"
#include "utils.hpp"

class MapServer : public rclcpp::Node {
public:
  MapServer();
  ~MapServer(){};

private:
  double map_resolution_ = 0.0;
  int map_width_ = 0;
  int map_height_ = 0;

  nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid_;
  grid_map::GridMap grid_map_;
  grid_map::GridMapRosConverter converter_;

  void
  occGridCallback(const as2_msgs::msg::LabeledOccupancyGrid::SharedPtr msg);
  void saveMapCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                       std_srvs::srv::Empty::Response::SharedPtr response);

  // Helpers
  void showMap(const cv::Mat &mat, std::string window_name = "Map");

  rclcpp::Subscription<as2_msgs::msg::LabeledOccupancyGrid>::SharedPtr
      occ_grid_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      occ_grid_filter_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr show_map_serv_;
};
#endif // MAP_SERVER_HPP_