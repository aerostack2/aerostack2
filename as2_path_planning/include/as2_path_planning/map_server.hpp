#ifndef MAP_SERVER_HPP_
#define MAP_SERVER_HPP_

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
  nav_msgs::msg::OccupancyGrid::SharedPtr last_occ_grid_;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid);
  void saveMapCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                       std_srvs::srv::Empty::Response::SharedPtr response);

  // Helpers
  void showMap(const cv::Mat &mat, std::string window_name = "Map");

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr show_map_serv_;
};
#endif // MAP_SERVER_HPP_