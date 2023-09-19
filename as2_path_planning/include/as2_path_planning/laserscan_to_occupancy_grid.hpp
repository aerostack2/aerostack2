#ifndef LASERSCAN_TO_OCCUPANCY_GRID_HPP_
#define LASERSCAN_TO_OCCUPANCY_GRID_HPP_

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "utils.hpp"

#define RESOLUTION 0.25
#define OCC_WIDTH 300
#define OCC_HEIGHT 300
#define RANGE_MIN 0.5

class LaserToOccupancyGridNode : public rclcpp::Node {
public:
  LaserToOccupancyGridNode();
  ~LaserToOccupancyGridNode(){};

private:
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  void
  processLaserScanWithTf(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  void positionCallback(const nav_msgs::msg::Odometry::SharedPtr data);

  std::vector<std::vector<int>> getMiddlePoints(std::vector<int> p1,
                                                std::vector<int> p2);

  std::vector<std::vector<int>> safeZone(std::vector<int> drone_cell);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
};
#endif // LASERSCAN_TO_OCCUPANCY_GRID_HPP_