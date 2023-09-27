#ifndef EXPLORER_HPP_
#define EXPLORER_HPP_

#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "utils.hpp"
#include "viz_utils.hpp"

#define FRONTIER_MIN_AREA 50 // in pixels

class Explorer : public rclcpp::Node {
public:
  Explorer();
  ~Explorer(){};

private:
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  geometry_msgs::msg::PoseStamped drone_pose_;
  std::vector<geometry_msgs::msg::PointStamped> frontier_centroids_;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void clickedPointCallback(const geometry_msgs::msg::PointStamped point);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // EXPLORER_HPP_