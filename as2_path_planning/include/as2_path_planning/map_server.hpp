#ifndef MAP_SERVER_HPP_
#define MAP_SERVER_HPP_

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "A_star_algorithm.hpp"
#include "utils.hpp"

class MapServer : public rclcpp::Node {
public:
  MapServer();
  ~MapServer(){};

private:
  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void clickedPointCallback(const geometry_msgs::msg::PointStamped point);
  void showMapCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                       std_srvs::srv::Empty::Response::SharedPtr response);

  void callFollowPathAction(std::vector<geometry_msgs::msg::Point> points);

  // Helpers
  void showMap(const cv::Mat &mat, std::string window_name = "Map");

  std::vector<cv::Point2i> safeZone(cv::Point2i drone_cell,
                                    double grid_resolution);

  // Attributes
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr show_map_serv_;

  nav_msgs::msg::OccupancyGrid::SharedPtr last_occ_grid_;
  geometry_msgs::msg::PoseStamped drone_pose_;

  // TODO: temporal while not coding the planner: path -> optimized_path ->
  // trajectory
  rclcpp_action::Client<as2_msgs::action::FollowPath>::SharedPtr
      follow_path_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  AStarPlanner planner_algorithm_;
};
#endif // MAP_SERVER_HPP_