#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <unordered_set>
#include <vector>

#include "A_star_algorithm.hpp"
#include "path_optimizer.hpp"
#include "utils.hpp"
#include "viz_utils.hpp"

class PathPlanner : public rclcpp::Node {
public:
  PathPlanner();
  ~PathPlanner(){};

private:
  AStarPlanner planner_algorithm_;
  geometry_msgs::msg::PoseStamped drone_pose_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  bool use_path_optimizer_ = false;
  double safety_distance_ = 1.0; // aprox drone size [m]

  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void occGridCbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  // Helpers
  void callFollowPathAction(std::vector<geometry_msgs::msg::Point> points);
  std::vector<cv::Point2i> safeZone(cv::Point2i drone_cell, int iterations);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      planner_goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_pub_;

  // TODO: temporal while not coding the planner: path -> optimized_path ->
  // trajectory
  rclcpp_action::Client<as2_msgs::action::FollowPath>::SharedPtr
      follow_path_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // PATH_PLANNER_HPP_