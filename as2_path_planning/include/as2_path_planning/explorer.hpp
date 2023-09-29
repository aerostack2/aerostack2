#ifndef EXPLORER_HPP_
#define EXPLORER_HPP_

#include <as2_core/names/topics.hpp>
#include <as2_msgs/action/navigate_to_point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "utils.hpp"
#include "viz_utils.hpp"

#define FRONTIER_MIN_AREA 20 // in pixels
#define SAFETY_DISTANCE 0.3

class Explorer : public rclcpp::Node {
public:
  using NavigateToPoint = as2_msgs::action::NavigateToPoint;
  using GoalHandleNavigateToPoint =
      rclcpp_action::ClientGoalHandle<NavigateToPoint>;

  Explorer();
  ~Explorer(){};

private:
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  geometry_msgs::msg::PoseStamped drone_pose_;
  geometry_msgs::msg::PointStamped goal_;
  std::vector<geometry_msgs::msg::PointStamped> frontier_centroids_;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  int processGoal(geometry_msgs::msg::PointStamped goal);
  void
  getFrontiers(const nav_msgs::msg::OccupancyGrid &occ_grid,
               std::vector<geometry_msgs::msg::PointStamped> &centroidsOutput,
               std::vector<cv::Mat> &frontiersOutput);
  void explore(geometry_msgs::msg::PointStamped goal);
  void navigateTo(geometry_msgs::msg::PointStamped goal);
  void visualizeFrontiers(
      const std::vector<geometry_msgs::msg::PointStamped> &centroids,
      const std::vector<cv::Mat> &frontiers);

  // Navigation To Point Action Client
  void navigationResponseCbk(
      const GoalHandleNavigateToPoint::SharedPtr &goal_handle);
  void navigationFeedbackCbk(
      GoalHandleNavigateToPoint::SharedPtr goal_handle,
      const std::shared_ptr<const NavigateToPoint::Feedback> feedback);
  void
  navigationResultCbk(const GoalHandleNavigateToPoint::WrappedResult &result);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  rclcpp_action::Client<NavigateToPoint>::SharedPtr navigation_action_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // EXPLORER_HPP_