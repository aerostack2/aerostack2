#ifndef EXPLORER_HPP_
#define EXPLORER_HPP_

#include <algorithm> // std::sort
#include <as2_core/names/topics.hpp>
#include <as2_msgs/action/navigate_to_point.hpp>
#include <cmath> // std::acos..
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector> // std::vector
#include <visualization_msgs/msg/marker.hpp>

#include "frontier_utils.hpp"
#include "utils.hpp"
#include "viz_utils.hpp"

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
  int frontier_min_area_ = 1;        // in pixels
  int frontier_max_area_ = 25;       // in pixels
  double safety_distance_ = 1.0;     // [m]
  double reached_dist_thresh_ = 0.5; // [m]

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  int processGoal(geometry_msgs::msg::PointStamped goal);
  void
  getFrontiers(const nav_msgs::msg::OccupancyGrid &occ_grid,
               std::vector<geometry_msgs::msg::PointStamped> &centroidsOutput,
               std::vector<cv::Mat> &frontiersOutput);
  void
  splitFrontier(const cv::Mat &frontier, int n_parts,
                std::vector<geometry_msgs::msg::PointStamped> &centroidsOutput,
                std::vector<cv::Mat> &frontiersOutput);
  void splitFrontierSnake(
      const cv::Mat &frontier, int n_parts,
      std::vector<geometry_msgs::msg::PointStamped> &centroidsOutput,
      std::vector<cv::Mat> &frontiersOutput);
  int explore(geometry_msgs::msg::PointStamped goal);
  int navigateTo(geometry_msgs::msg::PointStamped goal);
  void visualizeFrontiers(
      const std::vector<geometry_msgs::msg::PointStamped> &centroids,
      const std::vector<cv::Mat> &frontiers);
  // TODO: temporal
  std::vector<geometry_msgs::msg::PointStamped> filterCentroids(
      const nav_msgs::msg::OccupancyGrid &occ_grid,
      const std::vector<geometry_msgs::msg::PointStamped> &centroids);

  // Navigation To Point Action Client
  void navigationResponseCbk(
      const GoalHandleNavigateToPoint::SharedPtr &goal_handle);
  void navigationFeedbackCbk(
      GoalHandleNavigateToPoint::SharedPtr goal_handle,
      const std::shared_ptr<const NavigateToPoint::Feedback> feedback);
  void
  navigationResultCbk(const GoalHandleNavigateToPoint::WrappedResult &result);

  void startExplorationCbk(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_explore_srv_;

  rclcpp_action::Client<NavigateToPoint>::SharedPtr navigation_action_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // EXPLORER_HPP_