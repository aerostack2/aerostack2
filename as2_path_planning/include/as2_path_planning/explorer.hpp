#ifndef EXPLORER_HPP_
#define EXPLORER_HPP_

#include <algorithm> // std::sort
#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_motion_reference_handlers/hover_motion.hpp>
#include <as2_motion_reference_handlers/position_motion.hpp>
#include <as2_motion_reference_handlers/speed_motion.hpp>
#include <as2_msgs/action/navigate_to_point.hpp>
#include <as2_msgs/msg/yaw_mode.hpp>
#include <as2_msgs/srv/allocate_frontier.hpp>
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

class Explorer : public as2::Node {
public:
  using NavigateToPoint = as2_msgs::action::NavigateToPoint;
  using GoalHandleNavigateToPoint =
      rclcpp_action::ClientGoalHandle<NavigateToPoint>;

  Explorer();
  ~Explorer(){};

private:
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  geometry_msgs::msg::PoseStamped drone_pose_;
  double safety_distance_ = 1.0;     // [m]
  double reached_dist_thresh_ = 0.5; // [m]
  double spin_yaw_thresh_ = 0.05;    // [rad]
  double navigation_speed_ = 1.0;    // [m/s]
  bool cautiously_ = false;
  double spin_speed_ = 0.15; // [rad/s]

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  int processGoal(geometry_msgs::msg::PointStamped goal);
  int explore(geometry_msgs::msg::PointStamped goal);
  int navigateTo(geometry_msgs::msg::PointStamped goal,
                 uint8_t yaw_mode = as2_msgs::msg::YawMode::PATH_FACING);

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
  as2_msgs::srv::AllocateFrontier::Response::SharedPtr
  getFrontier(const geometry_msgs::msg::PoseStamped &goal);
  as2_msgs::srv::AllocateFrontier::Response::SharedPtr
  getFrontier(const geometry_msgs::msg::PointStamped &goal);

  bool rotate(const double goal_yaw, const double yaw_speed);
  double getCurrentYaw(const geometry_msgs::msg::PoseStamped &pose);

  /** Handlers **/
  as2::motionReferenceHandlers::SpeedMotion speed_handler_;
  as2::motionReferenceHandlers::PositionMotion position_handler_;
  as2::motionReferenceHandlers::HoverMotion hover_handler_;

  rclcpp::CallbackGroup::SharedPtr cbk_group_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_explore_srv_;
  rclcpp::Client<as2_msgs::srv::AllocateFrontier>::SharedPtr ask_frontier_cli_;

  rclcpp_action::Client<NavigateToPoint>::SharedPtr navigation_action_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // EXPLORER_HPP_