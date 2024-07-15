#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <vector>

#include "A_star_algorithm.hpp"
#include "path_optimizer.hpp"
#include "utils.hpp"
#include "viz_utils.hpp"

class PathPlanner : public rclcpp::Node {
public:
  using NavigateToPoint = as2_msgs::action::NavigateToPoint;
  using GoalHandleNavigateToPoint =
      rclcpp_action::ServerGoalHandle<NavigateToPoint>;
  using FollowPath = as2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  PathPlanner();
  ~PathPlanner(){};

private:
  AStarPlanner planner_algorithm_;
  geometry_msgs::msg::PoseStamped drone_pose_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  bool use_path_optimizer_ = false;
  double safety_distance_ = 1.0; // aprox drone size [m]
  std::vector<geometry_msgs::msg::Point> path_;
  std::shared_ptr<GoalHandleNavigateToPoint> navigation_goal_handle_;
  std::thread execution_thread_;

  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void occGridCbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // NavigateToPoint Action Server
  void
  navigateToPoint(const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
  rclcpp_action::GoalResponse
  navigationGoalCbk(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const NavigateToPoint::Goal> goal);
  rclcpp_action::CancelResponse navigationCancelCbk(
      const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);
  void navigationAcceptedCbk(
      const std::shared_ptr<GoalHandleNavigateToPoint> goal_handle);

  // FollowPath Action Client
  void
  followPathResponseCbk(const GoalHandleFollowPath::SharedPtr &goal_handle);
  void followPathFeedbackCbk(
      GoalHandleFollowPath::SharedPtr goal_handle,
      const std::shared_ptr<const FollowPath::Feedback> feedback);
  void followPathResultCbk(const GoalHandleFollowPath::WrappedResult &result);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_pub_;

  rclcpp_action::Server<NavigateToPoint>::SharedPtr navigation_action_server_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
#endif // PATH_PLANNER_HPP_