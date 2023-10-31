#ifndef BUG_ALGORITHM_HPP_
#define BUG_ALGORITHM_HPP_

#include <as2_core/node.hpp>
#include <as2_motion_reference_handlers/hover_motion.hpp>
#include <as2_motion_reference_handlers/position_motion.hpp>
#include <as2_motion_reference_handlers/speed_motion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class BugAlgorithm : public as2::Node {
public:
  BugAlgorithm();
  ~BugAlgorithm(){};

  // 1. rotate 90 degrees slow enough to get a good map (sensor work at 20hz)
  //      a good spin vel would be 0.35rad/s which ensures a sensor reading each
  //      1 degree. At max sensor range, two consecutive measurementes will be
  //      at 0.07m aprox
  // 2. move in goal direction for 3m (limit of the scanned area previously
  // eroded) following a bug behaviour

  // alternative:
  // perform a bug behavior if selected point is unknown try to spin

private:
  rclcpp::CallbackGroup::SharedPtr cbk_group_;

  geometry_msgs::msg::PoseStamped drone_pose_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;

  /** Handlers **/
  as2::motionReferenceHandlers::SpeedMotion speed_handler_;
  as2::motionReferenceHandlers::PositionMotion position_handler_;
  as2::motionReferenceHandlers::HoverMotion hover_handler_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      debug_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rotate_srv_;

  void occGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  void rotateCbk(const std_srvs::srv::Trigger::Request::SharedPtr request,
                 std_srvs::srv::Trigger::Response::SharedPtr response);

  bool translate(const geometry_msgs::msg::PoseStamped &goal,
                 const double max_horiz_speed, const double max_vert_speed);
  bool rotate(const double goal_yaw, const double yaw_speed);
  double getCurrentYaw(const geometry_msgs::msg::PoseStamped &pose);
};
#endif // BUG_ALGORITHM_HPP_