#ifndef DRONE_WATCHER_HPP_
#define DRONE_WATCHER_HPP_

#include "rclcpp_action/rclcpp_action.hpp"
#include <as2_core/names/actions.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_msgs/action/follow_path.hpp>
#include <as2_msgs/action/go_to_waypoint.hpp>
#include <as2_msgs/msg/behavior_status.hpp>
#include <as2_msgs/msg/yaw_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class DroneWatcher {
public:
  DroneWatcher(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
          node_parameters_ptr,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
          node_services_ptr,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr,
      rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
          node_waitables_ptr,
      std::string drone_id);
  ~DroneWatcher(){};

private:
  using GoTo = as2_msgs::action::GoToWaypoint;
  using GoalHandleGoTo = rclcpp_action::ClientGoalHandle<GoTo>;

public:
  geometry_msgs::msg::PoseStamped drone_pose_;
  uint8_t traj_gen_status = as2_msgs::msg::BehaviorStatus::IDLE;
  as2_msgs::action::FollowPath_FeedbackMessage follow_path_feedback_;

  bool stop();
  bool resume();
  void moveVertically(double z_diff);

  void avoidanceManeuver();
  void backToFollowPath();

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
      node_parameters_ptr_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
      node_waitables_ptr_;
  std::string drone_id_;

  rclcpp::CallbackGroup::SharedPtr cbk_group_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pose_sub_;
  rclcpp::Subscription<as2_msgs::msg::BehaviorStatus>::SharedPtr
      traj_gen_behavior_status_sub_;
  rclcpp::Subscription<as2_msgs::action::FollowPath_FeedbackMessage>::SharedPtr
      follow_path_feedback_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;

  rclcpp_action::Client<GoTo>::SharedPtr go_to_client_;

  void dronePoseCbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void
  trajGenBehaviorStatusCbk(const as2_msgs::msg::BehaviorStatus::SharedPtr msg);
  void followPathFeedbackCbk(
      const as2_msgs::action::FollowPath_FeedbackMessage::SharedPtr msg);
};

#endif // SAFEGUARD_HPP_