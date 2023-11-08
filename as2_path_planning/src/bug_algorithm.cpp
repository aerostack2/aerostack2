#include "frontier_utils.hpp"
#include <bug_algorithm.hpp>

BugAlgorithm::BugAlgorithm()
    : Node("bug_algorithm"), speed_handler_(this), position_handler_(this),
      hover_handler_(this) {
  RCLCPP_INFO(this->get_logger(), "BUG ALGO");

  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&BugAlgorithm::dronePoseCbk, this, std::placeholders::_1));

  rclcpp::SubscriptionOptions options;
  cbk_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cbk_group_;
  debug_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          std::bind(&BugAlgorithm::clickedPointCallback, this,
                    std::placeholders::_1),
          options);

  rotate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "rotate",
      std::bind(&BugAlgorithm::rotateCbk, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cbk_group_);
}

void BugAlgorithm::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_occ_grid_ = *(msg);
}

void BugAlgorithm::dronePoseCbk(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  drone_pose_ = *(msg);
}

void BugAlgorithm::clickedPointCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr point) {
  RCLCPP_INFO(this->get_logger(), "clicked point: %lf, %lf, %lf",
              point->point.x, point->point.y, point->point.z);

  double goal_yaw = atan2(point->point.y - drone_pose_.pose.position.y,
                          point->point.x - drone_pose_.pose.position.x);

  double dist = 3.5;
  while (utils::distance(drone_pose_.pose.position, point->point) > 0.5) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = point->header;
    goal.pose.position.x = drone_pose_.pose.position.x + dist * cos(goal_yaw);
    goal.pose.position.y = drone_pose_.pose.position.y + dist * sin(goal_yaw);
    goal.pose.position.z = drone_pose_.pose.position.z;

    double yaw = getCurrentYaw(drone_pose_);
    rotate(yaw + M_PI / 2, 0.15);

    translate(goal, 0.5, 0.3);

    RCLCPP_INFO(this->get_logger(), "NAVIGATE FINISHED");
  }
}

void BugAlgorithm::rotateCbk(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {

  double yaw = getCurrentYaw(drone_pose_);
  double goal_yaw = yaw + M_PI / 2 + 0.02; // +91 degrees
  goal_yaw = goal_yaw > M_PI / 2 ? goal_yaw - M_PI : goal_yaw;
  goal_yaw = goal_yaw < -M_PI / 2 ? goal_yaw + M_PI : goal_yaw;

  bool ret = rotate(goal_yaw, 0.3);

  RCLCPP_INFO(this->get_logger(), "Current yaw: %lf", yaw);
  RCLCPP_INFO(this->get_logger(), "ROTATE FINISHED");
  response->success = ret;
  response->message = "yaw: " + std::to_string(goal_yaw);
}

bool BugAlgorithm::translate(const geometry_msgs::msg::PoseStamped &goal,
                             const double max_horiz_speed,
                             const double max_vert_speed) {
  geometry_msgs::msg::TwistStamped speed_limit;
  speed_limit.header.frame_id = "drone0/base_link";
  speed_limit.header.stamp = this->now();
  speed_limit.twist.linear.x = max_horiz_speed;
  speed_limit.twist.linear.y = max_horiz_speed;
  speed_limit.twist.linear.z = max_vert_speed;
  speed_limit.twist.angular.x = 0.0;
  speed_limit.twist.angular.y = 0.0;
  speed_limit.twist.angular.z = 0.0;

  bool ret =
      position_handler_.sendPositionCommandWithYawSpeed(goal, speed_limit);
  while (utils::distance(drone_pose_.pose.position, goal.pose.position) > 0.5) {
    ret = position_handler_.sendPositionCommandWithYawSpeed(goal, speed_limit);
  }
  bool ret2 = hover_handler_.sendHover();
  return ret && ret2;
}

bool BugAlgorithm::rotate(const double goal_yaw, const double yaw_speed) {
  double yaw = getCurrentYaw(drone_pose_);
  double diff = std::abs(goal_yaw - yaw);
  double speed = diff > M_PI ? -yaw_speed : yaw_speed;
  bool ret1 = speed_handler_.sendSpeedCommandWithYawSpeed("drone0/base_link", 0,
                                                          0, 0, speed);

  while (std::abs(yaw - goal_yaw) > 0.05) {
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    yaw = getCurrentYaw(drone_pose_);
  }

  bool ret2 = hover_handler_.sendHover();
  return ret1 && ret2;
}

double
BugAlgorithm::getCurrentYaw(const geometry_msgs::msg::PoseStamped &pose) {
  double roll, pitch, yaw, goal_yaw;

  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  yaw = yaw > M_PI / 2 ? yaw - M_PI : yaw;
  yaw = yaw < -M_PI / 2 ? yaw + M_PI : yaw;
  return yaw;
}
