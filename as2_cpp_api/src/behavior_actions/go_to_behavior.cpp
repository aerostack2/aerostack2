#include "as2_cpp_api/behavior_actions/go_to_behavior.hpp"

namespace as2
{
namespace as2_cpp_api
{

GoToBehavior::GoToBehavior(const rclcpp::Node::SharedPtr & drone_node_interface)
: BehaviorHandler<as2_msgs::action::GoToWaypoint>(drone_node_interface, "GoToBehavior") {}

bool GoToBehavior::start(
  const geometry_msgs::msg::Pose & pose,
  const double speed,
  const int yaw_mode,
  const double yaw_angle,
  const bool use_yaw_angle,
  const std::string & frame_id,
  const bool wait_result)
{
  auto goal = as2_msgs::action::GoToWaypoint::Goal();
  goal.target_pose.header.stamp = node_->now();
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.point.x = pose.position.x;
  goal.target_pose.point.y = pose.position.y;
  goal.target_pose.point.z = pose.position.z;

  goal.max_speed = speed;
  goal.yaw.mode = yaw_mode;

  // TODO: Why ?????
  if (use_yaw_angle) {
    goal.yaw.angle = yaw_angle;
  }

  try {
    return BehaviorHandler::start(goal, wait_result);
  } catch (const std::exception & e) {
    return false;
  }
}

bool GoToBehavior::start(
  const geometry_msgs::msg::PoseStamped & pose_stamped,
  const double speed,
  const int yaw_mode,
  const double yaw_angle,
  const bool use_yaw_angle,
  const std::string & frame_id,
  const bool wait_result)
{
  return this->start(
    pose_stamped, speed, yaw_mode, yaw_angle, use_yaw_angle, frame_id,
    wait_result);
}

}
}
