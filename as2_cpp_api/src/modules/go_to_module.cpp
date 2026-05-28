#include "as2_cpp_api/modules/go_to_module.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

namespace as2
{
namespace as2_cpp_api
{

GoToModule::GoToModule()
{
}

void GoToModule::initialize(const rclcpp::Node::SharedPtr & drone_interface_node)
{
  go_to_behavior_ = std::make_shared<GoToBehavior>(drone_interface_node);
}

bool GoToModule::operator()(
  const double x,
  const double y,
  const double z,
  const double speed,
  const int yaw_mode,
  const double yaw_angle,
  const bool use_yaw_angle,
  const std::string & frame_id,
  const bool wait_result)
{

  return this->__go_to(x, y, z, speed, yaw_mode, yaw_angle, use_yaw_angle, frame_id, wait_result);
}

bool GoToModule::go_to_point(
  const std::vector<double> & point,
  const double speed,
  const std::string & frame_id,
  const bool wait_result)
{
  if (point.size() != 3) {
    throw std::runtime_error("Point size must be equal to 3.");
  }
  return this->__go_to(
    point[0], point[1], point[2], speed, as2_msgs::msg::YawMode::KEEP_YAW, 0.0,
    false, frame_id, wait_result);
}

bool GoToModule::go_to_point_path_facing(
  const std::vector<double> & point,
  const double speed,
  const std::string & frame_id,
  const bool wait_result)
{
  if (point.size() != 3) {
    throw std::runtime_error("Point size must be equal to 3.");
  }
  return this->__go_to(
    point[0], point[1], point[2], speed, as2_msgs::msg::YawMode::PATH_FACING,
    0.0, false, frame_id, wait_result);
}

bool GoToModule::__go_to(
  const double x,
  const double y,
  const double z,
  const double speed,
  const int yaw_mode,
  const double yaw_angle,
  const bool use_yaw_angle,
  const std::string & frame_id,
  const bool wait_result)
{
  auto pose = geometry_msgs::msg::Pose();
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return go_to_behavior_->start(
    pose, speed, yaw_mode, yaw_angle, use_yaw_angle, frame_id,
    wait_result);
}

}
}
