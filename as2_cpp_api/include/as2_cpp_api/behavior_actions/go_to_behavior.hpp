#ifndef BEHAVIOR_ACTIONS__GO_TO_BEHAVIOR
#define BEHAVIOR_ACTIONS__GO_TO_BEHAVIOR

#include "as2_cpp_api/behavior_actions/behavior_handler.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"

#include "cppmap3d/cppmap3d.hpp"

namespace as2
{
namespace as2_cpp_api
{

class GoToBehavior : public BehaviorHandler<as2_msgs::action::GoToWaypoint>
{

public:
  GoToBehavior(const rclcpp::Node::SharedPtr & drone_node_interface_);

  bool start(
    const geometry_msgs::msg::Pose & pose,
    const double speed,
    const int yaw_mode,
    const double yaw_angle,
    const bool use_yaw_angle,
    const std::string & frame_id,
    const bool wait_result);

  bool start(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const double speed,
    const int yaw_mode,
    const double yaw_angle,
    const bool use_yaw_angle,
    const std::string & frame_id,
    const bool wait_result);

  // bool start(
  //     const geographic_msgs::msg::GeoPose & geo_pose,
  //     const double speed,
  //     const int yaw_mode,
  //     const double yaw_angle,
  //     const std::string & frame_id,
  //     const bool wait_result);

  // bool start(
  //     const geographic_msgs::msg::GeoPoseStamped & geo_pose_stamped,
  //     const double speed,
  //     const int yaw_mode,
  //     const double yaw_angle,
  //     const std::string & frame_id,
  //     const bool wait_result);

};

}
}

#endif
