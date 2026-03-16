#include "as2_cpp_api/behavior_actions/take_off_behavior.hpp"


namespace as2
{
namespace as2_cpp_api
{

TakeoffBehavior::TakeoffBehavior(const rclcpp::Node::SharedPtr & drone_node_interface)
: BehaviorHandler<as2_msgs::action::Takeoff>(drone_node_interface, "TakeoffBehavior") {}

bool TakeoffBehavior::start(const double height, const double speed, const bool wait_result)
{
  auto goal = as2_msgs::action::Takeoff::Goal();
  goal.takeoff_height = height;
  goal.takeoff_speed = speed;

  try {
    return BehaviorHandler::start(goal, wait_result);
  } catch (const std::exception & e) {
    return false;
  }
}

}
}
