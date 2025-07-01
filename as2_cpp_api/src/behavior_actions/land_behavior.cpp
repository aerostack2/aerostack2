#include "as2_cpp_api/behavior_actions/land_behavior.hpp"


namespace as2
{
namespace as2_cpp_api
{

LandBehavior::LandBehavior(const rclcpp::Node::SharedPtr & drone_node_interface)
: BehaviorHandler<as2_msgs::action::Land>(drone_node_interface, "LandBehavior") {}

bool LandBehavior::start(const double speed, const bool wait_result)
{
  auto goal = as2_msgs::action::Land::Goal();
  goal.land_speed = speed;

  try {
    return BehaviorHandler::start(goal, wait_result);
  } catch (const std::exception & e) {
    return false;
  }
}

}
}
