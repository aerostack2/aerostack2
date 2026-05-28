#ifndef BEHAVIOR_ACTIONS__LAND_BEHAVIOR
#define BEHAVIOR_ACTIONS__LAND_BEHAVIOR

#include "as2_cpp_api/behavior_actions/behavior_handler.hpp"
#include "as2_msgs/action/land.hpp"

namespace as2
{
namespace as2_cpp_api
{

class LandBehavior : public BehaviorHandler<as2_msgs::action::Land>
{

public:
  LandBehavior(const rclcpp::Node::SharedPtr & drone_node_interface_);
  bool start(const double speed, const bool wait_result);
};

}
}

#endif
