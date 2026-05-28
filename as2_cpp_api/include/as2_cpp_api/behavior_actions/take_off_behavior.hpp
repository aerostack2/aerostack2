#ifndef BEHAVIOR_ACTIONS__TAKE_OFF_BEHAVIOR
#define BEHAVIOR_ACTIONS__TAKE_OFF_BEHAVIOR

#include "as2_cpp_api/behavior_actions/behavior_handler.hpp"
#include "as2_msgs/action/takeoff.hpp"

namespace as2
{
namespace as2_cpp_api
{

class TakeoffBehavior : public BehaviorHandler<as2_msgs::action::Takeoff>
{

public:
  TakeoffBehavior(const rclcpp::Node::SharedPtr & drone_node_interface_);
  bool start(const double height, const double speed, const bool wait_result);
};

}
}

#endif
