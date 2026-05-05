#ifndef MODULES__TAKEOFF_MODULE
#define MODULES__TAKEOFF_MODULE

#include "as2_cpp_api/behavior_actions/take_off_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2
{
namespace as2_cpp_api
{

class TakeoffModule
{
public:
  TakeoffModule();
  void initialize(const rclcpp::Node::SharedPtr & drone_interface_node);
  bool operator()(const double height, const double speed, const bool wait_result = true);

private:
  std::shared_ptr<TakeoffBehavior> takeoff_behavior_;
};

}
}

#endif
