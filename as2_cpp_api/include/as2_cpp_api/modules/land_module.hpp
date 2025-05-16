#ifndef MODULES__LAND_MODULE
#define MODULES__LAND_MODULE

#include "as2_cpp_api/behavior_actions/land_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2
{
namespace as2_cpp_api
{

class LandModule
{
public:
  LandModule();
  void initialize(const rclcpp::Node::SharedPtr & drone_interface_node);
  bool operator()(const double speed, const bool wait_result = true);

private:
  std::shared_ptr<LandBehavior> land_behavior_;
};

}
}

#endif
