#include "as2_cpp_api/modules/takeoff_module.hpp"

namespace as2
{
namespace as2_cpp_api
{

TakeoffModule::TakeoffModule()
{
}

void TakeoffModule::initialize(const rclcpp::Node::SharedPtr & drone_interface_node)
{
  takeoff_behavior_ = std::make_shared<TakeoffBehavior>(drone_interface_node);
}

bool TakeoffModule::operator()(
  const double height,
  const double speed,
  const bool wait_result)
{

  return takeoff_behavior_->start(height, speed, wait_result);
}

}
}
