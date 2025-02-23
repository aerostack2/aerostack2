#include "as2_cpp_api/modules/land_module.hpp"

namespace as2
{
namespace as2_cpp_api
{

LandModule::LandModule()
{
}

void LandModule::initialize(const rclcpp::Node::SharedPtr & drone_interface_node)
{
  land_behavior_ = std::make_shared<LandBehavior>(drone_interface_node);
}

bool LandModule::operator()(
  const double speed,
  const bool wait_result)
{

  return land_behavior_->start(speed, wait_result);
}

}
}
