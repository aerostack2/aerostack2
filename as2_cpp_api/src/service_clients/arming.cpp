#include "as2_cpp_api/service_clients/arming.hpp"

namespace as2
{
namespace as2_cpp_api
{

Arm::Arm(const rclcpp::Node::SharedPtr & drone_interface_base_node)
: ServiceBoolHandler(drone_interface_base_node, "set_arming_state")
{

}

bool Arm::operator()()
{
  return ServiceBoolHandler::operator()(true);
}

Disarm::Disarm(const rclcpp::Node::SharedPtr & drone_interface_base_node)
: ServiceBoolHandler(drone_interface_base_node, "set_arming_state")
{

}

bool Disarm::operator()()
{
  return ServiceBoolHandler::operator()(false);
}

}
}
