#include "as2_cpp_api/service_clients/offboard.hpp"

namespace as2
{
namespace as2_cpp_api
{

Offboard::Offboard(const rclcpp::Node::SharedPtr & drone_interface_base_node)
: ServiceBoolHandler(drone_interface_base_node, "set_offboard_mode")
{

}

bool Offboard::operator()()
{
  return ServiceBoolHandler::operator()(true);
}

Manual::Manual(const rclcpp::Node::SharedPtr & drone_interface_base_node)
: ServiceBoolHandler(drone_interface_base_node, "set_offboard_mode")
{

}

bool Manual::operator()()
{
  return ServiceBoolHandler::operator()(false);
}

}
}
