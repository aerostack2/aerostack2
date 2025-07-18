#ifndef SERVICE_CLIENTS_ARMING
#define SERVICE_CLIENTS_ARMING

#include "as2_cpp_api/service_clients/service_handler.hpp"

namespace as2
{
namespace as2_cpp_api
{

class Arm : public ServiceBoolHandler
{
public:
  Arm(const rclcpp::Node::SharedPtr & drone_interface_base_node);
  bool operator()();
};

class Disarm : public ServiceBoolHandler
{
public:
  Disarm(const rclcpp::Node::SharedPtr & drone_interface_base_node);
  bool operator()();
};

}
}
#endif
