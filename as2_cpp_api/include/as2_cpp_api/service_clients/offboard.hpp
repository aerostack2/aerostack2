#ifndef SERVICE_CLIENTS_OFFBOARD
#define SERVICE_CLIENTS_OFFBOARD

#include "as2_cpp_api/service_clients/service_handler.hpp"

namespace as2
{
namespace as2_cpp_api
{

class Offboard : public ServiceBoolHandler
{
public:
  Offboard(const rclcpp::Node::SharedPtr & drone_interface_base_node);
  bool operator()();
};

class Manual : public ServiceBoolHandler
{
public:
  Manual(const rclcpp::Node::SharedPtr & drone_interface_base_node);
  bool operator()();
};

}
}
#endif
