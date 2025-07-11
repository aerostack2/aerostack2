#ifndef SERVICE_CLIENTS_SERVICE_HANDLER
#define SERVICE_CLIENTS_SERVICE_HANDLER

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

namespace as2
{
namespace as2_cpp_api
{

template<typename ServiceMsgT>
class ServiceHandler
{
public:
  ServiceHandler(
    const rclcpp::Node::SharedPtr & client,
    const std::string & service_name);

  typename ServiceMsgT::Response::SharedPtr operator()(
    const typename ServiceMsgT::Request::SharedPtr & request);

private:
  typename rclcpp::Client<ServiceMsgT>::SharedPtr client_;
  rclcpp::Logger logger_{rclcpp::get_logger("ServiceHandler")};
};

class ServiceBoolHandler : public ServiceHandler<std_srvs::srv::SetBool>
{
public:
  ServiceBoolHandler(
    const rclcpp::Node::SharedPtr & drone_interface_base_node,
    const std::string & service_name);

  bool operator()(
    const bool value);

private:
  rclcpp::Logger logger_{rclcpp::get_logger("ServiceBoolHandler")};
};

}
}
#endif
