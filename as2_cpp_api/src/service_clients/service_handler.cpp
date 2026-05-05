#include "as2_cpp_api/service_clients/service_handler.hpp"

namespace as2
{
namespace as2_cpp_api
{

template<typename ServiceMsgT>
ServiceHandler<ServiceMsgT>::ServiceHandler(
  const rclcpp::Node::SharedPtr & drone_interface_base_node,
  const std::string & service_name)
{
  logger_ = drone_interface_base_node->get_logger();
  try {
    client_ = drone_interface_base_node->create_client<ServiceMsgT>(service_name);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Coud not create client for %s", service_name.c_str());
    throw e;
  }

  using namespace std::chrono_literals;
  if (!client_->wait_for_service(1s)) {
    RCLCPP_ERROR(logger_, "%s not available", client_->get_service_name());
  }
}

template<typename ServiceMsgT>
typename ServiceMsgT::Response::SharedPtr ServiceHandler<ServiceMsgT>::operator()(
  const typename ServiceMsgT::Request::SharedPtr & request)
{

  auto response_future = client_->async_send_request(request);

  if (response_future.valid()) {
    return response_future.get();
  } else {
    throw std::runtime_error("Future is not valid");
  }

  // Wait for the result.
  // if (rclcpp::spin_until_future_complete(node_, response_future) !=
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //     return false;
  // }

}

ServiceBoolHandler::ServiceBoolHandler(
  const rclcpp::Node::SharedPtr & drone_interface_base_node,
  const std::string & service_name)
: ServiceHandler<std_srvs::srv::SetBool>(drone_interface_base_node, service_name)
{
  logger_ = drone_interface_base_node->get_logger();
}

bool ServiceBoolHandler::operator()(
  const bool value)
{

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = value;

  try {
    auto response = ServiceHandler<std_srvs::srv::SetBool>::operator()(request);
    if (!response->success) {
      RCLCPP_ERROR(logger_, "Service returned failure");
    }

    return response->success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Service returned failure: Unknown Error");
    return false;
  }
}

}
}
