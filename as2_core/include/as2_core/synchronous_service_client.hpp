// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       synchronous_service_client.hpp
 *  \brief      Class for handling synchronous service clients in ROS2
 *              without taking care about the spin() method
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 ********************************************************************************/

#ifndef AS2_CORE__SYNCHRONOUS_SERVICE_CLIENT_HPP_
#define AS2_CORE__SYNCHRONOUS_SERVICE_CLIENT_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace as2
{

template<class ServiceT>
/**
 * @brief Class for handling synchronous service clients in ROS2
 *       without taking care about the spin() method
 */
class SynchronousServiceClient
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
  std::string service_name_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr_;

public:
  using SharedPtr = std::shared_ptr<SynchronousServiceClient<ServiceT>>;

  /**
   * @brief Constructor
   * @param service_name Name of the service
   * @param node Node to be used for the service client or (rclcpp::Node,as2::Node, rclcpp_lifecycle::LifecycleNode)
   */
  template<class NodeT>
  SynchronousServiceClient(const std::string & service_name, NodeT * node)
  : SynchronousServiceClient(
      service_name, node->get_node_base_interface(), node->get_node_graph_interface(),
      node->get_node_services_interface(), node->get_node_logging_interface())
  {}

  /**
   * @brief Constructor
   * @param service_name Name of the service
   * @param node_base_ptr NodeBaseInterface of the node
   * @param node_graph_ptr NodeGraphInterface of the node
   * @param node_services_ptr NodeServicesInterface of the node
   * @param node_logging_ptr NodeLoggingInterface of the node
   */
  SynchronousServiceClient(
    const std::string & service_name,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr)
  : service_name_(service_name), node_base_ptr_(node_base_ptr),
    node_services_ptr_(node_services_ptr), node_graph_ptr_(node_graph_ptr),
    node_logging_ptr_(node_logging_ptr)
  {
    callback_group_ =
      node_base_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_base_ptr_);
    service_client_ = rclcpp::create_client<ServiceT>(
      node_base_ptr_, node_graph_ptr_, node_services_ptr_, service_name,
      rmw_qos_profile_services_default, callback_group_);
  }

  /**
   * @brief Destructor
   */
  ~SynchronousServiceClient()
  {
    service_client_.reset();
    callback_group_.reset();
    callback_group_executor_.remove_callback_group(callback_group_);
  }

  /**
   * @brief Send Request synchronously to the service
   * @param request Request to be sent to the service
   * @param respone Response received from the service
   * @param wait_time Time to wait for the service to be available in seconds
   * @return True if the service was called successfully, false otherwise
   */

  bool sendRequest(const RequestT & req, ResponseT & resp, int wait_time = 0)
  {
    auto resp_ptr = std::make_shared<ResponseT>(resp);
    if (!sendRequest(std::make_shared<RequestT>(req), resp_ptr, wait_time)) {
      return false;
    }
    resp = *resp_ptr.get();
    return true;
  }

  /**
   * @brief Send Request synchronously to the service
   * @param request Request to be sent to the service
   * @param respone Response received from the service
   * @param wait_time Time to wait for the service to be available in seconds
   * @return True if the service was called successfully, false otherwise
   */
  bool sendRequest(
    const std::shared_ptr<RequestT> & req, std::shared_ptr<ResponseT> & resp, int wait_time = 0)
  {
    if (wait_time <= 0) {
      while (!service_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            node_logging_ptr_->get_logger(),
            "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_INFO(
          node_logging_ptr_->get_logger(), "service: %s not available, waiting again...",
          service_name_.c_str());
      }
    } else {
      if (!service_client_->wait_for_service(std::chrono::seconds(wait_time))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            node_logging_ptr_->get_logger(),
            "interrupted while waiting for the service. exiting.");
          return false;
        }
        RCLCPP_WARN(
          node_logging_ptr_->get_logger(), "service: %s not available, returning False ",
          service_name_.c_str());
        return false;
      }
    }

    auto result = service_client_->async_send_request(req);
    if (
      callback_group_executor_.spin_until_future_complete(result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        node_logging_ptr_->get_logger(), "failed to receive response from service '%s'",
        service_name_.c_str());
      return false;
    }

    resp = result.get();
    return true;
  }
};

}  // namespace as2
#endif  // AS2_CORE__SYNCHRONOUS_SERVICE_CLIENT_HPP_
