// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       ca_gateway_client.hpp
 *  \brief      Ca_gateway_client node header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_CA__CA_GATEWAY_CLIENT_HPP_
#define AS2_CA__CA_GATEWAY_CLIENT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "as2_msgs/msg/local_generic_message.hpp"
#include "as2_msgs/msg/inter_agent_message.hpp"
#include "as2_msgs/srv/register_module.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/create_generic_publisher.hpp"
#include "rclcpp/generic_publisher.hpp"

using std::placeholders::_1;

namespace as2_ca
{
class CAGatewayClient
{
public:
  CAGatewayClient() = default;
  explicit CAGatewayClient(rclcpp::Node * parent);
  ~CAGatewayClient();

  void clear();

  // Function to call module registration service
  template<typename T>
  void register_module(
    const std::string & type, const std::string & module_name,
    std::function<void(const T &, const std::string &)> callback)
  {
    auto logger = parent_->get_logger();
    try {
      // Create a request for the register_module service
      auto request_msg = std::make_shared<as2_msgs::srv::RegisterModule::Request>();
      request_msg->type = type;
      request_msg->module_name = module_name;

      RCLCPP_INFO(
        logger, "Registering module: %s with type: %s", module_name.c_str(),
        type.c_str());

      register_module_client_->async_send_request(
        request_msg,
        [this, module_name, callback](
          rclcpp::Client<as2_msgs::srv::RegisterModule>::SharedFuture future)
        {
          RCLCPP_INFO(
            parent_->get_logger(), "Received response for module registration: %s",
            module_name.c_str());
          auto response = future.get();
          if (response->topic.empty()) {
            RCLCPP_ERROR(
              parent_->get_logger(), "Failed to register module: %s",
              module_name.c_str());
            return;
          }
          RCLCPP_INFO(
            parent_->get_logger(), "Module registered successfully: %s, topic: %s",
            module_name.c_str(), response->topic.c_str());
          subscribe_to_local_generic<T>(response->topic, callback);
          RCLCPP_INFO(
            parent_->get_logger(), "Subscribed to local topic: %s for module: %s",
            response->topic.c_str(), module_name.c_str());
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Exception in register_module handler: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(logger, "Unknown exception in register_module handler");
    }
  }

  int get_subscriber_count();

  std::vector<std::string> get_known_peers();

  template<typename T>
  void forward_IA_msg(
    const T & msg, const std::string & type, const std::vector<std::string> & receivers)
  {
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&msg, &serialized_msg);

    as2_msgs::msg::LocalGenericMessage generic_msg;
    generic_msg.agents = receivers;
    generic_msg.type = type;
    const auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
    generic_msg.data.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
    forwarder_pub_->publish(generic_msg);
  }

private:
  // Subscriber for local_generic messages
  std::vector<rclcpp::Subscription<as2_msgs::msg::LocalGenericMessage>::SharedPtr>
  local_generic_subscribers_;

  rclcpp::Client<as2_msgs::srv::RegisterModule>::SharedPtr register_module_client_;

  std::string agent_id_;

  template<typename T>
  void subscribe_to_local_generic(
    const std::string & generic_topic_name, std::function<void(const T &,
    const std::string &)> callback)
  {
    // Create a subscription to the local generic topic
    auto subscription = parent_->create_subscription<as2_msgs::msg::LocalGenericMessage>(
      generic_topic_name, 10, [callback](
        const as2_msgs::msg::LocalGenericMessage::SharedPtr msg) {
        T deserialized_msg;

        // Construct SerializedMessage from the raw serialized bytes field
        rclcpp::SerializedMessage serialized_msg(msg->data.size());
        auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
        std::memcpy(rcl_msg.buffer, msg->data.data(), msg->data.size());
        rcl_msg.buffer_length = msg->data.size();

        rclcpp::Serialization<T> serializer;
        serializer.deserialize_message(&serialized_msg, &deserialized_msg);
        callback(deserialized_msg, msg->agents[0]);
      });

    // Store the subscription to keep it alive
    local_generic_subscribers_.push_back(subscription);
  }
  rclcpp::Node * parent_;


  rclcpp::Publisher<as2_msgs::msg::LocalGenericMessage>::SharedPtr forwarder_pub_;
};

}  // namespace as2_ca
#endif  // AS2_CA__CA_GATEWAY_CLIENT_HPP_
