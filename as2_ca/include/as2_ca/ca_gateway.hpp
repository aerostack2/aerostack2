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
 *  \file       ca_gateway.hpp
 *  \brief      Ca_gateway node header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_CA__CA_GATEWAY_HPP_
#define AS2_CA__CA_GATEWAY_HPP_

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
const char * INCOMING_TOPIC_SUFFIX = "gateway_in";
class CA_Gateway : public rclcpp::Node
{
public:
  explicit CA_Gateway(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
    * @brief Callback for inter-agent messages. Sends the received message to the corresponding local serialized topic.
    * @param msg The received inter-agent message.
   */
  void ia_message_callback(const as2_msgs::msg::InterAgentMessage::SharedPtr msg);

  /**
    * @brief Callback for registering local modules. Creates a publisher for the specified message type.
    * @param msg The register module message containing the type to register.
   */
  void register_module(
    const std::shared_ptr<as2_msgs::srv::RegisterModule::Request> request,
    std::shared_ptr<as2_msgs::srv::RegisterModule::Response> response);

  /**
    * @brief Callback for local serialized messages. Publishes the message as an inter-agent message.
    * @param msg The local serialized message.
   */
  void forward_local_message(const as2_msgs::msg::LocalGenericMessage::SharedPtr msg);

protected:
  // Subscriber to inter-agent messages
  rclcpp::Subscription<as2_msgs::msg::InterAgentMessage>::SharedPtr inter_agent_sub_;

  // Publisher to inter-agent messages
  rclcpp::Publisher<as2_msgs::msg::InterAgentMessage>::SharedPtr inter_agent_pub_;

  // Service to register local modules
  rclcpp::Service<as2_msgs::srv::RegisterModule>::SharedPtr register_module_srv_;

  // Subscriber to forward local messages to other agents
  rclcpp::Subscription<as2_msgs::msg::LocalGenericMessage>::SharedPtr outgoing_ca_sub_;

  // Registered types and their corresponding publishers. These publishers forward
  // external ia_messages to local modules
  std::unordered_map<std::string,
    rclcpp::Publisher<as2_msgs::msg::LocalGenericMessage>::SharedPtr> registered_types_;

  std::string gateway_in_topic_;
  std::string register_module_service_name_;
  std::string out_messages_topic_;
  std::string agent_id_;

  std::unordered_map<std::string,
    rclcpp::Publisher<as2_msgs::msg::InterAgentMessage>::SharedPtr> inter_agent_publishers_;

  rclcpp::TimerBase::SharedPtr peer_check_timer_;

  std::string incoming_topic_name_;
  /**
    * @brief Timer callback to check the status of peers. Removes publishers for peers that are no longer active.
  */
  void check_peers();
};
}  // namespace as2_ca

#endif  // AS2_CA__CA_GATEWAY_HPP_
