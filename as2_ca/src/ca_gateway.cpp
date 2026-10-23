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
 *  \file       ca_gateway.cpp
 *  \brief      Ca_gateway node implementation
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "as2_ca/ca_gateway.hpp"

namespace as2_ca
{

CA_Gateway::CA_Gateway(const rclcpp::NodeOptions & options)
: Node("ca_gateway", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting CA Gateway Node...");

  try {
    this->declare_parameter("out_messages_topic", "gateway_out");
    this->get_parameter("out_messages_topic", out_messages_topic_);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter declaration error: %s", e.what());
  }

  // Derive agent_id from the ROS namespace (e.g. "/drone1" → "drone1").
  // Never use a hardcoded parameter — the namespace is set via --ros-args -r __ns:=...
  {
    const std::string ns = this->get_namespace();
    agent_id_ = (!ns.empty() && ns.front() == '/') ? ns.substr(1) : ns;
  }
  RCLCPP_INFO(this->get_logger(), "CA Gateway agent_id: '%s'", agent_id_.c_str());

  try {
    this->declare_parameter("register_module_service_name", "register_module");
    this->get_parameter("register_module_service_name", register_module_service_name_);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter declaration/getting error: %s", e.what());
  }

  incoming_topic_name_ = std::string(this->get_namespace()) + "/" + INCOMING_TOPIC_SUFFIX;

  // Timer to check peer status
  peer_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CA_Gateway::check_peers, this));

  // Subscriber to own incoming topic (per-agent, replaces shared inter_agent_topic)
  inter_agent_sub_ = this->create_subscription<as2_msgs::msg::InterAgentMessage>(
    incoming_topic_name_, 10,
    std::bind(&CA_Gateway::ia_message_callback, this, std::placeholders::_1));

  // Service to register local modules
  register_module_srv_ = this->create_service<as2_msgs::srv::RegisterModule>(
    register_module_service_name_,
    std::bind(&CA_Gateway::register_module, this, std::placeholders::_1, std::placeholders::_2));

  // Subscriber to forward local messages to other agents
  outgoing_ca_sub_ = this->create_subscription<as2_msgs::msg::LocalGenericMessage>(
    out_messages_topic_, 10,
    std::bind(&CA_Gateway::forward_local_message, this, std::placeholders::_1));

  registered_types_ = std::unordered_map<std::string,
      rclcpp::Publisher<as2_msgs::msg::LocalGenericMessage>::SharedPtr>();

  RCLCPP_INFO(this->get_logger(), "CA Gateway Node started.");

  check_peers();   // Initial peer discovery on startup
}

void CA_Gateway::register_module(
  const std::shared_ptr<as2_msgs::srv::RegisterModule::Request> request,
  std::shared_ptr<as2_msgs::srv::RegisterModule::Response> response)
{
  const std::string & type = request->type;
  const std::string & module_name = request->module_name;

  auto logger = this->get_logger();
  RCLCPP_INFO(
    logger, "Received registration request from module %s for type %s",
    module_name.c_str(), type.c_str());

  if (registered_types_.find(type) == registered_types_.end()) {
    std::string topic_name = type + "_in";

    auto publisher = this->create_publisher<as2_msgs::msg::LocalGenericMessage>(
      topic_name,
      rclcpp::QoS(10));

    registered_types_[type] = publisher;

    RCLCPP_INFO(
      this->get_logger(), "Created new publisher of messages with type %s",
      type.c_str());

    response->topic = topic_name;

  } else {
    auto pub = registered_types_[type];
    RCLCPP_INFO(
      this->get_logger(), "Module %s already registered for type %s, returning topic %s",
      module_name.c_str(), type.c_str(), response->topic.c_str());
    response->topic = pub->get_topic_name();

  }
}

// Forward local messages to other agents as inter-agent messages
void CA_Gateway::forward_local_message(
  const as2_msgs::msg::LocalGenericMessage::SharedPtr msg)
{
  const std::vector<std::string> & receivers = msg->agents;
  as2_msgs::msg::InterAgentMessage ia_msg;
  ia_msg.sender = agent_id_;
  ia_msg.type = msg->type;
  ia_msg.data = msg->data;

  for (const auto & receiver : receivers) {
    if (receiver == agent_id_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Skipping self-forwarding for agent '%s'", receiver.c_str());
      continue;
    }
    auto it = inter_agent_publishers_.find(receiver);
    if (it == inter_agent_publishers_.end()) {
      RCLCPP_WARN(
        this->get_logger(), "No peer publisher found for agent %s. Message dropped.",
        receiver.c_str());
      continue;
    }
    it->second->publish(ia_msg);

    RCLCPP_INFO(
      this->get_logger(), "Forwarded message of type '%s' from local module to agent '%s' on topic '%s'",
      ia_msg.type.c_str(), receiver.c_str(), it->second->get_topic_name());
  }
}

void CA_Gateway::ia_message_callback(
  const as2_msgs::msg::InterAgentMessage::SharedPtr msg)
{
  const std::string & type = msg->type;

  auto it = registered_types_.find(type);
  if (it != registered_types_.end()) {
    as2_msgs::msg::LocalGenericMessage local_msg;
    local_msg.agents = {msg->sender};
    local_msg.data = msg->data;
    local_msg.type = msg->type;

    it->second->publish(local_msg);

    RCLCPP_INFO(
      this->get_logger(), "Received message of type '%s' from agent '%s', forwarded to local topic '%s'",
      type.c_str(), msg->sender.c_str(), it->second->get_topic_name());
  } else {
    RCLCPP_WARN(
      this->get_logger(), "No local module registered for message type %s. Dropping message.",
      type.c_str());
  }
}

void CA_Gateway::check_peers()
{
  const std::string suffix = "/" + std::string(INCOMING_TOPIC_SUFFIX);

  // Discover new peers
  auto topics = this->get_topic_names_and_types();
  for (auto & [name, types] : topics) {
    bool has_suffix = name.size() >= suffix.size() &&
      name.rfind(suffix) == name.size() - suffix.size();

    if (!has_suffix || name == incoming_topic_name_) {
      continue;
    }

    // Extract agent_id: strip leading '/' and the suffix (e.g. "/drone1/ca_incoming" -> "drone1")
    std::string peer_id = name.substr(1, name.size() - 1 - suffix.size());

    if (inter_agent_publishers_.find(peer_id) == inter_agent_publishers_.end()) {
      inter_agent_publishers_[peer_id] =
        this->create_publisher<as2_msgs::msg::InterAgentMessage>(name, 10);
      RCLCPP_INFO(
        this->get_logger(), "Discovered peer agent '%s' on topic %s",
        peer_id.c_str(), name.c_str());
    }
  }

  // Prune peers whose incoming topic no longer has any subscribers
  for (auto it = inter_agent_publishers_.begin(); it != inter_agent_publishers_.end(); ) {
    const std::string peer_topic = it->second->get_topic_name();
    if (this->count_subscribers(peer_topic) == 0) {
      RCLCPP_INFO(
        this->get_logger(), "Peer agent '%s' disconnected, removing publisher",
        it->first.c_str());
      it = inter_agent_publishers_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace as2_ca
