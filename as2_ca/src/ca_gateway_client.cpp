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
 *  \brief      Ca_gateway_client implementation file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "as2_ca/ca_gateway_client.hpp"
#include <algorithm>
#include <cctype>
#include <string>
#include <memory>
#include <vector>

using std::placeholders::_1;

namespace as2_ca
{
CAGatewayClient::CAGatewayClient(rclcpp::Node * parent)
{
  parent_ = parent;

  agent_id_ = parent_->get_namespace();

  // Add node namespace and register module
  std::string register_module_service = agent_id_ + "/register_module";
  std::string forward_generic_topic = agent_id_ + "/gateway_out";
  // Create a client for the module registration service
  register_module_client_ = parent_->create_client<as2_msgs::srv::RegisterModule>(
    register_module_service);

  // Wait for the service to be available
  while (!register_module_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(parent->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(parent->get_logger(), "Service not available, waiting again...");
  }

  RCLCPP_INFO(parent->get_logger(), "Connected to register_module service");

  forwarder_pub_ = parent_->create_publisher<as2_msgs::msg::LocalGenericMessage>(
    forward_generic_topic, 10);
}


int CAGatewayClient::get_subscriber_count()
{
  return local_generic_subscribers_.size();
}

std::vector<std::string> CAGatewayClient::get_known_peers()
{
  std::vector<std::string> peers;

  // Get all nodes in the system
  auto node_names = parent_->get_node_names();

  std::string own_id = agent_id_;
  if (!own_id.empty() && own_id.front() == '/') {own_id = own_id.substr(1);}

  for (const auto & node_name : node_names) {
    // node_name is fully-qualified, e.g. "/drone0/kb/knowledge_core" or "/rviz".
    // The agent's namespace is only the FIRST path segment ("drone0"), not
    // everything up to the last slash — nodes nested deeper than one level
    // (e.g. "drone0/kb/knowledge_core") would otherwise be misread as a
    // distinct peer "drone0/kb".
    if (node_name.empty() || node_name.front() != '/') continue;

    size_t second_slash = node_name.find('/', 1);
    if (second_slash == std::string::npos) {
      // Node lives directly in the root namespace (e.g. "/rviz",
      // "/kb_monitor", a randomly-named tf2 listener) — not a drone agent.
      continue;
    }
    std::string namespace_str = node_name.substr(1, second_slash - 1);

    // Only "droneN" namespaces are collision-avoidance agents — monitoring/
    // viz/KB tooling can live under their own multi-node namespaces too, and
    // none of those run a CollisionAvoidanceBehavior peer.
    bool is_drone_namespace = namespace_str.rfind("drone", 0) == 0 &&
      namespace_str.size() > 5 &&
      std::all_of(
        namespace_str.begin() + 5, namespace_str.end(),
        [](unsigned char c) {return std::isdigit(c);});

    if (is_drone_namespace && namespace_str != own_id) {
      // Check if this namespace is already in the peers list
      if (std::find(peers.begin(), peers.end(), namespace_str) == peers.end()) {
        peers.push_back(namespace_str);
      }
    }
  }

  return peers;
}

void CAGatewayClient::clear()
{
  this->local_generic_subscribers_.clear();
}

CAGatewayClient::~CAGatewayClient()
{
  clear();
}

}  // namespace as2_ca
