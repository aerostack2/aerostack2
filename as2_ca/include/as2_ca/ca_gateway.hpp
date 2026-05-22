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

/**
 * @brief Collective Awareness Gateway node.
 *
 * One instance runs per drone alongside its existing Aerostack2 stack.  It
 * owns all inter-agent traffic for that drone and provides three services to
 * local behaviors via @ref CAGatewayClient:
 *
 * - **Peer discovery**: a periodic timer scans active ROS 2 topics for peers
 *   whose incoming topic matches the well-known suffix and creates a publisher
 *   to each newly discovered peer.  Peers that disappear (subscriber count
 *   drops to zero) are pruned automatically, giving implicit failure detection
 *   within one discovery cycle.
 *
 * - **Inbound demultiplexing**: messages arriving on the drone's own incoming
 *   topic (@c <ns>/gateway_in) are routed to the local behavior that registered
 *   for that message type via the @c register_module service.
 *
 * - **Outbound forwarding**: serialized messages published by behaviors on
 *   @c <ns>/gateway_out are wrapped as @c InterAgentMessage and forwarded to
 *   the per-peer publishers.
 *
 * The gateway is framework-agnostic: it has no knowledge of the coordination
 * protocol; all policy lives in the behaviors that use it.
 */
class CA_Gateway : public rclcpp::Node
{
public:
  explicit CA_Gateway(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Route an inbound inter-agent message to the registered local behavior.
   *
   * Looks up the message type in @c registered_types_ and publishes a
   * @c LocalGenericMessage carrying the payload and sender ID to the
   * corresponding local topic.  Drops the message and logs a warning if no
   * behavior has registered for the type.
   *
   * @param msg Inbound inter-agent message received on the drone's incoming topic.
   */
  void ia_message_callback(const as2_msgs::msg::InterAgentMessage::SharedPtr msg);

  /**
   * @brief Service handler that registers a local behavior for a message type.
   *
   * Called by @ref CAGatewayClient::register_module.  Creates a
   * @c LocalGenericMessage publisher for @p type if one does not exist yet,
   * and returns its topic name so the client can subscribe.
   *
   * @param request  Registration request containing the message type tag and
   *                 the registering module's name.
   * @param response Filled with the topic name the caller should subscribe to,
   *                 or left empty on failure.
   */
  void register_module(
    const std::shared_ptr<as2_msgs::srv::RegisterModule::Request> request,
    std::shared_ptr<as2_msgs::srv::RegisterModule::Response> response);

  /**
   * @brief Forward an outbound @c LocalGenericMessage to the specified peers.
   *
   * Serializes the payload into an @c InterAgentMessage and publishes it to
   * each peer's incoming topic publisher.  Skips self-forwarding and drops
   * messages for peers that have not been discovered yet.
   *
   * @param msg Outbound generic message from a local behavior containing the
   *            target peer list, message type tag, and serialized payload.
   */
  void forward_local_message(const as2_msgs::msg::LocalGenericMessage::SharedPtr msg);

protected:
  rclcpp::Subscription<as2_msgs::msg::InterAgentMessage>::SharedPtr inter_agent_sub_;  ///< Subscriber to own incoming topic
  rclcpp::Publisher<as2_msgs::msg::InterAgentMessage>::SharedPtr inter_agent_pub_;     ///< (unused directly; per-peer publishers used instead)
  rclcpp::Service<as2_msgs::srv::RegisterModule>::SharedPtr register_module_srv_;      ///< Service for behaviors to register message-type handlers
  rclcpp::Subscription<as2_msgs::msg::LocalGenericMessage>::SharedPtr outgoing_ca_sub_; ///< Subscriber to outbound messages from local behaviors

  /// Map from message-type tag to the local publisher that delivers inbound messages of that type.
  std::unordered_map<std::string,
    rclcpp::Publisher<as2_msgs::msg::LocalGenericMessage>::SharedPtr> registered_types_;

  std::string gateway_in_topic_;             ///< Full topic name of the drone's incoming inter-agent topic
  std::string register_module_service_name_; ///< Service name for module registration
  std::string out_messages_topic_;           ///< Topic on which local behaviors publish outbound messages
  std::string agent_id_;                     ///< Drone namespace without leading '/'

  /// Map from peer agent ID to the publisher targeting that peer's incoming topic.
  std::unordered_map<std::string,
    rclcpp::Publisher<as2_msgs::msg::InterAgentMessage>::SharedPtr> inter_agent_publishers_;

  rclcpp::TimerBase::SharedPtr peer_check_timer_; ///< Periodic timer that drives peer discovery and pruning
  std::string incoming_topic_name_;               ///< Cached incoming topic name used to exclude self during discovery

  /**
   * @brief Discover new peers and prune disappeared ones.
   *
   * Scans the active topic list for topics whose name ends with the well-known
   * @c INCOMING_TOPIC_SUFFIX.  New topics cause a peer publisher to be created;
   * existing publishers whose topic has no subscribers are removed, providing
   * implicit failure detection within one discovery cycle.
   */
  void check_peers();
};
}  // namespace as2_ca

#endif  // AS2_CA__CA_GATEWAY_HPP_
