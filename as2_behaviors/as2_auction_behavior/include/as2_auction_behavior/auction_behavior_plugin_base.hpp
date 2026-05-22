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
 *  \file       auction_behavior_plugin_base.hpp
 *  \brief      auction behavior plugin base header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_
#define AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <as2_core/node.hpp>
#include <as2_behavior/behavior_utils.hpp>

#include "as2_core/state_interface.hpp"
#include "as2_ca/ca_gateway_client.hpp"
#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/auction_item_array.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace as2_auction_behavior
{


class AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;

public:
  virtual ~AuctionBehaviorPluginBase() = default;
  virtual void initialize(
    as2::Node * node_ptr,
    as2_ca::CAGatewayClient & client)
  {
    state_interface_ = StateInterface();
    node_ptr->declare_parameter<std::vector<std::string>>(
      "state_component",
      std::vector<std::string>());

    client_ = client;
    namespace_ = node_ptr->get_namespace();

    if (!namespace_.empty() && namespace_[0] == '/') {
      namespace_ = namespace_.substr(1);
    }

    node_ptr_ = node_ptr;
  }

  virtual void on_activate(std::shared_ptr<const GoalT> goal) = 0;
  virtual void on_deactivate() = 0;
  virtual void on_execution_end() = 0;

  void set_item_plugin(std::shared_ptr<AuctionItemPluginBase> item_plugin)
  {
    item_plugin_ = item_plugin;
  }

  virtual void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id)
  {
    auction_items_.clear();
    for (const auto & item_msg : msg.list) {
      auction_items_.push_back(item_plugin_->create(item_msg));
    }
  }

  virtual as2_msgs::msg::Bid compute_bid() = 0;

  virtual void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) = 0;

  virtual bool check_convergence() = 0;

  void send_bid(const as2_msgs::msg::Bid & bid)
  {
    if (bid.name.empty() || participants_.empty()) {
      return;
    }
    std::string claimed_tasks;
    for (const auto & name : bid.name) {
      claimed_tasks += "'" + name + "' ";
    }
    RCLCPP_INFO(
      rclcpp::get_logger("AuctionBehaviorPluginBase"),
      "Sending bid claiming %zu task(s) [%s] to %zu participant(s)",
      bid.name.size(), claimed_tasks.c_str(), participants_.size());
    client_.forward_IA_msg<as2_msgs::msg::Bid>(bid, "bid", participants_);
  }

  void on_bid_received(
    const as2_msgs::msg::Bid & msg,
    const std::string & agent_id)
  {
    // Update state
    update(msg, agent_id);

    // Check convergence
    if (check_convergence()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AuctionBehaviorPluginBase"), "Auction converged, no more bids will be sent");
      return;
    }

    // Compute new bids; an empty bid means bundle is full — do not forward to
    // avoid infinite loops between agents.
    as2_msgs::msg::Bid new_bid = compute_bid();
    send_bid(new_bid);
  }

  // Called by the behavior's on_run() tick. Override in plugins that need
  // periodic logic (e.g. retransmission, timeout-based convergence).
  virtual void on_run() {}

  void set_participans(const std::vector<std::string> & participants)
  {
    participants_ = participants;
  }

  virtual void configure(rclcpp::Node * node_ptr)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("AuctionBehaviorPluginBase"), "Configuring state interface");
    std::vector<std::string> state_components =
      node_ptr->get_parameter("state_component").as_string_array();
    std::string state_components_str;
    for (const auto & component : state_components) {
      state_components_str += component + " ";
    }
    RCLCPP_INFO(
      node_ptr->get_logger(), "State components: [%s]", state_components_str.c_str());
    state_interface_.configure(node_ptr, state_components);
  }

  virtual as2_msgs::action::Auction::Feedback get_feedback() = 0;
  virtual as2_msgs::action::Auction::Result get_result() = 0;

  /**
   * @brief Return the complete assignment map: item name -> assigned agent.
   *
   * Called after convergence to query the global auction result from any
   * participant's perspective. Each plugin must store the full conflict-
   * resolution outcome during solve_conflicts().
   */
  virtual std::map<std::string, std::string> get_global_assignment() const = 0;

protected:
  AuctionBehaviorPluginBase() = default;
  StateInterface state_interface_;
  as2_ca::CAGatewayClient client_;
  std::vector<std::shared_ptr<AuctionItemPluginBase>> auction_items_;
  std::shared_ptr<AuctionItemPluginBase> item_plugin_;
  std::vector<std::string> participants_;
  std::string namespace_;
  as2::Node * node_ptr_;

};

}  // namespace as2_auction_behavior

#endif  // AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_
