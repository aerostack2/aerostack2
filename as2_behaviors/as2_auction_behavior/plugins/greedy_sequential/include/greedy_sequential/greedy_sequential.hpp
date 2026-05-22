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
 *  \file       greedy_sequential.hpp
 *  \brief      Greedy broadcast auction plugin.
 *
 *  Protocol:
 *    1. On receiving auction items → compute costs for all items and broadcast.
 *    2. On receiving a peer bid → store it and resolve conflicts locally.
 *    3. Once all participant bids have arrived → declare convergence.
 *
 *  Conflict rule: for each item, the agent with the lowest cost wins.
 *  Tie-break: lexicographically smallest agent name.
 *
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_
#define GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "as2_auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace greedy_sequential
{

class Plugin : public as2_auction_behavior::AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT = as2_msgs::action::Auction::Result;

public:
  Plugin() = default;

  // Compute costs for all auction items and broadcast immediately.
  // Called for both the auctioneer and participant roles before any bid arrives.
  void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id) override;

  // No-op: the bid has already been sent in on_auction_items_received.
  void on_activate(std::shared_ptr<const GoalT> goal) override;
  void on_deactivate() override;
  void on_execution_end() override;

  // Returns empty -> bids are sent once in on_auction_items_received, not reactively.
  as2_msgs::msg::Bid compute_bid() override;

  // Store the peer's cost vector and re-run conflict resolution.
  void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) override;

  // True once every participant's bid has been received.
  bool check_convergence() override;

  FeedbackT get_feedback() override;
  ResultT get_result() override;
  std::map<std::string, std::string> get_global_assignment() const override;

protected:
  // This agent's cost for each item, computed once in on_auction_items_received.
  std::map<std::string, double> my_costs_;

  // Cost vectors received from peers: agent_id → (item_name → cost).
  std::map<std::string, std::map<std::string, double>> received_bids_;

  // Agent IDs from which a bid has been stored.
  std::unordered_set<std::string> received_from_;

  // Current assignment for this agent, updated after every new peer bid.
  std::vector<std::pair<std::string, double>> my_assignment_;

  // Global assignment map: item name → winning agent. Populated during solve_conflicts.
  std::map<std::string, std::string> global_assignment_;

private:
  // Per-item conflict resolution: for each item, assign to the cheapest agent.
  // Tie-break: lexicographically smallest agent name.
  void solve_conflicts();

  void reset();
};

}  // namespace greedy_sequential

#endif  // GREEDY_SEQUENTIAL__GREEDY_SEQUENTIAL_HPP_
