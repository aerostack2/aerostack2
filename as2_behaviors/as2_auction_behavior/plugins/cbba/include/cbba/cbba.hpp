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
 *  \file       cbba.hpp
 *  \brief      Consensus-Based Bundle Algorithm (CBBA) auction plugin.
 *
 *  Bid convention (min-cost): lower cost = better claim, consistent with item plugins that
 *  return distance/cost. The winning bid y[j] is the LOWEST cost seen for task j;
 *  y[j] is initialized to +∞ so any real cost can claim an uncontested task.
 *
 *  Protocol:
 *    Phase 1 (Bundle Building): Each agent greedily adds up to bundle_size tasks. A task j
 *    is claimable if the agent's cost < y[j] (current winning cost). The agent picks the
 *    task with the lowest cost among claimable tasks, sets y[j] = cost, z[j] = self.
 *
 *    Phase 2 (Consensus): Agents broadcast their full (y, z) state as a Bid message.
 *    For each task: if the incoming bid cost is strictly lower, adopt it. Lexicographic
 *    tie-break on agent name. If a bundled task is lost (z[j] ≠ self), cascade-remove
 *    all subsequent bundle entries so they can be re-bid with fresh marginal costs.
 *
 *  Convergence: All participants' bids received AND no state change in the last update.
 *
 *  Reference: Choi, H.-L., Brunet, L., & How, J. P. (2009). "Consensus-based
 *  decentralized auctions for robust task allocation." IEEE Trans. Robotics, 25(4).
 *
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef CBBA__CBBA_HPP_
#define CBBA__CBBA_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "as2_auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace cbba
{

class Plugin : public as2_auction_behavior::AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT = as2_msgs::action::Auction::Result;

public:
  Plugin() = default;

  // Declares the bundle_size parameter so the ROS parameter system knows about it
  // before on_auction_items_received() tries to read it via get_parameter().
  void initialize(
    as2::Node * node_ptr,
    as2_ca::CAGatewayClient & client) override
  {
    AuctionBehaviorPluginBase::initialize(node_ptr, client);
    node_ptr->declare_parameter("bundle_size", 1);
  }

  void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id) override;

  void on_activate(std::shared_ptr<const GoalT> goal) override;
  void on_deactivate() override;
  void on_execution_end() override;

  as2_msgs::msg::Bid compute_bid() override;

  // Apply one round of consensus rules, then cascade-remove and rebuild bundle if needed.
  void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) override;

  bool check_convergence() override;

  FeedbackT get_feedback() override;
  ResultT get_result() override;
  std::map<std::string, std::string> get_global_assignment() const override;

protected:
  // Cost of each task for this agent, computed once at auction start via item->evaluate().
  std::map<std::string, double> costs_;

  // CBBA consensus state:
  //   y_[j] = current winning bid (minimum cost seen) for task j; +∞ = unclaimed.
  //   z_[j] = agent that currently holds the winning bid for task j; "" = unclaimed.
  std::map<std::string, double> y_;
  std::map<std::string, std::string> z_;

  // This agent's current bundle (ordered, |bundle_| ≤ bundle_size_).
  std::vector<std::string> bundle_;

  // Agents from which at least one bid has been received.
  std::unordered_set<std::string> received_from_;

  // True if the last update() call changed any y_ or z_ entry.
  bool changed_ = false;

  // Maximum tasks per agent (Lt). Read from "bundle_size" ROS parameter.
  int bundle_size_ = 1;

  // Phase 1: greedily fill the bundle up to bundle_size_.
  void build_bundle();

  // Release bundle entries from position n_bar onwards and reset their y/z.
  void cascade_remove(size_t n_bar);

private:
  void reset();
};

}  // namespace cbba

#endif  // CBBA__CBBA_HPP_
