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
 *  return distance/cost. The winning bid y[j] is the LOWEST effective cost seen for task j;
 *  y[j] is initialised to +∞ so any real cost can claim an uncontested task.
 *
 *  Scoring — paper-faithful marginal insertion cost (Eq. 3 of Choi et al. 2009):
 *    The score of adding task j to the current path is the MARGINAL increase in total
 *    path length when j is inserted at its cheapest position:
 *
 *      c_ij[b_i] = min_n  d(prev_n, j) + d(j, next_n) − d(prev_n, next_n)
 *
 *    where prev_n / next_n are the predecessor / successor at insertion slot n, and
 *    d(·,·) is the Euclidean XY distance.  start_pos_ (the agent's pose at auction
 *    start) is used as the implicit predecessor at position 0.  Appending at the end
 *    simplifies to d(last_in_path, j).  This satisfies the Diminishing Marginal Gain
 *    (DMG) property (triangle inequality), which is required for the 50%-optimality
 *    guarantee of Theorem 2.
 *
 *    An optional workload_weight penalty (w × |bundle|) is added on top of the marginal
 *    cost before comparing bids.  All agents share the same parameter, so consensus
 *    comparisons remain sound.  Set to 0.0 for pure distance-marginal CBBA.
 *
 *  Protocol:
 *    Phase 1 (Bundle Building): Each agent greedily adds up to bundle_size tasks. A task j
 *    is claimable if:
 *      (a) this agent is already the consensus winner (z[j] == self), OR
 *      (b) effective_cost(j) strictly beats the current winning bid (y[j]).
 *    The agent picks the best claimable task, inserts it into path_ at the optimal spatial
 *    position, and appends it to bundle_ (recording addition order separately from spatial
 *    order, as required by the paper).
 *
 *    Phase 2 (Consensus): Agents broadcast their full (y, z) state as a Bid message.
 *    For each task: if the incoming bid cost is strictly lower, adopt it. Lexicographic
 *    tie-break on agent name (deterministic conflict resolution without timestamps, valid
 *    for fully-connected networks where D = 1).
 *
 *    Cascade removal: if a bundled task is lost (z[j] ≠ self), all subsequent bundle
 *    entries are erased from bundle_ and path_ so their marginal costs are recomputed
 *    fresh.  y_/z_ are deliberately NOT reset (unlike paper Eq. 6) because, in the
 *    min-cost bid convention, resetting causes infinite re-claim oscillation: after reset
 *    the agent immediately re-bids its own marginal cost, then the peer's lower bid
 *    re-triggers another reset.  Preserving y_/z_ prevents re-claiming tasks that were
 *    legitimately outbid; the already_winner check re-adds tasks the agent still holds
 *    by consensus (z[j] == self but removed by cascade).
 *
 *  Multi-round extension: if N_tasks > N_agents × bundle_size a single round cannot assign
 *  all tasks.  on_run() detects round stability and, when unassigned tasks remain, starts the
 *  next round: it widens round_cap_ by bundle_size_, updates start_pos_ to the tail of the
 *  current path so new tasks continue the journey, resets received_from_ / changed_, and
 *  sends a fresh bid.  This repeats until every task has a winner.  The final path_ is a
 *  globally space-optimised sequence across all rounds.
 *
 *  Convergence: round stable (all peers heard + no state change) AND all tasks assigned.
 *
 *  Reference: Choi, H.-L., Brunet, L., & How, J. P. (2009). "Consensus-based
 *  decentralized auctions for robust task allocation." IEEE Trans. Robotics, 25(4).
 *
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef CBBA__CBBA_HPP_
#define CBBA__CBBA_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
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

  // Declares ROS parameters so the parameter system knows about them before
  // on_auction_items_received() reads them via get_parameter().
  void initialize(
    as2::Node * node_ptr,
    as2_ca::CAGatewayClient & client) override
  {
    AuctionBehaviorPluginBase::initialize(node_ptr, client);
    node_ptr->declare_parameter("bundle_size", 1);
    node_ptr->declare_parameter("workload_weight", 0.0);
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

  // Periodic tick: detects round stability and, when unassigned tasks remain, starts
  // the next consensus round so all tasks are eventually covered.
  void on_run() override;

  bool check_convergence() override;

  FeedbackT get_feedback() override;
  ResultT get_result() override;
  std::map<std::string, std::string> get_global_assignment() const override;

protected:
  // ── Spatial data ──────────────────────────────────────────────────────────

  // XY positions of every task, extracted from AuctionItem features[0..1] at init.
  std::map<std::string, std::array<double, 2>> pos_;

  // Agent's XY position at auction start — used as the path origin (prev_0).
  std::array<double, 2> start_pos_{{0.0, 0.0}};

  // ── CBBA consensus state ──────────────────────────────────────────────────

  //   y_[j] = current winning bid (minimum effective cost seen) for task j; +∞ = unclaimed.
  //   z_[j] = agent holding the winning bid for task j; "" = unclaimed.
  std::map<std::string, double> y_;
  std::map<std::string, std::string> z_;

  // ── Bundle and path ───────────────────────────────────────────────────────

  // bundle_: tasks in addition order (|bundle_| ≤ bundle_size_).
  // path_:   same tasks in spatially optimised order (best-insertion order).
  //          Always |path_| == |bundle_|.
  std::vector<std::string> bundle_;
  std::vector<std::string> path_;

  // ── Convergence tracking ──────────────────────────────────────────────────

  // Agents from which at least one bid has been received in the current round.
  std::unordered_set<std::string> received_from_;

  // True if the last update() call changed any y_ or z_ entry.
  bool changed_ = false;

  // True once every task has a winner across all rounds.
  bool all_assigned_ = false;

  // ── Parameters ────────────────────────────────────────────────────────────

  // Maximum tasks per agent per round (Lt). Read from "bundle_size" ROS parameter.
  int bundle_size_ = 1;

  // Running bundle capacity: starts at bundle_size_, grows by bundle_size_ each round
  // so the same agent can accumulate tasks across multiple auction rounds.
  int round_cap_ = 1;

  // Optional additive load-balancing penalty: effective_cost = marginal + w × |bundle|.
  // Set to 0.0 for pure distance-marginal CBBA (paper default).
  double workload_weight_ = 0.0;

  // ── Spatial helpers ───────────────────────────────────────────────────────

  // Euclidean XY distance between two named tasks.
  double dist(const std::string & a, const std::string & b) const;

  // Euclidean XY distance from a raw position to a named task.
  double dist(const std::array<double, 2> & from, const std::string & to) const;

  // Cheapest marginal path-length increase of inserting `name` anywhere in `path`.
  // Returns {min_marginal, best_insertion_index}.
  std::pair<double, size_t> best_insertion(
    const std::string & name,
    const std::vector<std::string> & path) const;

  // Phase 1: greedily fill bundle_ / path_ up to round_cap_.
  void build_bundle();

  // Erase bundle_[n_bar:] from bundle_ and path_.
  // y_/z_ are intentionally preserved (see header-level doc on cascade removal).
  void cascade_remove(size_t n_bar);

  // True when the current round is stable: all peers heard and no state change.
  bool round_is_stable() const;

  // Collect names of tasks with no winner yet (z_[j] == "").
  std::vector<std::string> get_unassigned() const;

private:
  void reset();
};

}  // namespace cbba

#endif  // CBBA__CBBA_HPP_
