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
 *  \file       cbba.cpp
 *  \brief      CBBA auction plugin — paper-faithful marginal insertion scoring.
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "cbba/cbba.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <as2_core/names/topics.hpp>

namespace cbba
{

// ── Distance helpers ──────────────────────────────────────────────────────────

double Plugin::dist(const std::string & a, const std::string & b) const
{
  const auto & pa = pos_.at(a);
  const auto & pb = pos_.at(b);
  const double dx = pa[0] - pb[0];
  const double dy = pa[1] - pb[1];
  return std::sqrt(dx * dx + dy * dy);
}

double Plugin::dist(const std::array<double, 2> & from, const std::string & to) const
{
  const auto & pt = pos_.at(to);
  const double dx = from[0] - pt[0];
  const double dy = from[1] - pt[1];
  return std::sqrt(dx * dx + dy * dy);
}

// ── Marginal insertion cost (paper Eq. 3) ─────────────────────────────────────
//
// Returns the cheapest marginal path-length increase of inserting `name` at any
// position in `path`, together with the winning insertion index.
//
// Inserting at position n (0-indexed, n == path.size() means append):
//   prev = path[n-1]  (or start_pos_ for n == 0)
//   next = path[n]    (exists only when n < path.size())
//   marginal = d(prev, name) [+ d(name, next) − d(prev, next)]
//
// The bracketed term is the detour cost; it vanishes when appending.
std::pair<double, size_t> Plugin::best_insertion(
  const std::string & name,
  const std::vector<std::string> & path) const
{
  double best_marginal = std::numeric_limits<double>::infinity();
  size_t best_pos = 0;

  for (size_t n = 0; n <= path.size(); ++n) {
    const double d_prev = (n == 0) ?
      dist(start_pos_, name) :
      dist(path[n - 1], name);

    double marginal = d_prev;
    if (n < path.size()) {
      const double d_next = dist(name, path[n]);
      const double d_direct = (n == 0) ?
        dist(start_pos_, path[n]) :
        dist(path[n - 1], path[n]);
      marginal += d_next - d_direct;
    }

    if (marginal < best_marginal) {
      best_marginal = marginal;
      best_pos = n;
    }
  }

  return {best_marginal, best_pos};
}

// ── Convergence helpers ───────────────────────────────────────────────────────

bool Plugin::round_is_stable() const
{
  if (auction_items_.empty() || participants_.empty()) {
    return false;
  }
  for (const auto & p : participants_) {
    const std::string p_id = (!p.empty() && p[0] == '/') ? p.substr(1) : p;
    if (p_id == namespace_) {
      continue;
    }
    if (!received_from_.count(p_id)) {
      return false;
    }
  }
  return !changed_;
}

std::vector<std::string> Plugin::get_unassigned() const
{
  std::vector<std::string> out;
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    auto it = z_.find(name);
    if (it == z_.end() || it->second.empty()) {
      out.push_back(name);
    }
  }
  return out;
}

// ── Phase 1: bundle construction ─────────────────────────────────────────────
//
// Greedily adds tasks to bundle_ / path_ until round_cap_ is reached or no
// further task can be claimed.  A task is claimable when:
//   (a) this agent is already the consensus winner (re-add after cascade), OR
//   (b) effective_cost < y_[j]  (strictly beats the current winning bid).
// Among all claimable tasks the one with the lowest effective_cost is selected.
void Plugin::build_bundle()
{
  std::unordered_set<std::string> in_path(path_.begin(), path_.end());

  while (static_cast<int>(bundle_.size()) < round_cap_) {
    double best_effective = std::numeric_limits<double>::infinity();
    std::string best_task;
    size_t best_pos = 0;

    for (const auto & item : auction_items_) {
      const std::string & name = item->get_name();
      if (in_path.count(name)) {
        continue;
      }

      auto [marginal, pos] = best_insertion(name, path_);

      // Workload-adjusted effective cost spreads tasks across agents.
      const double effective = marginal +
        workload_weight_ * static_cast<double>(bundle_.size());

      const bool already_winner = (z_.at(name) == namespace_);
      if ((already_winner || effective < y_.at(name)) && effective < best_effective) {
        best_effective = effective;
        best_task = name;
        best_pos = pos;
      }
    }

    if (best_task.empty()) {
      break;
    }

    bundle_.push_back(best_task);
    path_.insert(path_.begin() + static_cast<std::ptrdiff_t>(best_pos), best_task);
    in_path.insert(best_task);
    // Only update y_/z_ when taking a new claim; already-won tasks keep their
    // consensus state so peers do not see a bid change for a stable assignment.
    if (z_[best_task] != namespace_) {
      y_[best_task] = best_effective;
      z_[best_task] = namespace_;
    }
  }
}

// ── Phase 2: cascade removal ─────────────────────────────────────────────────
//
// Erases bundle_[n_bar:] from bundle_ and the corresponding tasks from path_.
// y_/z_ are intentionally NOT reset: resetting would cause oscillation in the
// min-cost convention (see header doc), while preserving them prevents re-claiming
// tasks legitimately held by peers.  The already_winner check in build_bundle
// handles tasks that this agent still holds by consensus (z_==self) but were
// displaced from the bundle by cascade.
void Plugin::cascade_remove(size_t n_bar)
{
  std::unordered_set<std::string> removing;
  for (size_t n = n_bar; n < bundle_.size(); ++n) {
    removing.insert(bundle_[n]);
  }
  bundle_.erase(bundle_.begin() + static_cast<std::ptrdiff_t>(n_bar), bundle_.end());

  path_.erase(
    std::remove_if(
      path_.begin(), path_.end(),
      [&removing](const std::string & t) {return removing.count(t) > 0;}),
    path_.end());

  changed_ = true;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void Plugin::on_auction_items_received(
  const as2_msgs::msg::AuctionItemArray & msg,
  const std::string & agent_id)
{
  AuctionBehaviorPluginBase::on_auction_items_received(msg, agent_id);

  node_ptr_->get_parameter("bundle_size", bundle_size_);
  bundle_size_ = std::max(1, bundle_size_);
  round_cap_ = bundle_size_;   // grows by bundle_size_ each round
  node_ptr_->get_parameter("workload_weight", workload_weight_);
  workload_weight_ = std::max(0.0, workload_weight_);

  // Read agent's current XY position to seed the path origin.
  try {
    const auto pose = state_interface_.get_value<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose);
    start_pos_ = {pose.pose.position.x, pose.pose.position.y};
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("cbba"),
      "Could not read pose for path origin; defaulting to (0,0). (%s)", e.what());
    start_pos_ = {0.0, 0.0};
  }

  // Extract XY positions from each item's features[0..1] and initialise
  // consensus state.  z coords are not used in path cost (same as coordinate_item).
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    const auto & raw = item->get_item().features;
    if (raw.size() >= 2) {
      pos_[name] = {static_cast<double>(raw[0]), static_cast<double>(raw[1])};
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("cbba"),
        "Item '%s' has %zu features (need ≥ 2 for XY). Placing at origin.",
        name.c_str(), raw.size());
      pos_[name] = {0.0, 0.0};
    }
    y_[name] = std::numeric_limits<double>::infinity();
    z_[name] = "";
  }

  RCLCPP_INFO(
    rclcpp::get_logger("cbba"),
    "Received %zu items, bundle_size=%d, start=(%.2f, %.2f)",
    auction_items_.size(), bundle_size_, start_pos_[0], start_pos_[1]);

  build_bundle();
  send_bid(compute_bid());
}

void Plugin::on_activate(std::shared_ptr<const GoalT> /*goal*/) {}
void Plugin::on_deactivate() {}
void Plugin::on_execution_end() {reset();}

void Plugin::on_run()
{
  // Called every behavior tick.  Drive multi-round progression: when the current
  // consensus round is stable and unassigned tasks remain, start the next round.
  if (!round_is_stable()) {
    return;
  }

  const auto unassigned = get_unassigned();
  if (unassigned.empty()) {
    all_assigned_ = true;
    return;
  }

  // ── Start the next round ──────────────────────────────────────────────────
  // Widen the bundle cap so each agent can take bundle_size_ more tasks.
  round_cap_ += bundle_size_;

  // Advance the path origin to the last task assigned so far.  New tasks will
  // be appended (or inserted) relative to this point, creating one continuous
  // spatially-optimised path across all rounds.
  if (!path_.empty()) {
    start_pos_ = pos_.at(path_.back());
  }

  // Reset per-round convergence tracking so all peers must be heard again.
  received_from_.clear();
  changed_ = false;

  // Reset y_/z_ only for unassigned tasks so they can be freshly claimed.
  // Already-assigned tasks keep their consensus state untouched.
  for (const auto & name : unassigned) {
    y_[name] = std::numeric_limits<double>::infinity();
    z_[name] = "";
  }

  build_bundle();

  const int round_num = round_cap_ / bundle_size_;
  RCLCPP_INFO(
    rclcpp::get_logger("cbba"),
    "Round %d started — %zu tasks still unassigned, round_cap=%d",
    round_num, unassigned.size(), round_cap_);

  send_bid(compute_bid());
}

void Plugin::reset()
{
  pos_.clear();
  start_pos_ = {0.0, 0.0};
  y_.clear();
  z_.clear();
  bundle_.clear();
  path_.clear();
  received_from_.clear();
  auction_items_.clear();
  changed_ = false;
  all_assigned_ = false;
  round_cap_ = bundle_size_;
}

// ── Bidding ───────────────────────────────────────────────────────────────────

as2_msgs::msg::Bid Plugin::compute_bid()
{
  as2_msgs::msg::Bid bid;
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    bid.name.push_back(name);
    bid.amounts.push_back(y_.at(name));
    bid.winners.push_back(z_.at(name));
  }
  return bid;
}

void Plugin::update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id)
{
  if (agent_id == namespace_) {
    return;
  }

  changed_ = false;
  received_from_.insert(agent_id);

  // Decode incoming (y_k, z_k).
  std::map<std::string, double> y_k;
  std::map<std::string, std::string> z_k;
  for (size_t i = 0; i < bid_msg.name.size(); ++i) {
    y_k[bid_msg.name[i]] = (i < bid_msg.amounts.size()) ?
      bid_msg.amounts[i] : std::numeric_limits<double>::infinity();
    z_k[bid_msg.name[i]] = (i < bid_msg.winners.size()) ?
      bid_msg.winners[i] : "";
  }

  // Min-cost consensus: lower bid wins; lexicographic tie-break on agent name.
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    auto yk_it = y_k.find(name);
    if (yk_it == y_k.end()) {
      continue;
    }
    const double yk = yk_it->second;
    const std::string & zk = z_k.at(name);

    if (yk < y_[name]) {
      y_[name] = yk;
      z_[name] = zk;
      changed_ = true;
    } else if (yk == y_[name] && !zk.empty() && (z_[name].empty() || zk < z_[name])) {
      z_[name] = zk;
      changed_ = true;
    }
  }

  // Cascade removal: find the first bundle task no longer won by this agent.
  for (size_t n = 0; n < bundle_.size(); ++n) {
    if (z_[bundle_[n]] != namespace_) {
      cascade_remove(n);
      break;
    }
  }

  build_bundle();

  RCLCPP_DEBUG(
    rclcpp::get_logger("cbba"),
    "Updated from '%s' — bundle=%zu path=%zu changed=%s",
    agent_id.c_str(), bundle_.size(), path_.size(), changed_ ? "true" : "false");
}

// ── Convergence ───────────────────────────────────────────────────────────────

bool Plugin::check_convergence()
{
  // all_assigned_ is set by on_run() once every task has a winner.
  if (all_assigned_) {
    return true;
  }

  // Also converge immediately when the round is stable AND nothing is unassigned
  // (handles the common case where all tasks fit within a single round without
  // waiting for the next on_run() tick to set all_assigned_).
  if (round_is_stable() && get_unassigned().empty()) {
    all_assigned_ = true;
    return true;
  }

  return false;
}

// ── Result accessors ──────────────────────────────────────────────────────────

// get_feedback returns tasks in path_ order (spatial), which is more useful to the
// executing drone than bundle_ (addition) order.
Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & task_name : path_) {
    feedback.asignees.push_back(namespace_);
    feedback.amounts.push_back(y_.at(task_name));
    for (const auto & item : auction_items_) {
      if (item->get_name() == task_name) {
        feedback.items.push_back(item->get_item());
        break;
      }
    }
  }
  return feedback;
}

Plugin::ResultT Plugin::get_result()
{
  ResultT result;
  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    result.elements.push_back(item->get_item());
    auto it = z_.find(name);
    result.winners.push_back((it != z_.end()) ? it->second : "");
  }
  return result;
}

std::map<std::string, std::string> Plugin::get_global_assignment() const
{
  std::map<std::string, std::string> assignment;
  for (const auto & [task, winner] : z_) {
    if (!winner.empty()) {
      assignment[task] = winner;
    }
  }
  return assignment;
}

}  // namespace cbba

PLUGINLIB_EXPORT_CLASS(cbba::Plugin, as2_auction_behavior::AuctionBehaviorPluginBase)
