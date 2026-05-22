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
 *  \brief      CBBA auction plugin implementation.
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "cbba/cbba.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cbba
{

// ── Phase 1: bundle construction ─────────────────────────────────────────────

void Plugin::build_bundle()
{
  std::unordered_set<std::string> in_bundle(bundle_.begin(), bundle_.end());

  while (static_cast<int>(bundle_.size()) < bundle_size_) {
    double best_cost = std::numeric_limits<double>::infinity();
    std::string best_task;

    for (const auto & item : auction_items_) {
      const std::string & name = item->get_name();
      if (in_bundle.count(name)) {
        continue;
      }
      const double c = costs_.at(name);
      // Claimable if: (a) this agent is already the consensus winner (re-add after cascade),
      // or (b) this agent's cost strictly beats the current winning bid (new claim).
      const bool already_winner = (z_.at(name) == namespace_);
      if ((already_winner || c < y_.at(name)) && c < best_cost) {
        best_cost = c;
        best_task = name;
      }
    }

    if (best_task.empty()) {
      break;
    }

    bundle_.push_back(best_task);
    in_bundle.insert(best_task);
    // Only update y/z when taking a new claim; already-won tasks keep their consensus state.
    if (z_[best_task] != namespace_) {
      y_[best_task] = best_cost;
      z_[best_task] = namespace_;
    }
  }
}

// ── Phase 2: cascade removal ─────────────────────────────────────────────────

// Removes bundle entries from n_bar onwards without touching y_/z_. Preserving the consensus
// state prevents re-claiming tasks that were legitimately outbid: after removal, build_bundle
// will only re-add a task if this agent is still the consensus winner (z_==namespace_) or can
// strictly outbid the current winner (costs_ < y_).
void Plugin::cascade_remove(size_t n_bar)
{
  bundle_.erase(bundle_.begin() + n_bar, bundle_.end());
  changed_ = true;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void Plugin::on_auction_items_received(
  const as2_msgs::msg::AuctionItemArray & msg,
  const std::string & agent_id)
{
  AuctionBehaviorPluginBase::on_auction_items_received(msg, agent_id);

  node_ptr_->get_parameter("bundle_size", bundle_size_);
  bundle_size_ = std::max(1, bundle_size_);

  for (const auto & item : auction_items_) {
    const std::string & name = item->get_name();
    costs_[name] = static_cast<double>(item->evaluate(state_interface_));
    y_[name] = std::numeric_limits<double>::infinity();
    z_[name] = "";
  }

  RCLCPP_INFO(
    rclcpp::get_logger("cbba"),
    "Received %zu items, bundle_size=%d", auction_items_.size(), bundle_size_);

  build_bundle();
  send_bid(compute_bid());
}

void Plugin::on_activate(std::shared_ptr<const GoalT>/*goal*/) {}
void Plugin::on_deactivate() {}
void Plugin::on_execution_end() {reset();}

void Plugin::reset()
{
  costs_.clear();
  y_.clear();
  z_.clear();
  bundle_.clear();
  received_from_.clear();
  auction_items_.clear();
  changed_ = false;
}

// ── bidding ───────────────────────────────────────────────────────────────────

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
      // Tie-break: lexicographically smaller agent name wins deterministically.
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

  // Rebuild bundle after any removals or new information.
  build_bundle();

  RCLCPP_DEBUG(
    rclcpp::get_logger("cbba"),
    "Updated from '%s' — bundle size=%zu changed=%s",
    agent_id.c_str(), bundle_.size(), changed_ ? "true" : "false");
}

// ── convergence ───────────────────────────────────────────────────────────────

bool Plugin::check_convergence()
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

  // Stable when all peers have been heard from and last update caused no state change.
  return !changed_;
}

// ── result accessors ──────────────────────────────────────────────────────────

Plugin::FeedbackT Plugin::get_feedback()
{
  FeedbackT feedback;
  for (const auto & task_name : bundle_) {
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
