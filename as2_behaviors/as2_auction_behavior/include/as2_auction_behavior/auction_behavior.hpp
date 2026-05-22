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
 *  \file       auction_behavior.hpp
 *  \brief      auction behavior header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_
#define AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "as2_ca/ca_gateway_client.hpp"
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/auction.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "as2_auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_auction_behavior/auction_item_plugin_base.hpp"

/**
 * @brief Aerostack2 behavior that runs a distributed task-allocation auction.
 *
 * @c AuctionBehavior implements the *auctions* collective model from the
 * CORESENSE Collective Awareness architecture.  It can be activated either as
 * the **auctioneer** (initiated by a mission script) or as a **participant**
 * (triggered by a @c StartAuction modelet received from a peer).
 *
 * **Protocol summary**
 * 1. The auctioneer broadcasts @c StartAuction (carrying the item list and
 *    bidder set) to all other participants via @ref as2_ca::CAGatewayClient.
 * 2. Each agent calls @c compute_bid, forwards its @c Bid, and calls
 *    @c on_bid_received whenever a peer's bid arrives.
 * 3. @c on_run polls @c check_convergence every tick; on convergence it
 *    collects the global assignment and publishes it to the knowledge base.
 *
 * **Plugin architecture**
 * Two independently swappable plugins separate the protocol from the algorithm:
 * - @ref as2_auction_behavior::AuctionBehaviorPluginBase controls bid
 *   computation, conflict resolution, and convergence detection.
 * - @ref as2_auction_behavior::AuctionItemPluginBase controls cost evaluation
 *   for a specific item type (e.g. coordinate-based Euclidean distance).
 *
 * Results are published to the KB as @c (item_name, assignedTo, drone_ns)
 * triples plus coordinate and auction-ID facts.
 */
class AuctionBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Auction>
{
public:
  explicit AuctionBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AuctionBehavior();

  /**
   * @brief Register @c StartAuction and @c Bid handlers with the CA Gateway.
   *
   * Must be called once after construction.  The @c StartAuction handler
   * loads the item plugin and sends a self-goal so that @c on_run ticks
   * for convergence detection even when the node acts as a participant.
   */
  void configure();

private:
  using GoalT = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT = as2_msgs::action::Auction::Result;

  std::string behavior_name_;

  std::string plugin_name_;
  std::shared_ptr<pluginlib::ClassLoader<as2_auction_behavior::AuctionBehaviorPluginBase>> loader_;
  std::shared_ptr<as2_auction_behavior::AuctionBehaviorPluginBase> auction_plugin_;

  std::shared_ptr<pluginlib::ClassLoader<as2_auction_behavior::AuctionItemPluginBase>>
  item_loader_;
  std::shared_ptr<as2_auction_behavior::AuctionItemPluginBase> item_plugin_;
  std::string loaded_item_type_;

  GoalT goal_;
  FeedbackT feedback_;
  ResultT result_;

  as2_ca::CAGatewayClient client_;

  bool started;
  bool is_participant_{false};

  std::string auction_id_;

  rclcpp_action::Client<as2_msgs::action::Auction>::SharedPtr self_action_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  void publish_results_to_kb(const ResultT & result);

  bool on_activate(std::shared_ptr<const GoalT> goal) override;
  bool on_modify(std::shared_ptr<const GoalT> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const GoalT> & goal,
    std::shared_ptr<FeedbackT> & feedback_msg,
    std::shared_ptr<ResultT> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;
};
#endif  // AS2_AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_
