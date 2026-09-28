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
 *  \file       cbba_gtest.cpp
 *  \brief      Unit tests for the CBBA auction plugin (no live ROS node required).
 *
 *  Bid amounts represent COSTS (lower = better), consistent with item plugins that
 *  return Euclidean distance or similar cost functions.
 *
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "as2_auction_behavior/auction_item_plugin_base.hpp"
#include "as2_core/state_interface.hpp"
#include "cbba/cbba.hpp"
#include "as2_msgs/msg/bid.hpp"

// ── mock infrastructure ──────────────────────────────────────────────────────

class MockItem : public as2_auction_behavior::AuctionItemPluginBase
{
public:
  MockItem(const std::string & name, float cost)
  : name_(name), cost_(cost)
  {
    item_.name = name;
    // Place the task at (cost, 0) so dist(origin, task) == cost when start_pos_ == {0,0}.
    item_.features = {cost, 0.0f, 0.0f};
  }

  std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem &) const override
  {
    return std::make_shared<MockItem>(name_, cost_);
  }

  float evaluate(const StateInterface &) const override {return cost_;}
  std::string get_name() const override {return name_;}
  as2_msgs::msg::AuctionItem get_item() const override {return item_;}
  std::string to_string() const override
  {
    return "MockItem(name='" + name_ + "', cost=" + std::to_string(cost_) + ")";
  }

private:
  std::string name_;
  float cost_;
  as2_msgs::msg::AuctionItem item_;
};

// Exposes protected state for white-box testing without triggering ROS comms.
class TestablePlugin : public cbba::Plugin
{
public:
  // Populate auction_items_ and initialise CBBA state without ROS.
  // The item is placed at (cost, 0) so dist(start_pos_={0,0}, item) == cost,
  // preserving the original test semantics: lower cost = closer = picked first.
  void add_item(const std::string & name, float cost)
  {
    auction_items_.push_back(std::make_shared<MockItem>(name, cost));
    pos_[name] = {static_cast<double>(cost), 0.0};
    y_[name] = std::numeric_limits<double>::infinity();
    z_[name] = "";
  }

  // Override the XY position of an already-added item.
  // Useful when the default (cost, 0) layout is collinear and causes unintended
  // zero-marginal re-claims.
  void set_item_pos(const std::string & name, double x, double y)
  {
    pos_[name] = {x, y};
  }

  void set_namespace(const std::string & ns) {namespace_ = ns;}
  void add_participant(const std::string & p) {participants_.push_back(p);}
  void set_bundle_size(int sz) {bundle_size_ = sz; round_cap_ = sz;}

  void run_build_bundle() {build_bundle();}

  const std::vector<std::string> & get_bundle() const {return bundle_;}
  double get_y(const std::string & task) const {return y_.at(task);}
  const std::string & get_z(const std::string & task) const {return z_.at(task);}
};

static as2_msgs::msg::Bid make_bid(
  const std::vector<std::string> & names,
  const std::vector<double> & amounts,
  const std::vector<std::string> & winners)
{
  as2_msgs::msg::Bid bid;
  bid.name = names;
  bid.amounts = amounts;
  bid.winners = winners;
  return bid;
}

// ── tests ─────────────────────────────────────────────────────────────────────

class CBBATest : public ::testing::Test
{
protected:
  TestablePlugin plugin_;
};

TEST_F(CBBATest, NoConvergenceWithoutItems)
{
  EXPECT_FALSE(plugin_.check_convergence());
}

TEST_F(CBBATest, NoConvergenceWithoutParticipants)
{
  plugin_.add_item("task_a", 1.0f);
  plugin_.run_build_bundle();
  EXPECT_FALSE(plugin_.check_convergence());
}

TEST_F(CBBATest, ConvergesSoleParticipant)
{
  // With only self in participants_, there are no peers to wait for.
  // changed_ is false after build_bundle() (no update() called), so converge immediately.
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_item("task_a", 1.0f);
  plugin_.run_build_bundle();

  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(CBBATest, BundleBuildingPicksLowestCost)
{
  // Lower cost = better. Agent should pick task_b (cost 1 < cost 5).
  plugin_.set_namespace("drone0");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 5.0f);
  plugin_.add_item("task_b", 1.0f);
  plugin_.run_build_bundle();

  ASSERT_EQ(plugin_.get_bundle().size(), 1u);
  EXPECT_EQ(plugin_.get_bundle()[0], "task_b");
  EXPECT_EQ(plugin_.get_z("task_b"), "drone0");
  EXPECT_DOUBLE_EQ(plugin_.get_y("task_b"), 1.0);
}

TEST_F(CBBATest, BundleBuildingMultiTask)
{
  // Tasks placed on the X-axis: task_b@(2,0), task_c@(5,0), task_a@(10,0).
  // bundle_size=2: first pick is task_b (marginal from start=2, cheapest).
  // Second pick uses marginal insertion into path=[task_b]:
  //   task_c: append=dist(task_b,task_c)=3; prepend=5+3-2=6  → best=3
  //   task_a: append=dist(task_b,task_a)=8; prepend=10+8-2=16 → best=8
  //   → picks task_c (3 < 8).
  plugin_.set_namespace("drone0");
  plugin_.set_bundle_size(2);
  plugin_.add_item("task_a", 10.0f);
  plugin_.add_item("task_b", 2.0f);
  plugin_.add_item("task_c", 5.0f);
  plugin_.run_build_bundle();

  ASSERT_EQ(plugin_.get_bundle().size(), 2u);
  EXPECT_EQ(plugin_.get_bundle()[0], "task_b");  // added first (smallest marginal from start)
  EXPECT_EQ(plugin_.get_bundle()[1], "task_c");  // smallest marginal insertion into [task_b]
}

TEST_F(CBBATest, BundleDoesNotExceedSize)
{
  plugin_.set_namespace("drone0");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 1.0f);
  plugin_.add_item("task_b", 2.0f);
  plugin_.add_item("task_c", 3.0f);
  plugin_.run_build_bundle();

  EXPECT_EQ(plugin_.get_bundle().size(), 1u);
}

TEST_F(CBBATest, ConsensusOutbid)
{
  // drone0 claims task_a (cost=1.0). drone1 announces a lower cost=0.5 → drone1 wins.
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 1.0f);
  plugin_.run_build_bundle();

  ASSERT_EQ(plugin_.get_bundle().size(), 1u);

  // drone1 bids 0.5 for task_a — lower than drone0's 1.0.
  auto bid = make_bid({"task_a"}, {0.5}, {"drone1"});
  plugin_.update(bid, "drone1");

  EXPECT_EQ(plugin_.get_z("task_a"), "drone1");
  EXPECT_DOUBLE_EQ(plugin_.get_y("task_a"), 0.5);
  // drone0 lost task_a and has no other task to claim → bundle empty.
  EXPECT_TRUE(plugin_.get_bundle().empty());
}

TEST_F(CBBATest, CascadeRemoval)
{
  // Use a non-collinear layout to avoid zero-marginal re-claims:
  //
  //   start (0,0)
  //   task_b (0, 3)   — directly above start; dist(start,task_b)=3
  //   task_c (4, 0)   — directly right;       dist(start,task_c)=4
  //   task_a (10, 0)  — far right
  //
  // task_b is clearly off the path to task_c, so after cascade (path=[task_c])
  // the marginal of re-inserting task_b is always > 2.0 (well above drone1's 1.0).
  //
  // Initial bundle build:
  //   marginals from []: task_b=3, task_c=4, task_a=10 → pick task_b first, task_c second.
  //   (marginal of task_c into [task_b]: append=dist(task_b,task_c)=5; prepend=4+5-3=6 → 5)
  //   (marginal of task_a into [task_b]: append=dist(task_b,task_a)≈10.4; … → ~10.4)
  //
  // After drone1 outbids task_b (1.0 < 3.0): cascade removes [task_b, task_c].
  //   build_bundle with preserved y_/z_:
  //     task_b: already_winner? No. 3.0 < 1.0? No → not claimable.
  //     task_c: already_winner (z_="drone0") → re-add.
  //     task_a: marginal 5.0 < inf → re-add after task_c (fits bundle_size=2).
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(2);
  plugin_.add_item("task_a", 10.0f);
  plugin_.add_item("task_b", 3.0f);
  plugin_.add_item("task_c", 4.0f);
  plugin_.set_item_pos("task_b", 0.0, 3.0);   // directly above start
  plugin_.set_item_pos("task_c", 4.0, 0.0);   // directly right of start
  plugin_.set_item_pos("task_a", 10.0, 0.0);  // far right
  plugin_.run_build_bundle();

  ASSERT_EQ(plugin_.get_bundle().size(), 2u);
  EXPECT_EQ(plugin_.get_bundle()[0], "task_b");
  EXPECT_EQ(plugin_.get_bundle()[1], "task_c");

  // drone1 outbids drone0 on task_b with a bid well below drone0's marginal (3.0).
  auto bid = make_bid({"task_b"}, {1.0}, {"drone1"});
  plugin_.update(bid, "drone1");

  // task_b belongs to drone1 now.
  EXPECT_EQ(plugin_.get_z("task_b"), "drone1");
  // task_c: cascade released it from the bundle but drone0 is still the consensus
  // winner (z="drone0"). build_bundle re-adds it via the already_winner path.
  EXPECT_EQ(plugin_.get_z("task_c"), "drone0");
}

TEST_F(CBBATest, NoPeerChangeMeansConverged)
{
  // After receiving a bid that changes nothing, convergence should be declared.
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 5.0f);
  plugin_.add_item("task_b", 1.0f);  // drone0 claims this (cheaper)
  plugin_.run_build_bundle();

  // drone1 sends: it agrees drone0 wins task_b (cost=1.0 same), and claims task_a (cost=5.0).
  // y[task_b]=1.0 from drone1 is NOT < y[task_b]=1.0 (drone0's bid) → no update.
  // y[task_a]=5.0 from drone1 < inf → updates task_a to drone1.
  // After update: changed_=true (task_a assignment changed). So NOT converged yet.
  // Send again with no further changes: drone1 sends same state.
  auto bid = make_bid(
    {"task_a", "task_b"},
    {5.0, 1.0},
    {"drone1", "drone0"});
  plugin_.update(bid, "drone1");
  // First update: task_a changed (inf → 5.0). Not converged.
  // But we need to call update again to get a stable round.
  // Second call with same data: no changes.
  plugin_.update(bid, "drone1");

  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(CBBATest, TwoAgentsMutuallyExclusiveAssignment)
{
  // drone0 is cheaper on task_b (cost=1). Drone1 sends a better bid on task_a (cost=0.5).
  // Expected: drone0 keeps task_b, drone1 wins task_a.
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 2.0f);  // drone0 cost: 2.0
  plugin_.add_item("task_b", 1.0f);  // drone0 cost: 1.0  ← drone0 picks this

  plugin_.run_build_bundle();
  ASSERT_EQ(plugin_.get_bundle().size(), 1u);
  EXPECT_EQ(plugin_.get_bundle()[0], "task_b");

  // drone1 bids: it wins task_a (cost=0.5 < drone0's 2.0); it agrees drone0 wins task_b.
  auto bid = make_bid(
    {"task_a", "task_b"},
    {0.5, 1.0},
    {"drone1", "drone0"});
  plugin_.update(bid, "drone1");

  // Stable state: no update on task_b (1.0 == 1.0, but z_k[task_b]=="drone0"==z_[task_b]).
  // task_a: 0.5 < inf → updated to drone1.
  // drone0 did not lose task_b → no cascade → bundle stays [task_b].
  EXPECT_EQ(plugin_.get_z("task_b"), "drone0");
  EXPECT_EQ(plugin_.get_z("task_a"), "drone1");
  ASSERT_EQ(plugin_.get_bundle().size(), 1u);
  EXPECT_EQ(plugin_.get_bundle()[0], "task_b");

  // Second update with same data: no state changes → converged.
  plugin_.update(bid, "drone1");
  EXPECT_TRUE(plugin_.check_convergence());
}

TEST_F(CBBATest, LexicographicTieBreak)
{
  // Both drones have the same cost for task_a. "drone0" < "drone1" → drone0 wins.
  plugin_.set_namespace("drone1");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 1.0f);  // same cost for both agents
  plugin_.run_build_bundle();

  // drone1 claimed task_a (y=1.0, z=drone1).
  ASSERT_EQ(plugin_.get_bundle().size(), 1u);

  // drone0 sends the same cost 1.0 and claims task_a. "drone0" < "drone1" → tie-break wins.
  auto bid = make_bid({"task_a"}, {1.0}, {"drone0"});
  plugin_.update(bid, "drone0");

  // z should switch to drone0 (lexicographically smaller).
  EXPECT_EQ(plugin_.get_z("task_a"), "drone0");
  // Cascade releases task_a from drone1's bundle.
  EXPECT_TRUE(plugin_.get_bundle().empty());
}

TEST_F(CBBATest, GlobalAssignmentOnlyContainsWinners)
{
  plugin_.set_namespace("drone0");
  plugin_.add_participant("/drone0");
  plugin_.add_participant("/drone1");
  plugin_.set_bundle_size(1);
  plugin_.add_item("task_a", 2.0f);
  plugin_.add_item("task_b", 1.0f);  // drone0 picks this

  plugin_.run_build_bundle();

  // drone1 wins task_a (cost 0.5 < drone0's 2.0).
  auto bid = make_bid({"task_a", "task_b"}, {0.5, 1.0}, {"drone1", "drone0"});
  plugin_.update(bid, "drone1");

  auto assignment = plugin_.get_global_assignment();
  ASSERT_EQ(assignment.count("task_a"), 1u);
  ASSERT_EQ(assignment.count("task_b"), 1u);
  EXPECT_EQ(assignment["task_a"], "drone1");
  EXPECT_EQ(assignment["task_b"], "drone0");
}
