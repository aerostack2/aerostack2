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
 *  \file       auction_item_plugin_base.hpp
 *  \brief      auction item plugin base header file
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AS2_AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_
#define AS2_AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "as2_core/state_interface.hpp"
#include "as2_msgs/msg/auction_item.hpp"

namespace as2_auction_behavior
{

/**
 * @brief Abstract base for auction item plugins.
 *
 * An item plugin encapsulates a single auctionable entity and the cost metric
 * used to evaluate it from this drone's perspective.  Decoupling cost
 * computation from the auction algorithm allows the same @ref
 * AuctionBehaviorPluginBase to be reused with different item representations
 * (e.g. 2-D waypoints, panel IDs, GPS coordinates) without modification.
 *
 * Each concrete plugin must:
 * - Implement @ref create to deserialize an @c AuctionItem message into a
 *   typed instance of itself.
 * - Implement @ref evaluate to return a scalar cost (lower is preferred) from
 *   the drone's current state as exposed by @ref StateInterface.
 *
 * Plugins are loaded by name via @c pluginlib; the class name must be
 * registered in @c plugins.xml as @c <item_type>::Plugin.
 */
class AuctionItemPluginBase
{
public:
  virtual ~AuctionItemPluginBase() = default;

  /**
   * @brief Deserialize an @c AuctionItem message into a typed plugin instance.
   *
   * Acts as a factory method: the plugin loaded by @c pluginlib calls @c create
   * to produce one typed item object per element in the auction item list.
   *
   * @param item_msg Serialized item message from a @c StartAuction payload.
   * @return Shared pointer to a new, fully initialized item instance.
   */
  virtual std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem & item_msg) const = 0;

  /**
   * @brief Compute the cost of acquiring this item from the drone's current state.
   *
   * Lower cost means higher preference.  The method reads whatever state
   * fields it needs from @p state_interface (e.g. current pose for Euclidean
   * distance).
   *
   * @param state_interface Self-model providing the drone's current state.
   * @return Non-negative scalar cost; lower values indicate higher preference.
   */
  virtual float evaluate(
    const StateInterface & state_interface) const = 0;

  /** @brief Return the unique name identifying this item (used in @c Bid messages). */
  virtual std::string get_name() const = 0;

  /** @brief Return a human-readable string description of this item for logging. */
  virtual std::string to_string() const = 0;

  /** @brief Return the underlying @c AuctionItem message (used in result publishing). */
  virtual as2_msgs::msg::AuctionItem get_item() const = 0;

protected:
  AuctionItemPluginBase() {}
};

}  // namespace as2_auction_behavior

#endif  // AS2_AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_
