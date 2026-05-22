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
 *  \file       collision_avoidance_base.hpp
 *  \brief      Collision avoidance base plugin class
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <as2_msgs/msg/pose_stamped_with_id.hpp>
#include <as2_msgs/action/collision_avoidance.hpp>
#include <as2_msgs/msg/ca_path_lock_request.hpp>
#include <as2_msgs/msg/ca_path_lock_grant.hpp>
#include <as2_msgs/msg/ca_path_lock_release.hpp>
#include <as2_ca/ca_gateway_client.hpp>

namespace collision_avoidance_base
{

/**
 * @brief Snapshot of the plugin's current lock-arbitration state.
 *
 * Returned by @ref CollisionAvoidanceBase::status and forwarded verbatim
 * into the @c CollisionAvoidance action feedback.
 */
struct Status
{
  std::string state;                       ///< Plugin phase: "IDLE"|"REQUESTING"|"HOLDING"|"RELEASING"
  bool lock_held{false};                   ///< @c true when the local agent holds the path lock
  std::vector<std::string> pending_peers;  ///< Peers from whom a grant has not yet been received
  std::vector<std::string> conflicting_peers; ///< Peers whose path overlaps with ours
  uint32_t deferred_count{0};             ///< Number of grant messages deferred to unblock peers
};

/**
 * @brief Abstract base for collision avoidance arbitration plugins.
 *
 * A collision avoidance plugin implements the *consensus* collective model
 * from the CORESENSE Collective Awareness architecture.  It arbitrates access
 * to a flight corridor by exchanging three modelet types with peers via the
 * @ref as2_ca::CAGatewayClient:
 *
 * - @c CAPathLockRequest  — sent to all peers when this agent wants to fly a path.
 * - @c CAPathLockGrant    — sent back to a requester when this agent approves.
 * - @c CAPathLockRelease  — broadcast when the lock is released after motion.
 *
 * The @ref CollisionAvoidanceBehavior calls these methods in response to CA
 * Gateway callbacks and behavior lifecycle events (@c on_activate /
 * @c on_pause / @c on_deactivate).  The behavior drives the motion phase
 * (via @c GoToWaypoint) only after @ref status returns @c lock_held = @c true.
 *
 * Plugins are loaded by name via @c pluginlib.
 */
class CollisionAvoidanceBase
{
public:
  virtual ~CollisionAvoidanceBase() = default;

  /**
   * @brief Initialize the plugin with its runtime dependencies.
   *
   * @param node      Parent behavior node (used for logging and parameter access).
   * @param ca_client Shared CA Gateway client used to send lock messages.
   * @param own_id    This drone's namespace without leading '/'.
   */
  virtual void initialize(
    rclcpp::Node * node,
    std::shared_ptr<as2_ca::CAGatewayClient> ca_client,
    const std::string & own_id) = 0;

  /**
   * @brief Begin the lock-acquisition process for the given path.
   *
   * Sends a @c CAPathLockRequest to all peers in @p peer_snapshot and
   * transitions the plugin to the REQUESTING state.  The behavior's @c on_run
   * loop polls @ref status until @c lock_held becomes @c true.
   *
   * @param path           Sequence of waypoints whose corridor must be locked,
   *                       prepended with the drone's current pose by the behavior.
   * @param peer_snapshot  Peer namespaces known at request time (CA Gateway snapshot).
   * @param req_id         Monotonically increasing request identifier to correlate
   *                       grants with the correct request.
   * @param safety_distance Minimum separation distance (metres) used to detect
   *                       path conflicts.
   */
  virtual void start_acquisition(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & path,
    const std::vector<std::string> & peer_snapshot,
    uint32_t req_id,
    double safety_distance) = 0;

  /**
   * @brief Release the held lock and notify waiting peers.
   *
   * Sends @c CAPathLockRelease to all peers that were deferred during
   * acquisition and transitions to the RELEASING / IDLE state.
   */
  virtual void release() = 0;

  /**
   * @brief Handle an inbound @c CAPathLockRequest from a peer.
   * @param req    Incoming lock request.
   * @param sender Namespace of the requesting agent.
   */
  virtual void on_request(
    const as2_msgs::msg::CAPathLockRequest & req,
    const std::string & sender) = 0;

  /**
   * @brief Handle an inbound @c CAPathLockGrant from a peer.
   * @param grant  Incoming grant message.
   * @param sender Namespace of the granting agent.
   */
  virtual void on_grant(
    const as2_msgs::msg::CAPathLockGrant & grant,
    const std::string & sender) = 0;

  /**
   * @brief Handle an inbound @c CAPathLockRelease from a peer.
   * @param rel    Incoming release message.
   * @param sender Namespace of the releasing agent.
   */
  virtual void on_release(
    const as2_msgs::msg::CAPathLockRelease & rel,
    const std::string & sender) = 0;

  /** @brief Return a snapshot of the plugin's current arbitration state. */
  virtual Status status() const = 0;
};

}  // namespace collision_avoidance_base
