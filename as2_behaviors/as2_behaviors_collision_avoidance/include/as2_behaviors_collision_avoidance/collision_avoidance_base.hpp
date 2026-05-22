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

struct Status
{
  std::string state;                        // "IDLE"|"REQUESTING"|"HOLDING"|"RELEASING"
  bool lock_held{false};
  std::vector<std::string> pending_peers;
  std::vector<std::string> conflicting_peers;
  uint32_t deferred_count{0};
};

class CollisionAvoidanceBase
{
public:
  virtual ~CollisionAvoidanceBase() = default;

  virtual void initialize(
    rclcpp::Node * node,
    std::shared_ptr<as2_ca::CAGatewayClient> ca_client,
    const std::string & own_id) = 0;

  virtual void start_acquisition(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & path,
    const std::vector<std::string> & peer_snapshot,
    uint32_t req_id,
    double safety_distance) = 0;

  virtual void release() = 0;

  virtual void on_request(
    const as2_msgs::msg::CAPathLockRequest & req,
    const std::string & sender) = 0;

  virtual void on_grant(
    const as2_msgs::msg::CAPathLockGrant & grant,
    const std::string & sender) = 0;

  virtual void on_release(
    const as2_msgs::msg::CAPathLockRelease & rel,
    const std::string & sender) = 0;

  virtual Status status() const = 0;
};

}  // namespace collision_avoidance_base
