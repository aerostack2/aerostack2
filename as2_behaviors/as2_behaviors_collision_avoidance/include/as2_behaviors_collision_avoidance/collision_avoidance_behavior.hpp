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
 *  \file       collision_avoidance_behavior.hpp
 *  \brief      Collision avoidance behavior node class
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

 #pragma once

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <as2_behavior/behavior_server.hpp>
#include <as2_msgs/action/collision_avoidance.hpp>
#include <as2_msgs/action/go_to_waypoint.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <as2_ca/ca_gateway_client.hpp>
#include <as2_core/state_interface.hpp>
#include <as2_core/names/topics.hpp>

#include "as2_core/synchronous_service_client.hpp"
#include "as2_behaviors_collision_avoidance/collision_avoidance_base.hpp"

namespace collision_avoidance_behavior
{

using CollisionAvoidanceAction = as2_msgs::action::CollisionAvoidance;

/**
 * @brief Aerostack2 behavior that arbitrates multi-drone path access before motion.
 *
 * @c CollisionAvoidanceBehavior implements the *consensus* collective model from
 * the CORESENSE Collective Awareness architecture.  Before commanding the drone
 * to fly a path it acquires a distributed path lock from all known peers via the
 * CA Gateway; motion begins only when all peers have granted the lock.
 *
 * **Lifecycle phases** (driven by @c on_run)
 * 1. **REQUESTING_LOCK** — plugin sends @c CAPathLockRequest to all peers and
 *    waits until @c lock_held = @c true.
 * 2. **HOLDING_LOCK** — lock acquired; sends a @c GoToWaypoint goal if the
 *    action goal specifies a motion target.
 * 3. **EXECUTING_MOTION** — waits for the @c GoToBehavior to complete.
 * 4. **RELEASING_LOCK** — calls @c plugin_->release() and waits for IDLE.
 *
 * **Pause / Resume**
 * On pause the behavior releases the lock and pauses @c GoToBehavior (if
 * running).  On resume it re-acquires the lock using the stored path before
 * resuming @c GoToBehavior, so no motion corridor is left unguarded.
 *
 * **Plugin architecture**
 * The arbitration algorithm is encapsulated in a @ref
 * collision_avoidance_base::CollisionAvoidanceBase plugin loaded via
 * @c pluginlib.  The default plugin is @c pairwise_path_lock_plugin::Plugin.
 *
 * The @ref StateInterface is used at activation time to read the drone's current
 * pose and prepend it to the path, ensuring the lock covers the full corridor
 * from the drone's current position to the first waypoint.
 */
class CollisionAvoidanceBehavior
  : public as2_behavior::BehaviorServer<CollisionAvoidanceAction>
{
public:
  explicit CollisionAvoidanceBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CollisionAvoidanceBehavior() override = default;

private:
  bool on_activate(std::shared_ptr<const CollisionAvoidanceAction::Goal> goal) override;
  bool on_modify(std::shared_ptr<const CollisionAvoidanceAction::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & msg) override;
  bool on_pause(const std::shared_ptr<std::string> & msg) override;
  bool on_resume(const std::shared_ptr<std::string> & msg) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const CollisionAvoidanceAction::Goal> & goal,
    std::shared_ptr<CollisionAvoidanceAction::Feedback> & feedback,
    std::shared_ptr<CollisionAvoidanceAction::Result> & result) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

  std::unique_ptr<pluginlib::ClassLoader<collision_avoidance_base::CollisionAvoidanceBase>>
  loader_;                                                          ///< Plugin class loader
  std::shared_ptr<collision_avoidance_base::CollisionAvoidanceBase> plugin_; ///< Active arbitration plugin

  std::shared_ptr<as2_ca::CAGatewayClient> ca_client_; ///< CA Gateway client for modelet exchange

  StateInterface state_interface_; ///< Self-model: subscribes to pose and twist topics

  rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SharedPtr go_to_client_;     ///< Client for internal motion
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr goto_path_pause_client_{nullptr};  ///< GoToBehavior pause service
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr goto_path_resume_client_{nullptr}; ///< GoToBehavior resume service

  std::string own_id_;             ///< Drone namespace without leading '/'
  std::string current_plugin_name_;///< Name of the currently loaded plugin
  uint32_t req_id_counter_{0};     ///< Monotonically increasing request ID for lock messages
  bool released_{false};           ///< @c true when the lock has been released in this activation

  std::vector<as2_msgs::msg::PoseStampedWithID> stored_path_; ///< Full path stored at activation for pause/resume
  double stored_safety_dist_{0.0};                            ///< Safety distance stored at activation

  /**
   * @brief Internal phase of the motion-orchestration state machine.
   *
   * Transitions: IDLE_PHASE → REQUESTING_LOCK → HOLDING_LOCK →
   *              EXECUTING_MOTION → RELEASING_LOCK → (SUCCESS / loop back).
   */
  enum MotionPhase { IDLE_PHASE, REQUESTING_LOCK, HOLDING_LOCK, EXECUTING_MOTION, RELEASING_LOCK };
  MotionPhase current_phase_{IDLE_PHASE};

  bool motion_rejected_{false};  ///< Set by @c go_to_response_cbk when GoTo rejects the goal
  bool motion_succeeded_{false}; ///< Set by @c go_to_result_cbk on SUCCEEDED result
  bool motion_aborted_{false};   ///< Set when GoTo is canceled or aborted
  bool go_to_paused_{false};     ///< @c true while GoTo is paused; resume after lock re-acquisition

  rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr
    go_to_goal_handle_{nullptr}; ///< Handle used to cancel in-flight GoTo on deactivate

  /** @brief GoToWaypoint response callback — sets @c go_to_goal_handle_ or @c motion_rejected_. */
  void go_to_response_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr & goal_handle);

  /** @brief GoToWaypoint result callback — sets @c motion_succeeded_ or @c motion_aborted_. */
  void go_to_result_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult & result);

  /** @brief Synchronously call the GoToBehavior pause service. @return @c true on success. */
  bool pause_go_to();

  /** @brief Synchronously call the GoToBehavior resume service. @return @c true on success. */
  bool resume_go_to();

  /**
   * @brief Load (or reload) the arbitration plugin by name.
   * @param plugin_name Fully qualified plugin class name (e.g. @c pairwise_path_lock_plugin::Plugin).
   * @return @c true if the plugin was loaded and initialized successfully.
   */
  bool load_plugin(const std::string & plugin_name);
};

}  // namespace collision_avoidance_behavior
