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
 *  \file       collision_avoidance_behavior.cpp
 *  \brief      Collision avoidance behavior implementation
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <memory>
#include <string>

#include "as2_behaviors_collision_avoidance/collision_avoidance_behavior.hpp"
#include <as2_msgs/msg/ca_path_lock_request.hpp>
#include <as2_msgs/msg/ca_path_lock_grant.hpp>
#include <as2_msgs/msg/ca_path_lock_release.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

namespace collision_avoidance_behavior
{

CollisionAvoidanceBehavior::CollisionAvoidanceBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<CollisionAvoidanceAction>("CollisionAvoidanceBehavior", options)
{
  // Derive own_id from the node namespace, stripping leading '/'.
  own_id_ = std::string(get_namespace());
  if (!own_id_.empty() && own_id_.front() == '/') {
    own_id_ = own_id_.substr(1);
  }

  if (!state_interface_.configure(
      this, {
      as2_names::topics::self_localization::pose,
      as2_names::topics::self_localization::twist}))
  {
    RCLCPP_ERROR(get_logger(), "StateInterface configuration failed");
  }

  // Create plugin loader.
  loader_ = std::make_unique<
    pluginlib::ClassLoader<collision_avoidance_base::CollisionAvoidanceBase>>(
    "as2_behaviors_collision_avoidance",
    "collision_avoidance_base::CollisionAvoidanceBase");

  // Load default plugin from parameter (with safety check for empty string)
  declare_parameter<std::string>("plugin_name", "pairwise_path_lock_plugin::Plugin");
  std::string default_plugin_name = get_parameter("plugin_name").as_string();
  if (default_plugin_name.empty()) {
    RCLCPP_WARN(get_logger(), "plugin_name parameter is empty, using hardcoded default");
    default_plugin_name = "pairwise_path_lock_plugin::Plugin";
  }
  load_plugin(default_plugin_name);

  // Create the as2_ca gateway client.
  ca_client_ = std::make_shared<as2_ca::CAGatewayClient>(this);

  // Register three message types — callbacks forward to the plugin.
  ca_client_->register_module<as2_msgs::msg::CAPathLockRequest>(
    "ca_lock_request", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockRequest & msg, const std::string & sender) {
      if (plugin_) {
        plugin_->on_request(msg, sender);
      }
    });

  ca_client_->register_module<as2_msgs::msg::CAPathLockGrant>(
    "ca_lock_grant", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockGrant & msg, const std::string & sender) {
      if (plugin_) {
        plugin_->on_grant(msg, sender);
      }
    });

  ca_client_->register_module<as2_msgs::msg::CAPathLockRelease>(
    "ca_lock_release", "collision_avoidance",
    [this](const as2_msgs::msg::CAPathLockRelease & msg, const std::string & sender) {
      if (plugin_) {
        plugin_->on_release(msg, sender);
      }
    });

  plugin_->initialize(this, ca_client_, own_id_);

  // Create GoToWaypoint action client for internal motion orchestration.
  go_to_client_ = rclcpp_action::create_client<as2_msgs::action::GoToWaypoint>(
    this,
    "GoToBehavior");

  goto_path_pause_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::gotowaypoint) + "/_behavior/pause", this);

  goto_path_resume_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::gotowaypoint) + "/_behavior/resume", this);


  RCLCPP_INFO(get_logger(), "CollisionAvoidanceBehavior ready (id=%s)", own_id_.c_str());
}

bool CollisionAvoidanceBehavior::on_activate(
  std::shared_ptr<const CollisionAvoidanceAction::Goal> goal)
{
  if (goal->path.empty()) {
    RCLCPP_ERROR(get_logger(), "on_activate: path is empty");
    return false;
  }

  released_ = false;
  current_phase_ = REQUESTING_LOCK;
  motion_rejected_ = false;
  motion_succeeded_ = false;
  motion_aborted_ = false;
  go_to_paused_ = false;
  go_to_goal_handle_ = nullptr;

  // Check if goal specifies a different plugin and reload if needed
  if (!goal->plugin_name.empty() && goal->plugin_name != current_plugin_name_) {
    RCLCPP_INFO(
      get_logger(), "Switching plugin from '%s' to '%s'",
      current_plugin_name_.c_str(), goal->plugin_name.c_str());
    if (!load_plugin(goal->plugin_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to load plugin '%s'", goal->plugin_name.c_str());
      return false;
    }
  }

  double safety_dist = (goal->safety_distance > 0.0f) ?
    static_cast<double>(goal->safety_distance) :
    get_parameter_or("safety_distance", 1.5);

  // Prepend current drone pose to the path so the lock covers the full
  // segment from the drone's current position to the first waypoint.
  // This prevents a peer from crossing that opening segment during acquisition.
  auto full_path = goal->path;
  try {
    auto current_pose = state_interface_.get_value<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose);
    as2_msgs::msg::PoseStampedWithID pose_with_id;
    pose_with_id.id = "current_position";
    pose_with_id.pose = current_pose;
    full_path.insert(full_path.begin(), pose_with_id);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not prepend current pose to path: %s", e.what());
    // Continue with caller-provided path — not a fatal error.
  }

  stored_path_ = full_path;
  stored_safety_dist_ = safety_dist;

  auto peers = ca_client_->get_known_peers();  // snapshot at request time
  plugin_->start_acquisition(full_path, peers, ++req_id_counter_, safety_dist);
  return true;
}

bool CollisionAvoidanceBehavior::on_modify(
  std::shared_ptr<const CollisionAvoidanceAction::Goal>/*goal*/)
{
  RCLCPP_WARN(get_logger(), "on_modify: not supported in v1, lock is tied to the original path");
  return false;
}

bool CollisionAvoidanceBehavior::on_deactivate(const std::shared_ptr<std::string> & /*msg*/)
{
  // Cancel any active GoTo goal asynchronously (works whether GoTo is RUNNING
  // or already PAUSED; avoids the synchronous pause_go_to() service call that
  // fails with "Behavior is not running" when GoTo was previously paused).
  if (go_to_goal_handle_) {
    try {
      go_to_client_->async_cancel_goal(go_to_goal_handle_);
    } catch (const std::exception & e) {
      // Defensive: the action client may have already pruned this goal handle
      // (e.g. result arrived just before STOP); cancelling a finished/unknown
      // goal must never crash the whole behavior node.
      RCLCPP_WARN(
        get_logger(), "on_deactivate: cancel of GoTo goal failed (likely already finished): %s",
        e.what());
    }
    go_to_goal_handle_ = nullptr;
  }
  if (!released_) {
    plugin_->release();
    released_ = true;
  }
  current_phase_ = RELEASING_LOCK;
  motion_aborted_ = true;

  // When stop arrives while the behavior is PAUSED, run()'s guard
  // (status != RUNNING → return early) would prevent on_run from ever firing,
  // so the action result would never be sent and Python's wait_to_result()
  // would block forever.  Restore RUNNING so the run timer drives the
  // RELEASING_LOCK → SUCCESS path and delivers the result to the client.
  if (behavior_status_.status == BehaviorStatus::PAUSED) {
    behavior_status_.status = BehaviorStatus::RUNNING;
  }
  return false;
}

bool CollisionAvoidanceBehavior::on_pause(const std::shared_ptr<std::string> & /*msg*/)
{
  // Pause GoTo only if it is currently executing motion; otherwise it is not
  // running and there is nothing to pause (vacuously succeeded).
  bool go_to_paused = true;
  if (current_phase_ == EXECUTING_MOTION) {
    RCLCPP_INFO(get_logger(), "[PAUSE] GoTo is active — requesting GoToBehavior pause...");
    go_to_paused = pause_go_to();
    if (go_to_paused) {
      go_to_paused_ = true;
      RCLCPP_INFO(get_logger(), "[PAUSE] GoToBehavior paused successfully");
    } else {
      RCLCPP_WARN(get_logger(), "[PAUSE] GoToBehavior pause failed");
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "[PAUSE] GoTo not active (phase=%d) — no GoTo pause needed", current_phase_);
  }

  // Release locks unconditionally — regardless of GoTo result the drone is
  // either stopped or was never moving, so peers can safely claim the path.
  if (!released_) {
    RCLCPP_INFO(get_logger(), "[PAUSE] Releasing path lock");
    plugin_->release();
    released_ = true;
    RCLCPP_INFO(get_logger(), "[PAUSE] Path lock released");
  } else {
    RCLCPP_INFO(get_logger(), "[PAUSE] Lock was already released");
  }

  RCLCPP_INFO(
    get_logger(), "[PAUSE] Result: %s (go_to_paused=%s)",
    go_to_paused ? "PAUSED" : "FAILED", go_to_paused ? "true" : "false");
  return go_to_paused;
}

bool CollisionAvoidanceBehavior::on_resume(const std::shared_ptr<std::string> & /*msg*/)
{
  // The lock was released on pause.  Re-acquire it before letting the drone
  // move again.  on_run will call resume_go_to() once the lock is held.
  RCLCPP_INFO(
    get_logger(),
    "[RESUME] Re-acquiring path lock before resuming motion (path_size=%zu, safety_dist=%.2f m)",
    stored_path_.size(), stored_safety_dist_);
  released_ = false;
  current_phase_ = REQUESTING_LOCK;
  auto peers = ca_client_->get_known_peers();
  plugin_->start_acquisition(stored_path_, peers, ++req_id_counter_, stored_safety_dist_);
  RCLCPP_INFO(
    get_logger(),
    "[RESUME] Lock acquisition started (req_id=%u, peers=%zu) — waiting for grant",
    req_id_counter_, peers.size());
  return true;
}

as2_behavior::ExecutionStatus CollisionAvoidanceBehavior::on_run(
  const std::shared_ptr<const CollisionAvoidanceAction::Goal> & goal,
  std::shared_ptr<CollisionAvoidanceAction::Feedback> & feedback,
  std::shared_ptr<CollisionAvoidanceAction::Result> & result)
{
  auto s = plugin_->status();
  feedback->state = s.state;
  feedback->lock_held = s.lock_held;
  feedback->pending_peers = s.pending_peers;
  feedback->conflicting_peers = s.conflicting_peers;
  feedback->deferred_count = s.deferred_count;
  feedback->motion_in_progress = (current_phase_ == EXECUTING_MOTION);

  // Phase 1: REQUESTING_LOCK — wait for lock acquisition from peers
  if (current_phase_ == REQUESTING_LOCK) {
    if (s.lock_held) {
      // Lock acquired, transition to HOLDING_LOCK and send GoTo goal if motion requested
      current_phase_ = HOLDING_LOCK;
      RCLCPP_INFO(
        get_logger(),
        "✓ Lock ACQUIRED [path safety_distance=%.2f m, plugin=%s]",
        goal->safety_distance, current_plugin_name_.c_str());

      if (go_to_paused_) {
        // Re-acquired lock after a pause; resume the paused GoTo.
        RCLCPP_INFO(get_logger(), "[RESUME] Lock re-acquired — resuming GoToBehavior");
        go_to_paused_ = false;
        if (resume_go_to()) {
          RCLCPP_INFO(get_logger(), "[RESUME] GoToBehavior resumed — motion continuing");
          current_phase_ = EXECUTING_MOTION;
        } else {
          RCLCPP_WARN(
            get_logger(),
            "[RESUME] GoToBehavior resume failed — aborting and releasing lock");
          current_phase_ = RELEASING_LOCK;
          motion_aborted_ = true;
        }
        return as2_behavior::ExecutionStatus::RUNNING;
      }

      // Check if motion target is requested (non-zero position)
      bool has_motion_target =
        (goal->goal_pose.pose.position.x != 0.0 ||
        goal->goal_pose.pose.position.y != 0.0 ||
        goal->goal_pose.pose.position.z != 0.0);

      if (has_motion_target) {
        // Send GoToWaypoint goal
        auto go_to_goal = as2_msgs::action::GoToWaypoint::Goal();
        go_to_goal.target_pose.header = goal->goal_pose.header;
        go_to_goal.target_pose.point.x = goal->goal_pose.pose.position.x;
        go_to_goal.target_pose.point.y = goal->goal_pose.pose.position.y;
        go_to_goal.target_pose.point.z = goal->goal_pose.pose.position.z;
        go_to_goal.max_speed = goal->max_speed > 0.0f ? goal->max_speed : 1.0f;
        // Set default yaw mode (keep current yaw)
        go_to_goal.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;

        RCLCPP_INFO(
          get_logger(),
          " Starting motion: target=(%.2f, %.2f, %.2f) m, max_speed=%.2f m/s",
          go_to_goal.target_pose.point.x,
          go_to_goal.target_pose.point.y,
          go_to_goal.target_pose.point.z,
          go_to_goal.max_speed);

        if (!go_to_client_->wait_for_action_server(std::chrono::seconds(1))) {
          RCLCPP_WARN(
            get_logger(),
            " GoToWaypoint action server not available, skipping motion");
          current_phase_ = RELEASING_LOCK;
          motion_aborted_ = true;
        } else {
          auto send_goal_options =
            rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SendGoalOptions();
          send_goal_options.goal_response_callback =
            std::bind(&CollisionAvoidanceBehavior::go_to_response_cbk, this, std::placeholders::_1);
          send_goal_options.result_callback =
            std::bind(&CollisionAvoidanceBehavior::go_to_result_cbk, this, std::placeholders::_1);
          go_to_client_->async_send_goal(go_to_goal, send_goal_options);
          current_phase_ = EXECUTING_MOTION;
          RCLCPP_INFO(get_logger(), "✓ Motion goal sent to GoToBehavior");
        }
      } else {
        // No motion requested, go directly to releasing
        current_phase_ = RELEASING_LOCK;
        motion_succeeded_ = true;  // No motion = success
        RCLCPP_INFO(
          get_logger(),
          "→ No motion target specified, proceeding directly to release");
      }
    } else {
      // Still waiting for lock — throttle to avoid spam (once every 2 seconds)
      std::string granted_by;
      for (const auto & peer : s.granted_peers) {
        if (!granted_by.empty()) granted_by += ", ";
        granted_by += peer;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        " Waiting for lock: pending=%zu, granted_by=[%s], conflicting=%zu, locked=%s",
        s.pending_peers.size(), granted_by.c_str(), s.conflicting_peers.size(),
        s.lock_held ? "true" : "false");
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // Phase 2: HOLDING_LOCK — (handled above when transitioning to EXECUTING_MOTION)
  // This phase is brief and transitions to EXECUTING_MOTION or RELEASING_LOCK

  // Phase 3: EXECUTING_MOTION — wait for GoToBehavior callbacks to set flags
  if (current_phase_ == EXECUTING_MOTION) {
    if (motion_rejected_ || motion_aborted_) {
      RCLCPP_WARN(get_logger(), "⚠ Motion rejected or aborted, proceeding to release");
      current_phase_ = RELEASING_LOCK;
    } else if (motion_succeeded_) {
      RCLCPP_INFO(get_logger(), "✓ Motion completed successfully");
      current_phase_ = RELEASING_LOCK;
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "⏳ Motion in progress (GoToBehavior executing)...");
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // Phase 4: RELEASING_LOCK — release the path lock
  if (current_phase_ == RELEASING_LOCK) {
    if (!released_) {
      plugin_->release();
      released_ = true;
      RCLCPP_INFO(
        get_logger(),
        " Lock release triggered (deferred_peers=%u, lock_held=%s)",
        s.deferred_count,
        s.lock_held ? "true" : "false");
    }

    // Check if lock is fully released (plugin returns to IDLE)
    if (s.state == "IDLE") {
      result->collision_avoidance_success = motion_succeeded_;
      std::string status_msg = motion_succeeded_ ? "SUCCESS" : "FAILED";
      RCLCPP_INFO(
        get_logger(),
        "✓ Collision avoidance %s [lock_held=%s, motion=%s]",
        status_msg.c_str(),
        s.lock_held ? "true" : "false",
        motion_succeeded_ ? "completed" : "skipped/failed");
      return as2_behavior::ExecutionStatus::SUCCESS;
    } else {
      // Still releasing — throttle to avoid spam (once every 2 seconds)
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        " Releasing lock: state=%s, deferred=%u",
        s.state.c_str(), s.deferred_count);
    }
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}

void CollisionAvoidanceBehavior::on_execution_end(
  const as2_behavior::ExecutionStatus & /*state*/)
{
  if (!released_) {
    plugin_->release();
    released_ = true;
  }
  // StateInterface subscriptions are kept alive for the node lifetime so the
  // next on_activate has fresh pose data immediately. Call clear() only if the
  // node itself is shutting down (handled by the destructor implicitly).
}

void CollisionAvoidanceBehavior::go_to_response_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "⚠ GoToBehavior rejected the goal");
    motion_rejected_ = true;
  } else {
    go_to_goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "✓ GoToBehavior accepted goal, flying to target");
  }
}

bool CollisionAvoidanceBehavior::pause_go_to()
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool ok = goto_path_pause_client_->sendRequest(req, res, 3);
  if (!ok) {
    RCLCPP_WARN(get_logger(), "[PAUSE] pause_go_to: service call timed out or failed");
  } else if (!res.success) {
    RCLCPP_WARN(
      get_logger(), "[PAUSE] pause_go_to: service returned failure: %s", res.message.c_str());
  }
  return ok && res.success;
}

bool CollisionAvoidanceBehavior::resume_go_to()
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool ok = goto_path_resume_client_->sendRequest(req, res, 3);
  if (!ok) {
    RCLCPP_WARN(get_logger(), "[RESUME] resume_go_to: service call timed out or failed");
  } else if (!res.success) {
    RCLCPP_WARN(
      get_logger(), "[RESUME] resume_go_to: service returned failure: %s", res.message.c_str());
  }
  return ok && res.success;
}

void CollisionAvoidanceBehavior::go_to_result_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult & result)
{
  // The goal has reached a terminal state; rclcpp_action drops it from its
  // internal tracking table, so the handle held here would become stale.
  // Clear it now so on_deactivate doesn't try to cancel an already-finished goal.
  go_to_goal_handle_ = nullptr;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      motion_succeeded_ = result.result->go_to_success;
      if (!motion_succeeded_) {
        motion_aborted_ = true;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "⚠ GoToBehavior aborted");
      motion_aborted_ = true;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "GoToBehavior canceled");
      motion_aborted_ = true;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "⚠ GoToBehavior unknown result code");
      motion_aborted_ = true;
      break;
  }
}

bool CollisionAvoidanceBehavior::load_plugin(const std::string & plugin_name)
{
  try {
    plugin_ = loader_->createSharedInstance(plugin_name);
    plugin_->initialize(this, ca_client_, own_id_);
    current_plugin_name_ = plugin_name;
    RCLCPP_INFO(get_logger(), "Plugin loaded successfully: %s", plugin_name.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load plugin '%s': %s", plugin_name.c_str(), e.what());
    return false;
  }
}

}  // namespace collision_avoidance_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  collision_avoidance_behavior::CollisionAvoidanceBehavior)
