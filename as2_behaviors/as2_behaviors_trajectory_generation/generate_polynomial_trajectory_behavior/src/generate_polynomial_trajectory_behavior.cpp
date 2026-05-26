// Copyright 2026 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names
//    of its
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

/**
 * @file generate_polynomial_trajectory_behavior.cpp
 *
 * @brief Implementation of GeneratePolynomialTrajectoryBehavior.
 *
 * @authors Rafael Perez-Segui
 */

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <unordered_set>
#include <utility>

#include "as2_core/utils/frame_utils.hpp"

GeneratePolynomialTrajectoryBehavior::GeneratePolynomialTrajectoryBehavior(
  const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<
    as2_msgs::action::GeneratePolynomialTrajectory>(
    as2_names::actions::behaviors::trajectorygenerator, options),
  tf_handler_(this), trajectory_motion_handler_(this),
  hover_motion_handler_(this)
{
  // Load plugin
  loadPlugin();

  // Frames
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  map_frame_id_ = as2::tf::generateTfName(this, "map");

  std::string desired_frame_id;
  getParameter("desired_frame_id", desired_frame_id);
  RCLCPP_INFO(this->get_logger(), "Using desired_frame_id: %s", desired_frame_id.c_str());
  desired_frame_id_ = as2::tf::generateTfName(this, desired_frame_id);

  // Parameters
  getParameter("sampling_n", sampling_n_);
  getParameter("sampling_dt", sampling_dt_);
  getParameter("transform_threshold", transform_threshold_);
  getParameter("frequency_update_frame", frequency_update_frame_);
  getParameter("yaw_threshold", yaw_threshold_);
  getParameter("yaw_speed_threshold", yaw_speed_threshold_);
  getParameter("path_length", path_length_);

  // Parameters checks
  if (path_length_ < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "path_length must be >= 0, got %d. Disabling partial-trajectory feed.",
      path_length_);
    path_length_ = 0;
  }
  if (sampling_n_ < 1) {
    RCLCPP_ERROR(
      this->get_logger(), "Sampling n must be >= 1, got %d",
      sampling_n_);
    sampling_n_ = 1;
  }

  RCLCPP_INFO(
    this->get_logger(), "Sampling with n=%d and dt=%.4fs",
    sampling_n_, sampling_dt_);

  // Initialize debug publishers
  initDebugPublishers();

  // Subscriptions
  state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    as2_names::topics::self_localization::twist,
    as2_names::topics::self_localization::qos,
    std::bind(
      &GeneratePolynomialTrajectoryBehavior::stateCallback, this,
      std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "motion_reference/yaw", rclcpp::SensorDataQoS(),
    std::bind(
      &GeneratePolynomialTrajectoryBehavior::yawCallback, this,
      std::placeholders::_1));

  mod_waypoint_sub_ =
    this->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    as2_names::topics::motion_reference::modify_waypoint,
    as2_names::topics::motion_reference::qos_waypoint,
    std::bind(
      &GeneratePolynomialTrajectoryBehavior::modifyWaypointCallback,
      this, std::placeholders::_1));
}

void GeneratePolynomialTrajectoryBehavior::loadPlugin()
{
  try {
    getParameter("plugin_name", plugin_name_);

    plugin_loader_ = std::make_shared<pluginlib::ClassLoader<PluginBase>>(
      "as2_behaviors_trajectory_generation",
      "generate_polynomial_trajectory_behavior_plugin_base::"
      "GeneratePolynomialTrajectoryBase");

    const std::string class_name = plugin_name_ + "::Plugin";
    // Resolve and instantiate plugin class at runtime.
    plugin_ = plugin_loader_->createSharedInstance(class_name);
    plugin_->initialize(this, plugin_name_);

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory generator plugin loaded: %s",
      plugin_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to load plugin '%s': %s",
      plugin_name_.c_str(), ex.what());
    throw;
  }
}

void GeneratePolynomialTrajectoryBehavior::publishGenerationDebug()
{
  publishWaypoints();
  publishGeneratedTrajectory();
}

bool GeneratePolynomialTrajectoryBehavior::on_activate(
  std::shared_ptr<const Action::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on_activate");
  if (!has_vehicle_state_) {
    RCLCPP_ERROR(this->get_logger(), "No state received yet - rejecting goal");
    return false;
  }

  // Follow-reference mode requires exactly one waypoint and disables
  // path_length truncation (there is nothing to truncate).
  if (goal->follow_reference_mode && goal->path.size() != 1) {
    RCLCPP_ERROR(
      this->get_logger(),
      "follow_reference_mode requires exactly 1 waypoint, got %zu",
      goal->path.size());
    return false;
  }

  // Trace the goal path in its input frame, before TF conversion.
  logGoalWaypoints(goal->path, "on_activate");

  // Build mission waypoints in desired_frame_id_ (no synthetic prefix).
  std::vector<as2_msgs::msg::PoseStampedWithID> waypoints;
  if (!buildWaypoints(goal, waypoints)) {
    return false;
  }

  // Apply path_length truncation: feed only the first path_length_
  // waypoints to the plugin and queue the rest for incremental injection.
  // The plugin's updateWaypoints() decides at each window slide whether
  // to stitch smoothly or regenerate (fallback in the base class).
  // Skipped in follow_reference_mode (single waypoint).
  std::deque<as2_msgs::msg::PoseStampedWithID> pending;
  const auto goal_count = static_cast<int>(waypoints.size());
  if (!goal->follow_reference_mode && path_length_ > 0 &&
    goal_count > path_length_)
  {
    for (std::size_t i = static_cast<std::size_t>(path_length_);
      i < waypoints.size(); ++i)
    {
      pending.push_back(waypoints[i]);
    }
    waypoints.resize(static_cast<std::size_t>(path_length_));
  }

  // Degenerate-hold gate: latch the hold BEFORE invoking the plugin
  // when the goal is a single waypoint already within the distance
  // threshold. The plugin is skipped for the whole activation; on_run()
  // dispatches the static hold horizon (follow_reference_mode) or
  // reports SUCCESS (regular).
  if (!tryEnterDegenerateHold(waypoints) &&
    !generateTrajectory(waypoints, goal->max_speed))
  {
    return false;
  }

  // Persist the goal with canonicalized ids and frame-converted poses.
  // goal_.path holds the FULL mission queue (including pending) so the
  // feedback's remaining_waypoints count and on_resume rebuild stay correct.
  goal_ = *goal;
  goal_.path.assign(waypoints.begin(), waypoints.end());
  for (const auto & wp : pending) {
    goal_.path.emplace_back(wp);
  }
  pending_waypoints_queue_ = std::move(pending);
  active_waypoints_.assign(waypoints.begin(), waypoints.end());

  // Latch pause-after-generate flag and reset its sub-state.
  start_on_paused_ = goal->start_on_paused;
  has_paused_ = false;
  external_pause_ = false;

  // Flag follow_reference_mode
  follow_reference_mode_ = goal->follow_reference_mode;

  // Frame-drift watchdog.
  if (!degenerate_hold_ && frequency_update_frame_ > 0.0) {
    if (!timer_update_frame_) {
      timer_update_frame_ = this->create_wall_timer(
        std::chrono::milliseconds(
          static_cast<int>(1000.0 / frequency_update_frame_)),
        std::bind(
          &GeneratePolynomialTrajectoryBehavior::timerUpdateFrameCallback,
          this));
    }
    timer_update_frame_->reset();
    try {
      last_map_to_desired_ = tf_handler_.getTransform(
        map_frame_id_, desired_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not cache map->%s transform: %s",
        desired_frame_id_.c_str(), ex.what());
      timer_update_frame_->cancel();
    }
  }

  if (!degenerate_hold_) {
    publishGenerationDebug();
  }
  RCLCPP_INFO(
    this->get_logger(),
    "TrajectoryGenerator goal accepted%s",
    degenerate_hold_ ? " (degenerate-hold)" : "");
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::on_modify(
  std::shared_ptr<const Action::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on_modify");

  const std::string current_next =
    degenerate_hold_ ? degenerate_target_id_ : plugin_->getNextWaypointId();

  // Avoid desynchronization when finishing trajectory and a modify arrives
  if (current_next.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No pending waypoint reported by plugin - rejecting modify");
    return false;
  }

  // Trace the modify path in its input frame, before TF conversion.
  logGoalWaypoints(goal->path, "on_modify");

  // Convert each modify waypoint to desired_frame_id_ before merging.
  std::vector<as2_msgs::msg::PoseStampedWithID> modify_converted;
  if (!convertWaypointsToDesiredFrame(
      goal->path, modify_converted, "on_modify"))
  {
    return false;
  }

  // Already waypoints pose is modify
  // New waypoints are added to the end
  // TODO(RPS98): enable insert new waypoints in the already trajectory
  auto pending = mergeModifyIntoGoal(modify_converted, current_next);
  if (pending.empty()) {
    RCLCPP_ERROR(this->get_logger(), "on_modify: no pending waypoints left");
    return false;
  }

  goal_.yaw = goal->yaw;
  goal_.max_speed = goal->max_speed;
  goal_.stamp = goal->stamp;

  // Degenerate-hold transitions on modify: stay in hold if the merged
  // queue is still a single near waypoint, or exit hold and regenerate
  // via the plugin.
  if (degenerate_hold_) {
    active_waypoints_.assign(pending.begin(), pending.end());
    if (active_waypoints_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(), "on_modify: active waypoints empty after merge");
      return false;
    }
    if (tryEnterDegenerateHold(active_waypoints_)) {
      return true;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Degenerate-hold released by on_modify: regenerating trajectory.");
    degenerate_hold_ = false;
    degenerate_target_id_.clear();
    return generateTrajectory(active_waypoints_, goal_.max_speed);
  }

  return pushPendingToPlugin();
}

bool GeneratePolynomialTrajectoryBehavior::on_deactivate(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on_deactivate");
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::on_pause(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_WARN(
    this->get_logger(), "TrajectoryGenerator pause: capturing pending queue "
    "and switching to hover");
  if (timer_update_frame_) {
    timer_update_frame_->cancel();
  }
  // Snapshot the next pending id BEFORE resetting so on_resume can rebuild.
  // In degenerate-hold the plugin has no trajectory and would return "";
  // use the latched id so on_resume rebuilds the pending list correctly.
  paused_next_waypoint_id_ =
    degenerate_hold_ ? degenerate_target_id_ : plugin_->getNextWaypointId();
  if (!degenerate_hold_ && plugin_) {
    plugin_->reset();
  }
  // Clear the hold so a subsequent on_resume re-evaluates from scratch.
  degenerate_hold_ = false;
  degenerate_target_id_.clear();
  hover_motion_handler_.sendHover();
  // Mark this as an external pause so on_resume can tell it apart from the
  // auto-pause emitted by the start_on_paused flow.
  external_pause_ = true;
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::on_resume(
  const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on_resume");
  if (!has_vehicle_state_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot resume: no vehicle state");
    return false;
  }

  // Wake-up from the auto-pause emitted by start_on_paused: the trajectory
  // generated in on_activate is still loaded in the plugin and execution has
  // never started, so just clear the latch and let on_run drive the first
  // sample at trajectory_time_ == 0 (first_tick_after_anchor_ remains true).
  if (start_on_paused_ && !external_pause_) {
    if (!has_paused_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot resume: start_on_paused requested but on_run never reached "
        "the auto-PAUSED transition");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "TrajectoryGenerator: resuming from start_on_paused auto-pause");
    return true;
  }

  auto pending = getPendingWaypoints(paused_next_waypoint_id_);
  paused_next_waypoint_id_.clear();
  if (pending.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot resume: no pending waypoints");
    return false;
  }

  // Apply path_length truncation on resume so the partial-feed invariants
  // are restored.
  std::deque<as2_msgs::msg::PoseStampedWithID> resume_pending;
  const auto resumed_count = static_cast<int>(pending.size());
  if (path_length_ > 0 && resumed_count > path_length_) {
    for (std::size_t i = static_cast<std::size_t>(path_length_);
      i < pending.size(); ++i)
    {
      resume_pending.push_back(pending[i]);
    }
    pending.resize(static_cast<std::size_t>(path_length_));
  }

  // Degenerate-hold gate: resume directly into hold when the resumed
  // queue is a single waypoint within the distance threshold. Mirrors
  // on_activate.
  if (!tryEnterDegenerateHold(pending) &&
    !generateTrajectory(pending, goal_.max_speed))
  {
    return false;
  }

  // Refresh the stored queue with the resumed pending list.
  goal_.path = pending;
  for (const auto & wp : resume_pending) {
    goal_.path.emplace_back(wp);
  }
  goal_.stamp = this->now();
  pending_waypoints_queue_ = std::move(resume_pending);
  active_waypoints_.assign(pending.begin(), pending.end());

  if (!degenerate_hold_) {
    publishGenerationDebug();
  }
  return true;
}

as2_behavior::ExecutionStatus GeneratePolynomialTrajectoryBehavior::on_run(
  const std::shared_ptr<const Action::Goal> & /*goal*/,
  std::shared_ptr<Action::Feedback> & feedback_msg,
  std::shared_ptr<Action::Result> & result_msg)
{
  // Plugin readiness check. In degenerate-hold the plugin may have never
  // been invoked since on_activate, so the trajectory-generated subcheck
  // is gated on the hold flag.
  if (!plugin_ ||
    (!degenerate_hold_ && !plugin_->isTrajectoryGenerated()))
  {
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Pause-after-generate: if the goal requested start_on_paused, emit a
  // single PAUSED transition before any setpoint is sent. The wrapper stays
  // paused until an external on_resume() call switches the behavior back to
  // RUNNING.
  if (start_on_paused_ && !has_paused_) {
    has_paused_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "TrajectoryGenerator: start_on_paused requested, holding PAUSED until "
      "resume");
    return as2_behavior::ExecutionStatus::PAUSED;
  }

  // Degenerate-hold dispatch covers both the latched case (from
  // on_activate / on_resume / on_modify) and the lazy case (the live
  // target drifted within the threshold during normal tracking, e.g.
  // follow_reference re-publishing modify_waypoint).
  if (degenerate_hold_) {
    return runDegenerateHold(feedback_msg, result_msg);
  }
  if (tryEnterDegenerateHold(active_waypoints_)) {
    return runDegenerateHold(feedback_msg, result_msg);
  }

  // Advance the host trajectory time axis. The first tick after a
  // generation / resume keeps dt = 0 so the control sample lands at
  // trajectory_time_ exactly (which is 0 right after a fresh
  // generation, or the preserved value across stitched updates).
  const auto now = this->now();
  double dt = 0.0;
  if (first_tick_after_anchor_) {
    first_tick_after_anchor_ = false;
  } else {
    dt = (now - last_tick_time_).seconds();
  }
  last_tick_time_ = now;
  trajectory_time_ += dt;

  as2_msgs::msg::TrajectorySetpoints setpoints;
  if (!evaluateHorizon(
      trajectory_time_, sampling_dt_,
      static_cast<std::size_t>(sampling_n_), setpoints))
  {
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Asynchronous-regeneration plugins (e.g. dynamic_mav_trajectory_generator)
  // perform the deferred swap of an in-flight updateWaypoints() request inside
  // evaluateTrajectory(); the first sample of evaluateHorizon() above runs
  // with is_horizon_sample=false, which is exactly the call that triggers
  // that swap. consumeRegeneratedFlag() reports the transition once. Refresh
  // the debug path so RViz catches up with the new trajectory; synchronous
  // plugins keep the base default (always false) and this branch is a no-op.
  if (enable_debug_ && plugin_->consumeRegeneratedFlag()) {
    publishGenerationDebug();
  }

  if (!trajectory_motion_handler_.sendTrajectorySetpoints(setpoints)) {
    RCLCPP_ERROR(this->get_logger(), "Could not send trajectory setpoints");
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // Feedback: next id from plugin, remaining count from goal_.path.
  const std::string next_id = plugin_->getNextWaypointId();
  feedback_msg->next_waypoint_id = next_id;
  feedback_msg->remaining_waypoints = remainingWaypointCount(next_id);
  feedback_ = *feedback_msg;

  // path_length partial-trajectory feed: extend the active window via
  // plugin_->updateWaypoints() when the plugin-side remaining drops below
  // path_length_. Skipped when the queue is empty or the plugin reports no
  // pending waypoint yet (next_id empty). The plugin re-anchors its
  // internal offset against trajectory_time_ on regeneration; the host
  // axis keeps progressing without realignment.
  if (path_length_ > 0 && !pending_waypoints_queue_.empty() &&
    !next_id.empty())
  {
    auto next_it = std::find_if(
      active_waypoints_.begin(), active_waypoints_.end(),
      [&next_id](const as2_msgs::msg::PoseStampedWithID & wp) {
        return wp.id == next_id;
      });
    if (next_it != active_waypoints_.end()) {
      active_waypoints_.erase(active_waypoints_.begin(), next_it);
    }
    if (active_waypoints_.size() <
      static_cast<std::size_t>(path_length_))
    {
      const auto wp = pending_waypoints_queue_.front();
      active_waypoints_.push_back(wp);
      const auto compute_t0 = this->now();
      if (plugin_->updateWaypoints(
          active_waypoints_, goal_.max_speed, trajectory_time_))
      {
        const double compute_s = (this->now() - compute_t0).seconds();
        RCLCPP_INFO(
          this->get_logger(),
          "Trajectory updated (path_length feed): %zu waypoints, "
          "compute=%.4f s, duration=%.2f s",
          active_waypoints_.size(), compute_s, plugin_->getDuration());
        publishGenerationTime(compute_s);
        pending_waypoints_queue_.pop_front();
        if (enable_debug_) {
          publishGenerationDebug();
        }
      } else {
        // Roll back the speculative append and retry next cycle.
        active_waypoints_.pop_back();
        RCLCPP_WARN(
          this->get_logger(),
          "path_length: updateWaypoints failed, retrying next cycle "
          "(%zu pending)",
          pending_waypoints_queue_.size());
      }
    }
  }

  if (plugin_->isFinished(trajectory_time_)) {
    // In follow_reference_mode the behavior never terminates by itself
    if (!follow_reference_mode_) {
      if (!pending_waypoints_queue_.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Plugin reports trajectory done while %zu waypoints are still "
          "pending — partial mission only.",
          pending_waypoints_queue_.size());
      }
      result_msg->trajectory_generator_success = true;
      return as2_behavior::ExecutionStatus::SUCCESS;
    }
  }

  return as2_behavior::ExecutionStatus::RUNNING;
}

void GeneratePolynomialTrajectoryBehavior::on_execution_end(
  const as2_behavior::ExecutionStatus & state)
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator on_execution_end");
  if (timer_update_frame_) {
    timer_update_frame_.reset();
  }
  if (plugin_) {
    plugin_->reset();
  }
  resetRuntimeState();

  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    return;
  }
  // Keep vehicle safe if execution ended unexpectedly.
  if (state != as2_behavior::ExecutionStatus::SUCCESS) {
    hover_motion_handler_.sendHover();
  }
}

bool GeneratePolynomialTrajectoryBehavior::validateWaypoints(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints) const
{
  if (waypoints.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "At least 1 waypoint is required, got %zu",
      waypoints.size());
    return false;
  }

  std::unordered_set<std::string> ids;
  ids.reserve(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    if (waypoints[i].id.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Waypoint #%zu has empty id", i);
      return false;
    }
    if (!ids.insert(waypoints[i].id).second) {
      RCLCPP_ERROR(
        this->get_logger(), "Duplicate waypoint id '%s'",
        waypoints[i].id.c_str());
      return false;
    }
  }
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::convertWaypointsToDesiredFrame(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & in,
  std::vector<as2_msgs::msg::PoseStampedWithID> & out,
  const char * log_context)
{
  out.clear();
  out.reserve(in.size());

  for (std::size_t i = 0; i < in.size(); ++i) {
    const auto & wp = in[i];
    as2_msgs::msg::PoseStampedWithID converted;
    if (wp.id.empty()) {
      // Use id = waypoint_{index} for waypoints with empty id
      // index has 3 digits
      char buf[32];
      std::snprintf(buf, sizeof(buf), "waypoint_%03zu", i);
      converted.id = buf;
    } else {
      converted.id = wp.id;
    }

    try {
      converted.pose = tf_handler_.convert(wp.pose, desired_frame_id_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "%s: could not convert waypoint #%zu (id '%s'): %s",
        log_context, i, wp.id.c_str(), ex.what());
      return false;
    }

    out.emplace_back(std::move(converted));
  }

  return true;
}

void GeneratePolynomialTrajectoryBehavior::logGoalWaypoints(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
  const char * log_context) const
{
  RCLCPP_INFO(
    this->get_logger(),
    "%s: received %zu waypoint(s) in goal frame:",
    log_context, waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & wp = waypoints[i];
    const double yaw =
      as2::frame::getYawFromQuaternion(wp.pose.pose.orientation);
    RCLCPP_INFO(
      this->get_logger(),
      "  [%zu] id='%s' frame='%s' pos=(%.3f, %.3f, %.3f) yaw=%.3f rad",
      i,
      wp.id.c_str(),
      wp.pose.header.frame_id.c_str(),
      wp.pose.pose.position.x,
      wp.pose.pose.position.y,
      wp.pose.pose.position.z,
      yaw);
  }
}

bool GeneratePolynomialTrajectoryBehavior::buildWaypoints(
  const std::shared_ptr<const Action::Goal> & goal,
  std::vector<as2_msgs::msg::PoseStampedWithID> & out)
{
  if (goal->path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Goal path is empty");
    return false;
  }
  if (goal->max_speed <= 0.0f) {
    RCLCPP_ERROR(
      this->get_logger(), "Goal max_speed must be > 0, got %f",
      goal->max_speed);
    return false;
  }

  if (!convertWaypointsToDesiredFrame(goal->path, out, "buildWaypoints")) {
    return false;
  }

  return validateWaypoints(out);
}

bool GeneratePolynomialTrajectoryBehavior::generateTrajectory(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
  double max_speed)
{
  if (max_speed <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "max_speed must be > 0, got %f", max_speed);
    return false;
  }
  if (waypoints.empty()) {
    RCLCPP_ERROR(this->get_logger(), "generateTrajectory: empty waypoints");
    return false;
  }

  // Plugin already holds the latest vehicle state pushed by stateCallback;
  // it reads it directly as the boundary condition for the new trajectory.
  // The host's trajectory_time_ is reset to 0 here, so the plugin anchors
  // its internal offset such that t_trajectory == 0 maps to the start of
  // the new backend trajectory.
  trajectory_time_ = 0.0;
  const auto compute_t0 = this->now();
  if (!plugin_->generateTrajectory(waypoints, max_speed, trajectory_time_)) {
    RCLCPP_ERROR(this->get_logger(), "Plugin generateTrajectory failed");
    return false;
  }
  const double compute_s = (this->now() - compute_t0).seconds();
  RCLCPP_INFO(
    this->get_logger(),
    "Trajectory generated: %zu waypoints, compute=%.4f s, duration=%.2f s",
    waypoints.size(), compute_s, plugin_->getDuration());
  publishGenerationTime(compute_s);

  // Fresh trajectory: reset host time-axis bookkeeping. The first on_run
  // tick uses dt = 0 so the control sample lands at trajectory_time_ == 0
  // exactly, and yaw rate-limit machinery is re-anchored to the new start.
  first_tick_after_anchor_ = true;
  last_tick_time_ = this->now();
  time_zero_yaw_ = this->now();
  // Cache the initial yaw angle from the current pose in desired_frame_id
  init_yaw_angle_ =
    as2::frame::getYawFromQuaternion(plugin_->getVehiclePose().pose.orientation);
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::evaluatePoint(
  double t, as2_msgs::msg::TrajectoryPoint & out, bool is_horizon_sample)
{
  if (!plugin_ || !plugin_->isTrajectoryGenerated()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "evaluatePoint: trajectory not generated yet");
    return false;
  }
  if (!has_vehicle_state_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "evaluatePoint: vehicle pose not available");
    return false;
  }

  // The plugin maps t to its own backend axis and clamps internally.
  if (!plugin_->evaluate(t, out, is_horizon_sample)) {
    return false;
  }

  out.yaw_angle = static_cast<float>(computeYaw(out));
  return true;
}

bool GeneratePolynomialTrajectoryBehavior::evaluateHorizon(
  double t0, double dt, std::size_t n,
  as2_msgs::msg::TrajectorySetpoints & out)
{
  if (n == 0 || dt <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "evaluateHorizon: invalid arguments (n=%zu, dt=%f)", n, dt);
    return false;
  }

  out.header.frame_id = desired_frame_id_;
  out.header.stamp = this->now();
  out.setpoints.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    // First sample may use current-state correction; the rest are horizon-only.
    const bool is_horizon_sample = (i > 0);
    if (!evaluatePoint(
        t0 + static_cast<double>(i) * dt, out.setpoints[i],
        is_horizon_sample))
    {
      return false;
    }
  }

  if (enable_debug_ && !out.setpoints.empty()) {
    publishEvaluatedPoint(out.setpoints.front(), false);
    if (out.setpoints.size() >= 2) {
      publishEvaluatedPoint(out.setpoints.back(), true);
    }
  }
  return true;
}

double GeneratePolynomialTrajectoryBehavior::computeYaw(
  const as2_msgs::msg::TrajectoryPoint & point)
{
  const double current_yaw =
    as2::frame::getYawFromQuaternion(
    plugin_->getVehiclePose().pose.orientation);

  switch (goal_.yaw.mode) {
    // Centralized yaw policy dispatch based on the active goal's yaw mode.
    case as2_msgs::msg::YawMode::KEEP_YAW:
      return init_yaw_angle_;
    case as2_msgs::msg::YawMode::PATH_FACING:
      return computeYawAnglePathFacing(point.twist.x, point.twist.y);
    case as2_msgs::msg::YawMode::FIXED_YAW:
      return goal_.yaw.angle;
    case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
      if (has_yaw_from_topic_) {
        return yaw_from_topic_;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Yaw from topic not received yet, using initial yaw angle");
      return init_yaw_angle_;
    case as2_msgs::msg::YawMode::FACE_REFERENCE:
      return computeYawFaceReference();
    default:
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unknown yaw mode %u, using current yaw", goal_.yaw.mode);
      return current_yaw;
  }
}

double GeneratePolynomialTrajectoryBehavior::computeYawAnglePathFacing(
  double vx, double vy) const
{
  if (std::sqrt(vx * vx + vy * vy) > yaw_threshold_) {
    return as2::frame::getVector2DAngle(vx, vy);
  }
  return as2::frame::getYawFromQuaternion(
    plugin_->getVehiclePose().pose.orientation);
}

const as2_msgs::msg::PoseStampedWithID *
GeneratePolynomialTrajectoryBehavior::getNextReferenceWaypoint() const
{
  if (goal_.path.empty() || !plugin_) {
    return nullptr;
  }

  const std::string next_id = plugin_->getNextWaypointId();
  if (next_id.empty()) {
    return nullptr;
  }

  const auto cursor = std::find_if(
    goal_.path.begin(), goal_.path.end(),
    [&next_id](const as2_msgs::msg::PoseStampedWithID & wp) {
      return wp.id == next_id;
    });
  if (cursor == goal_.path.end()) {
    return nullptr;
  }

  // Skip waypoints whose id contains the "ny" substring
  const auto it = std::find_if(
    cursor, goal_.path.end(),
    [](const as2_msgs::msg::PoseStampedWithID & wp) {
      return wp.id.find("ny") == std::string::npos;
    });
  return (it != goal_.path.end()) ? &(*it) : nullptr;
}

double GeneratePolynomialTrajectoryBehavior::computeYawFaceReference()
{
  const auto & vehicle_pose = plugin_->getVehiclePose();
  const double current_yaw =
    as2::frame::getYawFromQuaternion(vehicle_pose.pose.orientation);
  const auto * next_waypoint = getNextReferenceWaypoint();
  if (next_waypoint == nullptr) {
    time_zero_yaw_ = this->now();
    return current_yaw;
  }

  const Eigen::Vector2d diff(
    next_waypoint->pose.pose.position.x - vehicle_pose.pose.position.x,
    next_waypoint->pose.pose.position.y - vehicle_pose.pose.position.y);

  if (diff.norm() <= yaw_threshold_) {
    time_zero_yaw_ = this->now();
    return current_yaw;
  }

  const double target_yaw = as2::frame::getVector2DAngle(diff.x(), diff.y());
  const double yaw_error = as2::frame::angleMinError(target_yaw, current_yaw);
  const rclcpp::Duration dt = this->now() - time_zero_yaw_;
  const double dt_seconds = std::max(dt.seconds(), 1e-3);
  // Convert angle error to bounded yaw-rate command. A non-positive
  // yaw_speed_threshold disables the rate limit (raw error tracked).
  double yaw_speed = yaw_error / dt_seconds;
  if (yaw_speed_threshold_ > 0.0) {
    yaw_speed =
      std::clamp(yaw_speed, -yaw_speed_threshold_, yaw_speed_threshold_);
  }
  time_zero_yaw_ = this->now();
  return current_yaw + yaw_speed * dt_seconds;
}

std::vector<as2_msgs::msg::PoseStampedWithID>
GeneratePolynomialTrajectoryBehavior::getPendingWaypoints(
  const std::string & next_id) const
{
  if (next_id.empty() || goal_.path.empty()) {
    return {goal_.path.begin(), goal_.path.end()};
  }
  const auto it = std::find_if(
    goal_.path.begin(), goal_.path.end(),
    [&next_id](const as2_msgs::msg::PoseStampedWithID & wp) {
      return wp.id == next_id;
    });
  if (it == goal_.path.end()) {
    return {goal_.path.begin(), goal_.path.end()};
  }
  return {it, goal_.path.end()};
}

uint16_t GeneratePolynomialTrajectoryBehavior::remainingWaypointCount(
  const std::string & next_id) const
{
  if (goal_.path.empty()) {
    return 0;
  }
  if (next_id.empty()) {
    return static_cast<uint16_t>(goal_.path.size());
  }
  const auto it = std::find_if(
    goal_.path.begin(), goal_.path.end(),
    [&next_id](const as2_msgs::msg::PoseStampedWithID & wp) {
      return wp.id == next_id;
    });
  if (it == goal_.path.end()) {
    return 0;
  }
  return static_cast<uint16_t>(std::distance(it, goal_.path.end()));
}

std::vector<as2_msgs::msg::PoseStampedWithID>
GeneratePolynomialTrajectoryBehavior::mergeModifyIntoGoal(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & modify_waypoints,
  const std::string & next_id)
{
  // Drop already-passed waypoints.
  if (!next_id.empty() && !goal_.path.empty()) {
    const auto it = std::find_if(
      goal_.path.begin(), goal_.path.end(),
      [&next_id](const as2_msgs::msg::PoseStampedWithID & wp) {
        return wp.id == next_id;
      });
    if (it != goal_.path.end()) {
      goal_.path.erase(goal_.path.begin(), it);
    }
  }

  // Update existing entries / append new ones at the end.
  for (const auto & mod_wp : modify_waypoints) {
    auto it = std::find_if(
      goal_.path.begin(), goal_.path.end(),
      [&mod_wp](const as2_msgs::msg::PoseStampedWithID & wp) {
        return wp.id == mod_wp.id;
      });
    if (it != goal_.path.end()) {
      it->pose = mod_wp.pose;
    } else {
      goal_.path.emplace_back(mod_wp);
    }
  }

  return {goal_.path.begin(), goal_.path.end()};
}

bool GeneratePolynomialTrajectoryBehavior::pushPendingToPlugin()
{
  if (!plugin_) {
    return false;
  }

  const std::string next_id = plugin_->getNextWaypointId();
  auto pending = getPendingWaypoints(next_id);

  // path_length: split the pending list into the active window we feed to
  // the plugin and a queue we drain incrementally in on_run.
  std::deque<as2_msgs::msg::PoseStampedWithID> queued;
  splitForPathLength(pending, queued);

  // If no pending waypoints, no waypoints to push
  if (pending.empty()) {
    return false;
  }

  // Delegate to updateWaypoints. The plugin reads the live vehicle
  // state from its protected members (refreshed continuously by
  // stateCallback) and decides between smooth stitching (preserving its
  // internal offset) or regeneration (re-anchoring its offset against
  // trajectory_time_). The host axis is unaffected either way.
  const auto compute_t0 = this->now();
  if (!plugin_->updateWaypoints(pending, goal_.max_speed, trajectory_time_)) {
    RCLCPP_WARN(this->get_logger(), "updateWaypoints failed");
    return false;
  }
  const double compute_s = (this->now() - compute_t0).seconds();
  RCLCPP_INFO(
    this->get_logger(),
    "Trajectory updated (push pending): %zu waypoints, compute=%.4f s, "
    "duration=%.2f s",
    pending.size(), compute_s, plugin_->getDuration());
  publishGenerationTime(compute_s);
  active_waypoints_ = pending;
  pending_waypoints_queue_ = std::move(queued);
  publishGenerationDebug();
  return true;
}

void GeneratePolynomialTrajectoryBehavior::splitForPathLength(
  std::vector<as2_msgs::msg::PoseStampedWithID> & active,
  std::deque<as2_msgs::msg::PoseStampedWithID> & queued) const
{
  if (path_length_ <= 0) {
    return;
  }
  if (static_cast<int>(active.size()) <= path_length_) {
    return;
  }
  for (std::size_t i = static_cast<std::size_t>(path_length_);
    i < active.size(); ++i)
  {
    queued.push_back(active[i]);
  }
  active.resize(static_cast<std::size_t>(path_length_));
}

void GeneratePolynomialTrajectoryBehavior::resetRuntimeState()
{
  goal_ = Action::Goal{};
  feedback_ = Action::Feedback{};
  paused_next_waypoint_id_.clear();
  has_yaw_from_topic_ = false;
  trajectory_time_ = 0.0;
  first_tick_after_anchor_ = true;
  init_yaw_angle_ = 0.0;
  start_on_paused_ = false;
  has_paused_ = false;
  external_pause_ = false;
  follow_reference_mode_ = false;
  degenerate_hold_ = false;
  degenerate_target_id_.clear();
  pending_waypoints_queue_.clear();
  active_waypoints_.clear();
}

bool GeneratePolynomialTrajectoryBehavior::isDegenerateTarget(
  const as2_msgs::msg::PoseStampedWithID & last_wp) const
{
  const auto & pos = plugin_->getVehiclePose().pose.position;
  const double dx = last_wp.pose.pose.position.x - pos.x;
  const double dy = last_wp.pose.pose.position.y - pos.y;
  const double dz = last_wp.pose.pose.position.z - pos.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <
         generate_polynomial_trajectory_behavior_plugin_base::
         kDegenerateDistanceM;
}

bool GeneratePolynomialTrajectoryBehavior::tryEnterDegenerateHold(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints)
{
  // Single-waypoint guard: a multi-waypoint mission has intermediate
  // motion even if its last waypoint is near the vehicle, so the plugin
  // must still plan it. The hold is meaningful only for the single-target
  // case (typical of follow_reference and go_to-to-near).
  if (waypoints.size() != 1) {
    return false;
  }
  if (!isDegenerateTarget(waypoints.front())) {
    return false;
  }
  enterDegenerateHold(waypoints.front());
  return true;
}

void GeneratePolynomialTrajectoryBehavior::enterDegenerateHold(
  const as2_msgs::msg::PoseStampedWithID & last_wp)
{
  degenerate_hold_ = true;
  degenerate_target_ = last_wp.pose.pose.position;
  degenerate_target_id_ = last_wp.id;
  // generateTrajectory() (which normally anchors init_yaw_angle_) is
  // skipped while in hold; anchor it here so buildHoldHorizon() uses
  // the live vehicle yaw.
  init_yaw_angle_ = as2::frame::getYawFromQuaternion(
    plugin_->getVehiclePose().pose.orientation);
  RCLCPP_WARN(
    this->get_logger(),
    "Target within %.3f m of vehicle: degenerate-hold engaged on '%s'.",
    generate_polynomial_trajectory_behavior_plugin_base::kDegenerateDistanceM,
    last_wp.id.c_str());
}

void GeneratePolynomialTrajectoryBehavior::buildHoldHorizon(
  const geometry_msgs::msg::Point & target,
  as2_msgs::msg::TrajectorySetpoints & out) const
{
  out.header.frame_id = desired_frame_id_;
  out.header.stamp = this->now();
  out.setpoints.assign(
    static_cast<std::size_t>(sampling_n_), as2_msgs::msg::TrajectoryPoint{});
  for (auto & sp : out.setpoints) {
    sp.position.x = target.x;
    sp.position.y = target.y;
    sp.position.z = target.z;
    sp.twist.x = 0.0;
    sp.twist.y = 0.0;
    sp.twist.z = 0.0;
    sp.acceleration.x = 0.0;
    sp.acceleration.y = 0.0;
    sp.acceleration.z = 0.0;
    sp.yaw_angle = static_cast<float>(init_yaw_angle_);
  }
}

as2_behavior::ExecutionStatus
GeneratePolynomialTrajectoryBehavior::runDegenerateHold(
  std::shared_ptr<Action::Feedback> & feedback_msg,
  std::shared_ptr<Action::Result> & result_msg)
{
  // Outside follow_reference_mode the host should not stay running on a
  // static target: report SUCCESS the first tick the hold is observed.
  if (!follow_reference_mode_) {
    result_msg->trajectory_generator_success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // Leave the hold if the live waypoint moved outside the threshold:
  // regenerate the trajectory and let the next tick evaluate normally.
  if (!active_waypoints_.empty() &&
    !isDegenerateTarget(active_waypoints_.back()))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Degenerate-hold released: regenerating trajectory.");
    degenerate_hold_ = false;
    degenerate_target_id_.clear();
    if (!generateTrajectory(active_waypoints_, goal_.max_speed)) {
      result_msg->trajectory_generator_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }
    feedback_msg->next_waypoint_id = plugin_->getNextWaypointId();
    feedback_msg->remaining_waypoints =
      remainingWaypointCount(feedback_msg->next_waypoint_id);
    feedback_ = *feedback_msg;
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // Stay in hold: publish a static horizon at the latched target.
  as2_msgs::msg::TrajectorySetpoints setpoints;
  buildHoldHorizon(degenerate_target_, setpoints);
  if (!trajectory_motion_handler_.sendTrajectorySetpoints(setpoints)) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not send degenerate-hold setpoints");
    result_msg->trajectory_generator_success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }
  feedback_msg->next_waypoint_id = degenerate_target_id_;
  feedback_msg->remaining_waypoints =
    remainingWaypointCount(degenerate_target_id_);
  feedback_ = *feedback_msg;
  return as2_behavior::ExecutionStatus::RUNNING;
}

bool GeneratePolynomialTrajectoryBehavior::computeFrameError()
{
  // A non-positive transform_threshold disables the drift check.
  if (transform_threshold_ <= 0.0) {
    return false;
  }
  if (last_map_to_desired_.header.frame_id.empty()) {
    return false;
  }

  geometry_msgs::msg::TransformStamped current;
  try {
    current = tf_handler_.getTransform(
      map_frame_id_, desired_frame_id_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "computeFrameError: could not get transform: %s", ex.what());
    return false;
  }

  const Eigen::Vector3d current_t(current.transform.translation.x,
    current.transform.translation.y,
    current.transform.translation.z);
  const Eigen::Vector3d last_t(last_map_to_desired_.transform.translation.x,
    last_map_to_desired_.transform.translation.y,
    last_map_to_desired_.transform.translation.z);
  // Trigger regeneration when map-to-desired translation drifts too far.
  return (current_t - last_t).norm() > transform_threshold_;
}

void GeneratePolynomialTrajectoryBehavior::timerUpdateFrameCallback()
{
  if (!computeFrameError()) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Map<->odom drift exceeded %.2fm threshold, regenerating trajectory",
    transform_threshold_);

  // Re-convert cached goal poses under the new frame state.
  auto goal_ptr = std::make_shared<const Action::Goal>(goal_);
  std::vector<as2_msgs::msg::PoseStampedWithID> refreshed;
  if (!buildWaypoints(goal_ptr, refreshed)) {
    return;
  }
  goal_.path = refreshed;

  pushPendingToPlugin();

  // Refresh transform cache regardless of the push outcome.
  try {
    last_map_to_desired_ = tf_handler_.getTransform(
      map_frame_id_, desired_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not refresh transform cache: %s",
      ex.what());
  }
}

void GeneratePolynomialTrajectoryBehavior::stateCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  try {
    auto [vehicle_pose, vehicle_twist] =
      tf_handler_.getState(
      *twist_msg,
      desired_frame_id_,
      desired_frame_id_,
      base_link_frame_id_);
    // Push the freshly-resolved state directly into the plugin so it
    // becomes the single source of truth for everything downstream
    // (plugin entry points and wrapper readers via getVehiclePose /
    // getVehicleTwist accessors).
    if (plugin_) {
      plugin_->setVehicleState(vehicle_pose, vehicle_twist);
    }
    if (!has_vehicle_state_) {
      RCLCPP_INFO(this->get_logger(), "State callback active");
      has_vehicle_state_ = true;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "stateCallback: could not get transform: %s",
      ex.what());
  }
}

void GeneratePolynomialTrajectoryBehavior::yawCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  has_yaw_from_topic_ = true;
  yaw_from_topic_ = msg->data;
}

void GeneratePolynomialTrajectoryBehavior::modifyWaypointCallback(
  const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg)
{
  if (!plugin_) {
    return;
  }

  // Apply each new pose onto goal_.path (with frame conversion).
  for (const auto & wp : msg->poses) {
    geometry_msgs::msg::PoseStamped pose_stamped = wp.pose;
    if (pose_stamped.header.frame_id != desired_frame_id_) {
      try {
        pose_stamped = tf_handler_.convert(pose_stamped, desired_frame_id_);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "modifyWaypointCallback: could not convert frame for '%s': %s",
          wp.id.c_str(), ex.what());
        continue;
      }
    }
    for (auto & gp : goal_.path) {
      if (gp.id == wp.id) {
        gp.pose = pose_stamped;
        break;
      }
    }
  }

  // Delegate to the plugin via updateWaypoints. The plugin decides
  // whether to stitch smoothly (preserving its internal offset) or
  // regenerate (re-anchoring its offset against trajectory_time_).
  pushPendingToPlugin();
}

void GeneratePolynomialTrajectoryBehavior::initDebugPublishers()
{
  std::string path_topic;
  std::string ref_setpoint;
  std::string ref_end_waypoint;
  std::string ref_waypoints;
  std::string generation_time_topic;

  getParameter("debug.path_topic", path_topic);
  getParameter("debug.reference_setpoint", ref_setpoint);
  getParameter("debug.reference_end_waypoint", ref_end_waypoint);
  getParameter("debug.reference_waypoints", ref_waypoints);
  getParameter("debug.generation_time_topic", generation_time_topic);

  if (!path_topic.empty()) {
    debug_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);
  }
  if (!ref_setpoint.empty()) {
    debug_ref_point_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
      ref_setpoint,
      1);
  }
  if (!ref_end_waypoint.empty()) {
    debug_end_ref_point_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
      ref_end_waypoint, 1);
  }
  if (!ref_waypoints.empty()) {
    debug_waypoints_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      ref_waypoints, 1);
  }
  if (!generation_time_topic.empty()) {
    debug_generation_time_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(
      generation_time_topic, 1);
  }

  enable_debug_ = debug_path_pub_ || debug_ref_point_pub_ ||
    debug_end_ref_point_pub_ || debug_waypoints_pub_ || debug_generation_time_pub_;
}

void GeneratePolynomialTrajectoryBehavior::publishGenerationTime(
  double seconds)
{
  if (!debug_generation_time_pub_) {
    return;
  }
  std_msgs::msg::Float64 msg;
  msg.data = seconds;
  debug_generation_time_pub_->publish(msg);
}

void GeneratePolynomialTrajectoryBehavior::publishGeneratedTrajectory()
{
  if (!debug_path_pub_ || !plugin_ || !plugin_->isTrajectoryGenerated()) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = desired_frame_id_;
  path_msg.header.stamp = this->now();

  // Fixed decimation step and safety cap for the debug trajectory dump.
  // The host axis runs from trajectory_time_ (current) until the plugin
  // reports isFinished(); the cap keeps an unbounded backend trajectory
  // from spinning forever.
  constexpr double kStep = 0.2;
  constexpr double kMaxDuration = 600.0;

  const double t_start = trajectory_time_;
  for (double t = t_start;
    !plugin_->isFinished(t) && (t - t_start) <= kMaxDuration;
    t += kStep)
  {
    as2_msgs::msg::TrajectoryPoint setpoint;
    if (!evaluatePoint(t, setpoint, true)) {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = setpoint.position.x;
    pose.pose.position.y = setpoint.position.y;
    pose.pose.position.z = setpoint.position.z;
    path_msg.poses.emplace_back(std::move(pose));
  }

  debug_path_pub_->publish(path_msg);
}

void GeneratePolynomialTrajectoryBehavior::publishWaypoints()
{
  if (!debug_waypoints_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray msg;
  const auto stamp = this->now();

  // Clear any markers left over from a previous publish so that, when the
  // pending list shrinks (e.g. after on_modify drops the already-consumed
  // waypoints from goal_.path), the stale spheres with high ids don't
  // remain visible in RViz forever.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = desired_frame_id_;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.emplace_back(clear);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = desired_frame_id_;
  marker.header.stamp = stamp;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.r = 1.0f;
  marker.color.a = 1.0f;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

  int id = 0;
  for (const auto & wp : goal_.path) {
    marker.id = id++;
    marker.pose.position = wp.pose.pose.position;
    msg.markers.emplace_back(marker);
  }
  debug_waypoints_pub_->publish(msg);
}

void GeneratePolynomialTrajectoryBehavior::publishEvaluatedPoint(
  const as2_msgs::msg::TrajectoryPoint & point, bool is_last_in_horizon)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = desired_frame_id_;
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0f;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  marker.pose.position.x = point.position.x;
  marker.pose.position.y = point.position.y;
  marker.pose.position.z = point.position.z;

  if (!is_last_in_horizon && debug_ref_point_pub_) {
    marker.color.b = 1.0f;
    debug_ref_point_pub_->publish(marker);
  } else if (is_last_in_horizon && debug_end_ref_point_pub_) {
    marker.color.g = 1.0f;
    debug_end_ref_point_pub_->publish(marker);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GeneratePolynomialTrajectoryBehavior)
