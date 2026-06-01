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
 * @file generate_polynomial_trajectory_behavior.hpp
 *
 * @brief Pluginlib-based BehaviorServer wrapper for polynomial trajectory
 * generators.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_
#define GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_motion_reference_handlers/trajectory_motion.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

/**
 * @class GeneratePolynomialTrajectoryBehavior
 *
 * ROS2 BehaviorServer that wraps polynomial trajectory generator plugins. It
 * receives a path of waypoints and other parameters as a goal, and then uses the
 * specified plugin to generate a trajectory.
 */
class GeneratePolynomialTrajectoryBehavior
  : public as2_behavior::BehaviorServer<
    as2_msgs::action::GeneratePolynomialTrajectory>
{
public:
  using Action = as2_msgs::action::GeneratePolynomialTrajectory;
  using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
    GeneratePolynomialTrajectoryBase;

  /**
   * @brief Construct behavior server node.
   *
   * @param options ROS node options.
   */
  explicit GeneratePolynomialTrajectoryBehavior(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Virtual destructor.
   */
  ~GeneratePolynomialTrajectoryBehavior() override = default;

private:
  // BehaviorServer overrides.

  /**
   * @brief Activate behavior with a new goal.
   *
   * @param goal Requested behavior goal.
   * @return true when goal activation succeeds.
   */
  bool on_activate(std::shared_ptr<const Action::Goal> goal) override;

  /**
   * @brief Modify currently active goal.
   *
   * @param goal New goal request.
   * @return true when goal modification succeeds.
   */
  bool on_modify(std::shared_ptr<const Action::Goal> goal) override;

  /**
   * @brief Deactivate behavior.
   *
   * @param message Optional deactivation message.
   * @return true when deactivation succeeds.
   */
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  /**
   * @brief Pause behavior and hand over to hover.
   *
   * @param message Optional pause message.
   * @return true when pause handling succeeds.
   */
  bool on_pause(const std::shared_ptr<std::string> & message) override;

  /**
   * @brief Resume behavior from stored progress.
   *
   * @param message Optional resume message.
   * @return true when resume activation succeeds.
   */
  bool on_resume(const std::shared_ptr<std::string> & message) override;

  /**
   * @brief Execute behavior run step.
   *
   * @param goal Active goal.
   * @param feedback_msg Output feedback message.
   * @param result_msg Output result message.
   * @return Current execution status.
   */
  as2_behavior::ExecutionStatus
  on_run(
    const std::shared_ptr<const Action::Goal> & goal,
    std::shared_ptr<Action::Feedback> & feedback_msg,
    std::shared_ptr<Action::Result> & result_msg) override;

  /**
   * @brief Finalize execution and cleanup runtime state.
   *
   * @param state Final behavior execution state.
   */
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

  // Utils and internal methods.

  /**
   * @brief Read parameter through shared helper.
   *
   * @tparam T Parameter type.
   * @param param_name Parameter name.
   * @param param_value [in] default value when @p use_default is true,
   *                    [out] read value.
   * @param use_default Whether to use @p param_value as default.
   */
  template<typename T>
  inline void getParameter(
    const std::string & param_name, T & param_value,
    bool use_default = false)
  {
    generate_polynomial_trajectory_behavior_plugin_base::getParameter(
      this, param_name, param_value, use_default);
  }

  /**
   * @brief Load trajectory generation plugin.
   */
  void loadPlugin();

  /**
   * @brief Build internal waypoint list from goal.
   *
   * The output contains only the user-specified mission waypoints
   * converted to desired_frame_id_ with normalized ids. The synthetic
   * "current" entry is no longer prepended here: each plugin reads the
   * vehicle state from the protected base members and injects it into
   * its backend if needed.
   *
   * @param goal Input goal.
   * @param out Output waypoint vector.
   * @return true when waypoint list is valid and converted.
   */
  bool buildWaypoints(
    const std::shared_ptr<const Action::Goal> & goal,
    std::vector<as2_msgs::msg::PoseStampedWithID> & out);

  /**
   * @brief Validate waypoint constraints.
   *
   * @param waypoints Waypoint list to validate.
   * @return true when waypoint list is valid.
   */
  bool validateWaypoints(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints) const;

  /**
   * @brief Convert waypoint poses to desired_frame_id_ and normalize ids.
   *
   * Empty ids are auto-generated as waypoint_XXX using the waypoint index.
   *
   * @param in Input waypoint list.
   * @param out Output converted waypoint list.
   * @param log_context Context string for error logs.
   * @return true when all waypoints are converted.
   */
  bool convertWaypointsToDesiredFrame(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & in,
    std::vector<as2_msgs::msg::PoseStampedWithID> & out,
    const char * log_context);

  /**
   * @brief Log goal waypoints (id, position, yaw) in their input frame.
   *
   * Emitted before any TF conversion, so the trace reflects exactly what the
   * client sent in goal->path. Yaw is extracted from each pose's orientation
   * (radians).
   *
   * @param waypoints Goal waypoint list (as received).
   * @param log_context Tag prepended to each line (e.g. "on_activate").
   */
  void logGoalWaypoints(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    const char * log_context) const;

  /**
   * @brief Generate a fresh trajectory through the plugin and reset time
   *        bases.
   *
   * Refreshes the plugin vehicle state (pose + twist) before delegating
   * to plugin_->generateTrajectory(). The plugin reads the live state
   * from the protected base members and is responsible for injecting it
   * into its backend, so the waypoint list passed here contains only
   * the user-specified mission waypoints (no synthetic "current" entry).
   *
   * @param waypoints Mission waypoints in desired_frame_id_.
   * @param max_speed Maximum speed in m/s.
   * @return true when generation succeeds.
   */
  bool generateTrajectory(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed);

  /**
   * @brief Get pending waypoints from goal_.path starting at next_id.
   *
   * @param next_id Id of the next pending waypoint.
   * @return Subset of goal_.path from next_id (inclusive) to the end.
   *         If next_id is empty or not found, returns the full path.
   */
  std::vector<as2_msgs::msg::PoseStampedWithID>
  getPendingWaypoints(const std::string & next_id) const;

  /**
   * @brief Count remaining waypoints in goal_.path from next_id onwards.
   *
   * @param next_id Id of the next pending waypoint.
   * @return Number of waypoints from next_id (inclusive) to end of path.
   */
  uint16_t remainingWaypointCount(const std::string & next_id) const;

  /**
   * @brief Apply a modify request onto goal_.path.
   *
   * Drops already-passed waypoints (those before next_id), updates the pose
   * for any matching id, and appends new ids at the end. Waypoints not
   * referenced in the modify list are kept with their previous pose.
   *
   * @param modify_waypoints Waypoints from the modify goal, already in
   *                         desired_frame_id_.
   * @param next_id Id of the next pending waypoint at modify time.
   * @return The resulting pending waypoint list (= new goal_.path).
   */
  std::vector<as2_msgs::msg::PoseStampedWithID>
  mergeModifyIntoGoal(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & modify_waypoints,
    const std::string & next_id);

  /**
   * @brief Publish trajectory and waypoints debug topics.
   */
  void publishGenerationDebug();

  /**
   * @brief Push the current pending queue to the plugin via updateWaypoints.
   *
   * Refreshes the plugin vehicle state and calls
   * plugin_->updateWaypoints(..., trajectory_time_) with the pending
   * list derived from goal_.path and the plugin's getNextWaypointId().
   * The plugin decides whether to stitch smoothly (preserving its
   * internal offset) or regenerate from scratch (re-anchoring its
   * offset against the current trajectory_time_). The wrapper does NOT
   * realign anything: trajectory_time_ keeps progressing monotonically.
   * Triggers publishGenerationDebug() on success.
   *
   * @return true when the plugin accepts the update.
   */
  bool pushPendingToPlugin();

  /**
   * @brief Evaluate one trajectory setpoint.
   *
   * Refreshes the plugin vehicle state (pose + twist) before delegating to
   * the plugin so closed-loop backends can read it via the protected
   * members.
   *
   * @param t Evaluation time in seconds.
   * @param out Output trajectory point.
   * @param is_horizon_sample True for horizon predictions and debug
   *                          sampling; false for the live control setpoint.
   * @return true when evaluation succeeds.
   */
  bool evaluatePoint(
    double t, as2_msgs::msg::TrajectoryPoint & out,
    bool is_horizon_sample);

  /**
   * @brief Evaluate a horizon of setpoints.
   *
   * @param t0 Initial evaluation time in seconds.
   * @param dt Sampling period in seconds.
   * @param n Number of samples.
   * @param out Output setpoint array.
   * @return true when all samples are evaluated.
   */
  bool evaluateHorizon(
    double t0, double dt, std::size_t n,
    as2_msgs::msg::TrajectorySetpoints & out);

  /**
   * @brief Compute yaw command for a trajectory point.
   *
   * @param point Current trajectory point.
   * @param is_horizon_sample If is first sample of the horizon
   * @return Yaw angle in radians.
   */
  double computeYaw(
    const as2_msgs::msg::TrajectoryPoint & point,
    bool is_horizon_sample);

  /**
   * @brief Compute yaw aligned to XY velocity vector.
   *
   * @param vx X velocity in m/s.
   * @param vy Y velocity in m/s.
   * @return Yaw angle in radians.
   */
  double computeYawAnglePathFacing(double vx, double vy) const;

  /**
   * @brief Compute yaw facing the next reference waypoint, rate-limited.
   *
   * @param current_yaw Yaw of the previous sample
   * @return Yaw angle in radians.
   */
  double computeYawFaceReference(double current_yaw);

  /**
   * @brief Get next reference waypoint based on plugin progress.
   *
   * @return Pointer to waypoint within goal_.path, or nullptr when none.
   */
  const as2_msgs::msg::PoseStampedWithID * getNextReferenceWaypoint() const;

  /**
   * @brief Reset runtime state variables.
   */
  void resetRuntimeState();

  /**
   * @brief Test whether the active goal's last waypoint is closer than
   *        `kDegenerateDistanceM` to the current vehicle pose.
   *
   * Used to short-circuit the plugin and either terminate with SUCCESS
   * (when follow_reference_mode is disabled) or enter a static hold
   * (when follow_reference_mode is enabled).
   *
   * @param last_wp Reference waypoint (typically active_waypoints_.back()).
   * @return true when the distance is below `kDegenerateDistanceM`.
   */
  bool isDegenerateTarget(
    const as2_msgs::msg::PoseStampedWithID & last_wp) const;

  /**
   * @brief Try to latch the host into degenerate-hold for @p waypoints.
   *
   * Convenience wrapper around isDegenerateTarget + enterDegenerateHold,
   * meant to be called BEFORE any plugin invocation
   * (generateTrajectory / updateWaypoints).
   *
   * The hold is only engaged when the active goal contains a SINGLE
   * waypoint (the only case where a near-target trajectory is degenerate
   * by itself — a multi-waypoint mission still has intermediate motion
   * even if its last waypoint sits close to the vehicle).
   *
   * @param waypoints Active waypoint list (typically active_waypoints_).
   * @return true when the hold has been engaged (the plugin must not be
   *         called); false when the size is not 1 or the target is outside
   *         the threshold.
   */
  bool tryEnterDegenerateHold(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints);

  /**
   * @brief Latch the host into degenerate-hold and emit a WARN log.
   *
   * Stores `last_wp.pose.pose.position` and `last_wp.id` so subsequent
   * on_run ticks can publish a static reference even if the active
   * window changes (e.g. modify_waypoint replacing the entry).
   *
   * @param last_wp Reference waypoint whose pose is held.
   */
  void enterDegenerateHold(
    const as2_msgs::msg::PoseStampedWithID & last_wp);

  /**
   * @brief Tick the degenerate-hold branch.
   *
   * - If `!follow_reference_mode_`, returns SUCCESS immediately.
   * - Else if the live last waypoint is again above `kDegenerateDistanceM`,
   *   clears the hold flag, regenerates the trajectory and returns RUNNING
   *   for the next tick (the caller falls through to normal evaluation).
   * - Otherwise publishes a static horizon at `degenerate_target_` and
   *   returns RUNNING.
   *
   * @param feedback_msg Output feedback (next_waypoint_id, remaining).
   * @param result_msg   Output result (success flag).
   * @return ExecutionStatus appropriate for the current tick.
   */
  as2_behavior::ExecutionStatus runDegenerateHold(
    std::shared_ptr<Action::Feedback> & feedback_msg,
    std::shared_ptr<Action::Result> & result_msg);

  /**
   * @brief Build a static horizon of `sampling_n_` setpoints at `target`.
   *
   * Velocity and acceleration are zero; yaw is held at `init_yaw_angle_`.
   *
   * @param target Position to hold.
   * @param out    Output setpoints (resized to `sampling_n_`).
   */
  void buildHoldHorizon(
    const geometry_msgs::msg::Point & target,
    as2_msgs::msg::TrajectorySetpoints & out) const;

  /**
   * @brief Handle vehicle state updates.
   *
   * @param msg Incoming twist message.
   */
  void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief Handle external yaw updates.
   *
   * @param msg Incoming yaw message.
   */
  void yawCallback(const std_msgs::msg::Float32::SharedPtr msg);

  /**
   * @brief Handle waypoint modification requests.
   *
   * @param msg Incoming waypoint updates.
   */
  void modifyWaypointCallback(
    const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg);

  /**
   * @brief Periodic frame-drift watchdog callback.
   */
  void timerUpdateFrameCallback();

  /**
   * @brief Compute whether frame drift exceeds threshold.
   *
   * @return true when regeneration is required.
   */
  bool computeFrameError();

  /**
   * @brief Initialize debug publishers.
   */
  void initDebugPublishers();

  /**
   * @brief Publish trajectory generation/update compute time.
   *
   * No-op when the publisher is disabled (empty topic name in config).
   *
   * @param seconds Compute time in seconds.
   */
  void publishGenerationTime(double seconds);

  /**
   * @brief Split a waypoint list into the active window fed to the plugin
   * and a queue drained incrementally by on_run.
   *
   * No-op when path_length_ <= 0. The active vector is truncated in
   * place. When the plugin's updateWaypoints() falls back to the base
   * default (i.e. regeneration), each window slide implies a fresh
   * trajectory; this is functionally correct but may cause time-base
   * realignment per slide.
   *
   * @param active [in/out] Full pending list. On return, contains only the
   *               first path_length_ entries.
   * @param queued [in/out] Receives the truncated tail (appended in order).
   */
  void splitForPathLength(
    std::vector<as2_msgs::msg::PoseStampedWithID> & active,
    std::deque<as2_msgs::msg::PoseStampedWithID> & queued) const;

  /**
   * @brief Publish sampled generated trajectory for visualization.
   */
  void publishGeneratedTrajectory();

  /**
   * @brief Publish active waypoints markers.
   */
  void publishWaypoints();

  /**
   * @brief Publish evaluated point marker.
   *
   * @param point Point to publish.
   * @param is_last_in_horizon True when it is the horizon last sample.
   */
  void publishEvaluatedPoint(
    const as2_msgs::msg::TrajectoryPoint & point,
    bool is_last_in_horizon);

  // Member variables.

  // Frame ids
  std::string desired_frame_id_;
  std::string map_frame_id_;
  std::string base_link_frame_id_;

  // Handlers
  as2::tf::TfHandler tf_handler_;
  as2::motionReferenceHandlers::TrajectoryMotion trajectory_motion_handler_;
  as2::motionReferenceHandlers::HoverMotion hover_motion_handler_;

  // Plugin
  std::shared_ptr<pluginlib::ClassLoader<PluginBase>> plugin_loader_;
  std::shared_ptr<PluginBase> plugin_;
  std::string plugin_name_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr
    mod_waypoint_sub_;

  // Trajectory behavior parameters
  int sampling_n_{1};
  double sampling_dt_{0.01};
  double yaw_threshold_{0.1};
  double yaw_speed_threshold_{0.0};

  // Aux yaw for face reference
  double horizon_yaw_{0.0};

  // Active-window size fed to the plugin. 0 disables the feature and the
  // wrapper feeds the full mission up front (legacy behavior).
  int path_length_{0};

  // Pending waypoints not yet pushed to the plugin (path_length > 0).
  // Stored in desired_frame_id_, in mission order. Drained as the plugin
  // consumes its active window.
  std::deque<as2_msgs::msg::PoseStampedWithID> pending_waypoints_queue_;

  // Mirror of the waypoint slice currently held by the plugin (without the
  // "current" prefix). Used to count plugin-side remaining and to extend the
  // active window via plugin_->updateWaypoints() when path_length > 0.
  std::vector<as2_msgs::msg::PoseStampedWithID> active_waypoints_;

  // Frame drift checking parameters
  rclcpp::TimerBase::SharedPtr timer_update_frame_;
  double frequency_update_frame_{0.0};
  double transform_threshold_{1.0};
  geometry_msgs::msg::TransformStamped last_map_to_desired_;

  // Wrapper-level latch indicating that the state subscription has
  // delivered at least one valid TwistStamped (so downstream calls
  // can rely on plugin_->getVehiclePose() / getVehicleTwist()).
  bool has_vehicle_state_{false};

  // Stored goal and feedback. goal_.path is the source of truth for the
  // ordered queue of pending mission waypoints (in desired_frame_id_).
  Action::Goal goal_;
  Action::Feedback feedback_;

  // Trajectory time bookkeeping.
  //
  // The wrapper owns a single logical trajectory time axis,
  // trajectory_time_, reset to 0 at every fresh generateTrajectory()
  // call (including external resume regeneration) and advanced by
  // (now - last_tick_time_) on every on_run tick. Each plugin maps
  // trajectory_time_ to its own backend time axis via an internal
  // offset that the wrapper never observes. updateWaypoints() does NOT
  // touch trajectory_time_; the plugin re-anchors its offset on
  // regeneration and preserves it on smooth stitching, so the host
  // axis is unaffected.
  //
  // first_tick_after_anchor_ guards the first tick after generation or
  // resume so the dt accumulator does not jump (last_tick_time_ would
  // otherwise be uninitialized or stale).
  double trajectory_time_{0.0};
  rclcpp::Time last_tick_time_;
  bool first_tick_after_anchor_{true};

  double init_yaw_angle_{0.0};
  rclcpp::Time time_zero_yaw_;

  // External yaw input.
  bool has_yaw_from_topic_{false};
  float yaw_from_topic_{0.0f};

  // Snapshot of next waypoint id captured in on_pause for on_resume.
  std::string paused_next_waypoint_id_;

  // Pause-after-generate flow (Action::Goal::start_on_paused).
  // - start_on_paused_:  copy of the goal flag, latched on_activate.
  // - has_paused_:       true once on_run has emitted the auto-PAUSED
  //                      transition (idempotent, only fires once).
  // - external_pause_:   true once on_pause has been called by an external
  //                      request, so on_resume can distinguish a user
  //                      pause/resume from the auto-pause wake-up.
  bool start_on_paused_{false};
  bool has_paused_{false};
  bool external_pause_{false};

  // Follow-reference mode flag
  bool follow_reference_mode_{false};

  // Degenerate-hold state
  bool degenerate_hold_{false};
  geometry_msgs::msg::Point degenerate_target_;
  std::string degenerate_target_id_;

  // Debug publishers
  bool enable_debug_{false};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    debug_ref_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    debug_end_ref_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    debug_generation_time_pub_;
};

#endif  // GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR_HPP_
