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
 * @file generate_polynomial_trajectory_base.hpp
 *
 * @brief Abstract base class for polynomial trajectory generator plugins.
 *
 * The base keeps only the minimal plugin-facing contract:
 *   - lifecycle initialization with access to the hosting node,
 *   - a namespaced `getParameter()` helper for plugin configuration,
 *   - the pure virtual generate/evaluate/reset/progress API.
 *
 * All ROS 2 runtime orchestration remains in
 * GeneratePolynomialTrajectoryBehavior.
 *
 * @authors Rafael Perez-Segui
 */

#ifndef GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BASE_HPP_
#define GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BASE_HPP_

#include <cstdint>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"

namespace generate_polynomial_trajectory_behavior_plugin_base
{

/**
 * @brief Declare (if needed) and read a ROS 2 parameter on the hosting node.
 *
 * @tparam T          Parameter type.
 * @param node_ptr    Node pointer.
 * @param param_name  Fully-qualified parameter name.
 * @param param_value [in] default value used when @p use_default is true,
 *                    [out] read value on success.
 * @param use_default Whether @p param_value is taken as a fallback default.
 */
template<typename T>
inline void getParameter(
  as2::Node * node_ptr, const std::string & param_name,
  T & param_value, bool use_default = false)
{
  try {
    if (!node_ptr->has_parameter(param_name)) {
      if (use_default) {
        node_ptr->declare_parameter<T>(param_name, param_value);
      } else {
        node_ptr->declare_parameter<T>(param_name);
      }
    }

    if constexpr (std::is_same<T, std::vector<double>>::value) {
      param_value = node_ptr->get_parameter(param_name).as_double_array();
    } else if constexpr (std::is_same<T, double>::value) {
      param_value = node_ptr->get_parameter(param_name).as_double();
    } else if constexpr (std::is_same<T, std::string>::value) {
      param_value = node_ptr->get_parameter(param_name).as_string();
    } else if constexpr (std::is_same<T, bool>::value) {
      param_value = node_ptr->get_parameter(param_name).as_bool();
    } else if constexpr (std::is_same<T, int>::value) {
      param_value =
        static_cast<int>(node_ptr->get_parameter(param_name).as_int());
    } else {
      RCLCPP_WARN(
        node_ptr->get_logger(),
        "Parameter type %s not expected; falling back to generic accessor.",
        typeid(T).name());
      param_value = node_ptr->get_parameter(param_name).get_value<T>();
    }
    RCLCPP_INFO(
      node_ptr->get_logger(), "Parameter %s: %s", param_name.c_str(),
      node_ptr->get_parameter(param_name).value_to_string().c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_ptr->get_logger(), "Error getting parameter %s: %s",
      param_name.c_str(), e.what());
  }
}

/**
 * @brief Distance threshold (in metres) under which the host behaviour
 * short-circuits the plugin and publishes a degenerate-hold trajectory:
 * the last waypoint is sent as a static reference with zero velocity and
 * zero acceleration.
 */
inline constexpr double kDegenerateDistanceM = 0.05;

/**
 * @brief Abstract plugin contract for polynomial trajectory generation.
 */
class GeneratePolynomialTrajectoryBase
{
public:
  /**
   * @brief Construct plugin base instance.
   */
  GeneratePolynomialTrajectoryBase() = default;

  /**
   * @brief Virtual destructor.
   */
  virtual ~GeneratePolynomialTrajectoryBase() = default;

  /**
   * @brief Initialize the plugin with the hosting node and its namespace.
   *
   * Plugins may use the node only for configuration and logging.
   *
   * @param node Hosting node pointer.
   * @param plugin_name Plugin namespace identifier.
   */
  void initialize(as2::Node * node, const std::string & plugin_name);

  /**
   * @brief Generate a fresh trajectory from mission waypoints.
   *
   * The initial vehicle state (position and velocity) is NOT passed as
   * an argument: the plugin reads it from the protected base members
   * vehicle_pose_ / vehicle_twist_ (refreshed by the host via
   * setVehicleState before this call). Plugins must inject vehicle_pose_
   * as the trajectory start position and may use vehicle_twist_ as the
   * starting velocity boundary condition when their backend supports it.
   * The buildCurrentWaypoint() helper provides a ready-to-use entry for
   * backends that expect the initial state as the first waypoint.
   *
   * The host owns a single logical trajectory time axis exposed as
   * t_trajectory (seconds, monotonic, reset to 0 on every fresh
   * generation). Plugins are responsible for mapping t_trajectory to
   * whatever internal time axis their backend uses; the host never
   * observes that mapping. On entry, t_trajectory_now indicates the
   * current value of the host's axis, so the plugin can anchor its
   * internal offset such that t_backend == 0 when t_trajectory ==
   * t_trajectory_now (or whichever convention the backend prefers).
   *
   * @param waypoints Ordered mission waypoints (the user-specified path
   *                  only, no synthetic "current" entry).
   * @param max_speed Maximum cruise speed in m/s.
   * @param t_trajectory_now Current value of the host's trajectory time
   *                  axis at the moment of generation (typically 0.0
   *                  for a fresh activation, or the live trajectory_time_
   *                  for a regeneration triggered by updateWaypoints()).
   * @return true when trajectory generation succeeds.
   */
  virtual bool
  generateTrajectory(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed,
    double t_trajectory_now) = 0;

  /**
   * @brief Evaluate trajectory references at time t_trajectory.
   *
   * The vehicle pose and twist are not passed in this call: they are kept
   * as protected members (vehicle_pose_, vehicle_twist_) refreshed by the
   * host via setVehicleState(). Implementations that need closed-loop
   * feedback (e.g. dynamic re-planners) read them directly and must gate
   * any state-mutating use on @p is_horizon_sample being false, since
   * @p is_horizon_sample is true for both horizon predictions and debug
   * evaluations.
   *
   * The plugin maps t_trajectory to its internal backend axis using its
   * private offset and clamps to its own valid range; values outside
   * the trajectory horizon are saturated to the closest endpoint
   * rather than returning false.
   *
   * @param t_trajectory Evaluation time in seconds, in the host's
   *                     trajectory time axis.
   * @param out Output reference point.
   * @param is_horizon_sample True for horizon predictions and debug
   *                          sampling; false for the live control setpoint.
   * @return true when evaluation succeeds.
   */
  virtual bool evaluate(
    double t_trajectory,
    as2_msgs::msg::TrajectoryPoint & out,
    bool is_horizon_sample = false) = 0;

  /**
   * @brief Whether the trajectory is exhausted at the given t_trajectory.
   *
   * Returns true when t_trajectory has reached or exceeded the end of
   * the currently held trajectory in the host's time axis.
   *
   * @param t_trajectory Current host trajectory time in seconds.
   * @return true when the trajectory has finished.
   */
  virtual bool isFinished(double t_trajectory) const = 0;

  /**
   * @brief Total duration of the currently held trajectory.
   *
   * Returns the temporal length of the trajectory in seconds (i.e.
   * backend max - backend min in the plugin's internal axis). Used by
   * the host for diagnostics and feedback only; it does NOT expose the
   * backend axis offset to the wrapper.
   *
   * @return Duration in seconds, or 0.0 when no trajectory is held.
   */
  virtual double getDuration() const = 0;

  /**
   * @brief Check whether a trajectory is currently generated.
   *
   * @return true when a valid trajectory is available.
   */
  virtual bool isTrajectoryGenerated() = 0;

  /**
   * @brief Reset internal plugin trajectory state.
   */
  virtual void reset() = 0;

  /**
   * @brief Update the active trajectory with a new pending waypoint list.
   *
   * Default implementation: regenerate from scratch by calling reset()
   * followed by generateTrajectory() with the current vehicle state.
   * Plugins that support smooth online re-planning (i.e. preserving the
   * trajectory time origin so evaluation continues without restart)
   * override this method and keep their internal time-axis offset
   * unchanged so the host's t_trajectory keeps mapping to the same
   * backend trajectory.
   *
   * @param waypoints Ordered pending waypoints (no helper "current" entry).
   * @param max_speed Maximum cruise speed in m/s.
   * @param t_trajectory_now Current value of the host's trajectory time
   *                  axis. Plugins that regenerate must re-anchor their
   *                  offset to this value; plugins that stitch must
   *                  keep their offset untouched.
   * @return true when the update succeeds.
   */
  virtual bool
  updateWaypoints(
    const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
    double max_speed,
    double t_trajectory_now)
  {
    reset();
    return generateTrajectory(waypoints, max_speed, t_trajectory_now);
  }

  /**
   * @brief Consume and return whether the underlying backend trajectory
   * has been regenerated since the previous call.
   *
   * Plugins with a synchronous updateWaypoints() (the base default)
   * always return false: their swap is observable immediately on return
   * from updateWaypoints() and the host can rely on the publish that
   * happens right after in pushPendingToPlugin().
   *
   * Plugins that maintain an asynchronous regeneration pipeline
   * override this to expose the deferred swap event so the host can
   * refresh debug publications once the backend has actually swapped
   * its internal trajectory.
   *
   * @return true at most once per regeneration event.
   */
  virtual bool consumeRegeneratedFlag() {return false;}

  /**
   * @brief Get the id of the next pending waypoint as known by the plugin.
   *
   * @return Empty string when there is no pending waypoint, else its id.
   */
  virtual std::string getNextWaypointId() = 0;

  /**
   * @brief Refresh the vehicle pose and twist held by the plugin.
   *
   * The host is expected to call this once per incoming state message
   * (in its odom/twist subscription callback). The plugin therefore
   * always holds the latest valid pose and twist; plugin entry points
   * (generateTrajectory, updateWaypoints, evaluate) do not need a
   * per-call refresh from the host. Both fields are stored as-is;
   * coordinate frame consistency with the trajectory frame is the
   * host's responsibility.
   *
   * @param pose Current vehicle pose in the trajectory frame.
   * @param twist Current vehicle twist (linear in body or world per the
   *              host configuration; not interpreted by the base class).
   */
  void setVehicleState(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & twist);

  /**
   * @brief Read-only access to the latest vehicle pose set by the host.
   *
   * The host refreshes the underlying member through setVehicleState()
   * once per state message; the value seen here is the latest value
   * successfully delivered by the host.
   *
   * @return Const reference to the cached vehicle pose.
   */
  const geometry_msgs::msg::PoseStamped & getVehiclePose() const
  {
    return vehicle_pose_;
  }

  /**
   * @brief Read-only access to the latest vehicle twist set by the host.
   *
   * @return Const reference to the cached vehicle twist.
   */
  const geometry_msgs::msg::TwistStamped & getVehicleTwist() const
  {
    return vehicle_twist_;
  }

protected:
  /**
   * @brief Optional plugin-specific initialization hook.
   */
  virtual void ownInitialize() {}

  /**
   * @brief Build a synthetic waypoint capturing the current vehicle pose.
   *
   * Returns a PoseStampedWithID with id="current" and pose=vehicle_pose_,
   * intended to be prepended to the waypoint list before feeding the
   * backend whenever the backend expects the start state as the first
   * waypoint. The accompanying initial velocity must be read separately
   * from vehicle_twist_ — PoseStampedWithID does not carry twist.
   *
   * @return Synthetic "current" waypoint built from vehicle_pose_.
   */
  as2_msgs::msg::PoseStampedWithID buildCurrentWaypoint() const;

  /**
   * @brief Read a plugin-scoped parameter.
   *
   * @tparam T Parameter type.
   * @param param_name Parameter name.
   * @param param_value [in] default value when @p use_default is true,
   *                    [out] read value.
   * @param use_default Whether to declare with default value.
   */
  template<typename T>
  inline void getParameter(
    const std::string & param_name, T & param_value,
    bool use_default = false)
  {
    generate_polynomial_trajectory_behavior_plugin_base::getParameter(
      node_ptr_, qualifyParameterName(param_name), param_value, use_default);
  }

  /**
   * @brief Get mutable hosting node pointer.
   *
   * @return Mutable hosting node pointer.
   */
  as2::Node * getNodePtr() {return node_ptr_;}

  /**
   * @brief Get const hosting node pointer.
   *
   * @return Const hosting node pointer.
   */
  const as2::Node * getNodePtr() const {return node_ptr_;}

  /**
   * @brief Get plugin namespace name.
   *
   * @return Plugin namespace string.
   */
  const std::string & getPluginName() const {return plugin_name_;}

  // Latest vehicle state set by the host via setVehicleState(). Derived
  // plugins may read these directly inside generateTrajectory(),
  // updateWaypoints() and evaluate(); they are valid only after the host
  // has invoked the setter at least once.
  geometry_msgs::msg::PoseStamped vehicle_pose_;
  geometry_msgs::msg::TwistStamped vehicle_twist_;

  // Node pointer
  as2::Node * node_ptr_{nullptr};

private:
  /**
   * @brief Qualify parameter name with plugin namespace.
   *
   * @param param_name Parameter key.
   * @return Namespaced parameter key.
   */
  std::string qualifyParameterName(const std::string & param_name) const;

  std::string plugin_name_;
};

}  // namespace generate_polynomial_trajectory_behavior_plugin_base

#endif  // GENERATE_POLYNOMIAL_TRAJECTORY_BEHAVIOR__GENERATE_POLYNOMIAL_TRAJECTORY_BASE_HPP_
