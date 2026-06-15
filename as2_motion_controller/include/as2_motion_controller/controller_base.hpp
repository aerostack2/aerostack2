// Copyright 2023 Universidad Politécnica de Madrid
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
 *  @file       controller_base.hpp
 *  @brief      Declares the as2_motion_controller_plugin_base::ControllerBase class.
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__CONTROLLER_BASE_HPP_
#define AS2_MOTION_CONTROLLER__CONTROLLER_BASE_HPP_

#include <set>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"

namespace as2_motion_controller_plugin_base
{

/**
 * @brief Base class for controller plugins loaded by ControllerManager.
 *
 * Plugins inherit from this class and implement the pure-virtual hooks. The
 * base owns the per-tick state cache, hover latch and essential-parameter
 * tracking so plugins only deal with controller-specific logic.
 */
class ControllerBase
{
public:
  ControllerBase() = default;
  virtual ~ControllerBase() = default;

  ControllerBase(const ControllerBase &) = delete;
  ControllerBase & operator=(const ControllerBase &) = delete;

  // API for ControllerHandler / ControllerManager

  /**
   * @brief Initialize the plugin.
   *
   * Called by ControllerManager after the per-plugin setters have been
   * configured. Declares frame parameters, runs ownInitialize() and seeds
   * the pending-essentials set from getEssentialParameters().
   *
   * @param node_ptr Non-owning pointer to the controller node.
   */
  void initialize(as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
    declareFrameParameters();
    ownInitialize();
    if (!essential_params_ready_) {
      const auto essentials = getEssentialParameters();
      pending_essentials_ = std::set<std::string>(essentials.begin(), essentials.end());
    }
  }

  /**
   * @brief Inject the TfHandler owned by ControllerManager.
   *
   * @param tf_handler Non-owning pointer to the TfHandler instance.
   */
  void setTfHandler(as2::tf::TfHandler * tf_handler) {tf_handler_ = tf_handler;}

  /**
   * @brief Set the namespaced FLU (base_link) frame id used by the controller node.
   *
   * @param frame_id Fully-qualified base_link frame id.
   */
  void setBaseLinkFrameId(const std::string & frame_id) {base_link_frame_id_ = frame_id;}

  /**
   * @brief Set the per-plugin parameter namespace (e.g. "pid_speed_controller").
   *
   * Plugins compose their parameter names with param("foo") which returns
   * "<namespace>.foo".
   *
   * @param ns Plugin parameter namespace.
   */
  void setPluginParamNamespace(const std::string & ns) {plugin_param_namespace_ = ns;}

  /**
   * @brief Request that the next state update synthesizes a hover reference.
   *
   * Called by ControllerHandler after a successful setMode(HOVER).
   */
  void requestHoverLatch() {hover_pending_ = true;}

  /**
   * @brief Override the pose frame id used by the controller for state and references.
   *
   * Typically called from setMode() to react to the active control mode.
   *
   * @param frame_id Fully-qualified pose frame id.
   */
  void setDesiredPoseFrameId(const std::string & frame_id)
  {
    desired_pose_frame_id_ = frame_id;
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Pose frame set to '%s'", frame_id.c_str());
  }

  /**
   * @brief Override the twist frame id used by the controller for state and references.
   *
   * See setDesiredPoseFrameId().
   *
   * @param frame_id Fully-qualified twist frame id.
   */
  void setDesiredTwistFrameId(const std::string & frame_id)
  {
    desired_twist_frame_id_ = frame_id;
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Twist frame set to '%s'", frame_id.c_str());
  }

  /**
   * @brief Frame id (already namespaced) that the plugin expects for pose state and references.
   */
  std::string getDesiredPoseFrameId() const {return desired_pose_frame_id_;}

  /**
   * @brief Frame id (already namespaced) that the plugin expects for twist state and references.
   */
  std::string getDesiredTwistFrameId() const {return desired_twist_frame_id_;}

  /**
   * @brief Whether all essential parameters have been received and applied.
   *
   * Consulted by ControllerHandler before accepting setMode.
   */
  bool essentialParamsReady() const {return essential_params_ready_;}

  /**
   * @brief Filter a parameter batch by the plugin namespace and dispatch each match to updateParameter().
   *
   * Tracks the essential names still pending and, the first time the set
   * empties, flips essential_params_ready_ and calls onAllParametersRead()
   * exactly once. Invoked by ControllerManager with the initial bulk and by
   * ControllerHandler::parametersCallback for runtime changes. The pending
   * set is populated at the end of initialize() from getEssentialParameters()
   * so plugins do not need to manage this state themselves.
   *
   * @param batch Parameter batch from rclcpp.
   */
  void dispatchParameters(const std::vector<rclcpp::Parameter> & batch)
  {
    const std::string prefix = plugin_param_namespace_ + ".";
    const bool latch_was_false = !essential_params_ready_;
    for (const auto & p : batch) {
      const std::string & name = p.get_name();
      if (name.compare(0, prefix.size(), prefix) != 0) {continue;}
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Parameter %s := %s",
        p.get_name().c_str(), p.value_to_string().c_str());
      updateParameter(p);
      pending_essentials_.erase(name);
    }
    if (latch_was_false && pending_essentials_.empty()) {
      essential_params_ready_ = true;
      onAllParametersRead();
    }
  }

  /**
   * @brief Update the latest state (pose + twist) seen by the controller.
   *
   * Validates that the incoming frames match the desired ones, caches the
   * state, consumes a pending hover latch (if any) and forwards the state
   * to onUpdateState().
   *
   * @param pose_msg Latest pose message received by the controller node.
   * @param twist_msg Latest twist message received by the controller node.
   */
  void updateState(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg)
  {
    if (pose_msg.header.frame_id != desired_pose_frame_id_ ||
      twist_msg.header.frame_id != desired_twist_frame_id_)
    {
      auto & clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(
        node_ptr_->get_logger(), clk, 1000,
        "State frame mismatch. Got pose '%s' / twist '%s'. "
        "Expected pose '%s' / twist '%s'.",
        pose_msg.header.frame_id.c_str(), twist_msg.header.frame_id.c_str(),
        desired_pose_frame_id_.c_str(), desired_twist_frame_id_.c_str());
      return;
    }

    state_pose_ = pose_msg;
    state_twist_ = twist_msg;
    state_received_ = true;

    if (hover_pending_) {
      latchHoverReference(state_pose_, state_twist_);
      hover_pending_ = false;
    }

    onUpdateState(pose_msg, twist_msg);
  }

  /**
   * @brief Update the pose reference.
   *
   * @param ref Latest pose reference message received by the controller node.
   */
  void updateReference(const geometry_msgs::msg::PoseStamped & ref)
  {
    reference_received_ = true;
    onUpdateReference(ref);
  }

  /**
   * @brief Update the twist reference.
   *
   * @param ref Latest twist reference message received by the controller node.
   */
  void updateReference(const geometry_msgs::msg::TwistStamped & ref)
  {
    reference_received_ = true;
    onUpdateReference(ref);
  }

  /**
   * @brief Update the trajectory reference.
   *
   * @param ref Latest trajectory reference message received by the controller node.
   */
  void updateReference(const as2_msgs::msg::TrajectorySetpoints & ref)
  {
    reference_received_ = true;
    onUpdateReference(ref);
  }

  /**
   * @brief Update the thrust reference.
   *
   * @param ref Latest thrust reference message received by the controller node.
   */
  void updateReference(const as2_msgs::msg::Thrust & ref)
  {
    reference_received_ = true;
    onUpdateReference(ref);
  }

  // Plugin entry points the derived class must implement

  /**
   * @brief Plugin-specific initialization, called from initialize().
   */
  virtual void ownInitialize() {}

  /**
   * @brief Plugin hook called by the base after frame validation and hover latch.
   *
   * The plugin updates its internal state/integrators here.
   *
   * @param pose_msg Latest validated pose message.
   * @param twist_msg Latest validated twist message.
   */
  virtual void onUpdateState(
    const geometry_msgs::msg::PoseStamped & pose_msg,
    const geometry_msgs::msg::TwistStamped & twist_msg) = 0;

  /**
   * @brief Plugin hook for pose reference. Default: no-op.
   *
   * @param ref Latest pose reference message.
   */
  virtual void onUpdateReference(const geometry_msgs::msg::PoseStamped & /*ref*/) {}

  /**
   * @brief Plugin hook for twist reference. Default: no-op.
   *
   * @param ref Latest twist reference message.
   */
  virtual void onUpdateReference(const geometry_msgs::msg::TwistStamped & /*ref*/) {}

  /**
   * @brief Plugin hook for trajectory reference. Default: no-op.
   *
   * @param ref Latest trajectory reference message.
   */
  virtual void onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & /*ref*/) {}

  /**
   * @brief Plugin hook for thrust reference. Default: no-op.
   *
   * @param ref Latest thrust reference message.
   */
  virtual void onUpdateReference(const as2_msgs::msg::Thrust & /*ref*/) {}

  /**
   * @brief Compute the output signal of the controller plugin.
   *
   * @param dt Time elapsed since the last call to computeOutput().
   * @param pose Output pose; frame depends on the output control mode.
   * @param twist Output twist; frame depends on the output control mode.
   * @param thrust Output thrust.
   * @return true if the output is valid.
   */
  virtual bool computeOutput(
    double dt,
    geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::TwistStamped & twist,
    as2_msgs::msg::Thrust & thrust) = 0;

  /**
   * @brief Update the control mode to be used by the controller plugin.
   *
   * @param mode_in Input control mode requested.
   * @param mode_out Output control mode requested.
   * @return true if the in-out control mode configuration is valid.
   */
  virtual bool setMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out) = 0;

  /**
   * @brief Names of the parameters whose presence is required before the plugin can accept setMode.
   *
   * Names must be already namespaced with `<plugin_name>.`. The manager
   * tracks reception of these names and invokes onAllParametersRead() once
   * the last one arrives.
   *
   * @return Vector of fully-qualified essential parameter names.
   */
  virtual std::vector<std::string> getEssentialParameters() const = 0;

  /**
   * @brief Apply a single parameter to the plugin.
   *
   * Called by the manager for every parameter (essential or not) with a name
   * starting with `<plugin_name>.`, both at startup and on runtime changes.
   * The plugin can read essentialParamsReady() to decide whether to apply at
   * runtime or defer the configuration to onAllParametersRead().
   *
   * @param parameter Parameter to apply.
   */
  virtual void updateParameter(const rclcpp::Parameter & parameter) = 0;

  /**
   * @brief Hook fired once when every essential parameter has been delivered.
   *
   * The latch essentialParamsReady() is already true on entry. Plugins use
   * this hook to perform first-time configuration of the underlying
   * solver/controller from the now-fully-populated parameter set.
   */
  virtual void onAllParametersRead() {}

  /**
   * @brief Reset the controller.
   *
   * Default implementation clears the per-mode flags maintained by the base.
   * Plugins should override and call ControllerBase::reset() so the base
   * state is also cleared. essential_params_ready_ is intentionally NOT
   * cleared here; it is a monotonic latch — parameters are read once at
   * startup via the rclcpp parameter callback, and reset() runs on every
   * successful setMode and would otherwise leave the latch permanently
   * false, rejecting all subsequent mode transitions.
   */
  virtual void reset()
  {
    hover_pending_ = false;
    state_received_ = false;
    reference_received_ = false;
  }

  /**
   * @brief Default hover latch: synthesize a single-point trajectory at the cached state.
   *
   * Override in plugins that do not consume trajectory references; in that
   * case feed the hover via updateReference(pose) / updateReference(twist)
   * with twist set to zero.
   *
   * @param pose Cached state pose used as the hover anchor.
   * @param twist Cached state twist (unused by the default).
   */
  virtual void latchHoverReference(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & /*twist*/)
  {
    // Hover pose reference
    updateReference(pose);

    // Hover speed reference
    geometry_msgs::msg::TwistStamped zero_twist;
    zero_twist.header = pose.header;
    zero_twist.twist.linear.x = 0.0;
    zero_twist.twist.linear.y = 0.0;
    zero_twist.twist.linear.z = 0.0;
    zero_twist.twist.angular.x = 0.0;
    zero_twist.twist.angular.y = 0.0;
    zero_twist.twist.angular.z = 0.0;
    updateReference(zero_twist);

    // Hover trajectory reference
    as2_msgs::msg::TrajectorySetpoints traj;
    traj.header = pose.header;
    as2_msgs::msg::TrajectoryPoint point;
    point.position.x = pose.pose.position.x;
    point.position.y = pose.pose.position.y;
    point.position.z = pose.pose.position.z;
    point.twist.x = 0.0;
    point.twist.y = 0.0;
    point.twist.z = 0.0;
    point.acceleration.x = 0.0;
    point.acceleration.y = 0.0;
    point.acceleration.z = 0.0;
    point.yaw_angle = as2::frame::getYawFromQuaternion(pose.pose.orientation);
    traj.setpoints.push_back(point);
    updateReference(traj);

    // Hover thrust reference
    as2_msgs::msg::Thrust thrust;
    thrust.header = pose.header;
    thrust.thrust = 9.81;  // Default to gravity for hover (needs mass)
    thrust.thrust_normalized = 0.5;
    updateReference(thrust);

    reference_received_ = true;
  }

  /**
   * @brief Whether at least one motion reference has been received.
   */
  bool isReferenceReceived() const {return reference_received_;}

protected:
  // Plugin helpers (read-only access to base-owned state)

  /**
   * @brief Mark the reference as received.
   *
   * @param value Value to set for reference_received_.
   */
  void setReferenceReceived(bool value) {reference_received_ = value;}

  /**
   * @brief Non-owning pointer to the controller node.
   */
  as2::Node * getNodePtr() const {return node_ptr_;}

  /**
   * @brief TfHandler owned by ControllerManager.
   *
   * May be null until the manager injects it; access from ownInitialize() or later.
   */
  as2::tf::TfHandler * getTfHandler() const {return tf_handler_;}

  /**
   * @brief Namespaced frame id of the body FLU/base_link frame.
   */
  const std::string & getBaseLinkFrameId() const {return base_link_frame_id_;}

  /**
   * @brief Per-plugin parameter namespace (e.g. "pid_speed_controller").
   */
  const std::string & getPluginParamNamespace() const {return plugin_param_namespace_;}

  /**
   * @brief Compose a fully-qualified parameter name under the plugin namespace.
   *
   * @param tail Trailing parameter name to append to the plugin namespace.
   * @return Fully-qualified parameter name.
   */
  std::string param(const std::string & tail) const
  {
    return plugin_param_namespace_.empty() ? tail : plugin_param_namespace_ + "." + tail;
  }

  /**
   * @brief Last validated state pose cached by the base.
   */
  const geometry_msgs::msg::PoseStamped & getStatePose() const {return state_pose_;}

  /**
   * @brief Last validated state twist cached by the base.
   */
  const geometry_msgs::msg::TwistStamped & getStateTwist() const {return state_twist_;}

  /**
   * @brief Whether at least one state message has been received and validated.
   */
  bool isStateReceived() const {return state_received_;}

  /**
   * @brief Whether a hover latch is pending consumption on the next state update.
   */
  bool isHoverPending() const {return hover_pending_;}

private:
  // Implementation details

  /**
   * @brief Declare and read the desired_pose_frame / desired_twist_frame parameters.
   *
   * Stores their namespaced values in desired_pose_frame_id_ and
   * desired_twist_frame_id_.
   */
  void declareFrameParameters()
  {
    if (!node_ptr_->has_parameter("desired_pose_frame")) {
      node_ptr_->declare_parameter<std::string>("desired_pose_frame", "odom");
    }
    if (!node_ptr_->has_parameter("desired_twist_frame")) {
      node_ptr_->declare_parameter<std::string>("desired_twist_frame", "base_link");
    }
    const std::string pose_param =
      node_ptr_->get_parameter("desired_pose_frame").as_string();
    const std::string twist_param =
      node_ptr_->get_parameter("desired_twist_frame").as_string();
    desired_pose_frame_id_ = as2::tf::generateTfName(node_ptr_, pose_param);
    desired_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, twist_param);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Controller desired_pose_frame = '%s', desired_twist_frame = '%s'",
      desired_pose_frame_id_.c_str(), desired_twist_frame_id_.c_str());
  }

  // Node and configuration injected from outside the plugin.
  as2::Node * node_ptr_ = nullptr;
  as2::tf::TfHandler * tf_handler_ = nullptr;
  std::string base_link_frame_id_;
  std::string plugin_param_namespace_;
  std::string desired_pose_frame_id_;
  std::string desired_twist_frame_id_;

  // Last validated state cached by updateState() for hover latch and
  // diagnostics.
  geometry_msgs::msg::PoseStamped state_pose_;
  geometry_msgs::msg::TwistStamped state_twist_;

  // Plugin-side flags owned by the base.
  bool state_received_ = false;
  bool reference_received_ = false;
  bool hover_pending_ = false;
  bool essential_params_ready_ = false;

  // Set of essential parameter names not yet received by the plugin.
  // Populated at the end of initialize() from getEssentialParameters() and
  // decremented inside dispatchParameters() as parameters arrive. Once
  // emptied for the first time, the latch above is flipped and
  // onAllParametersRead() fires.
  std::set<std::string> pending_essentials_;
};   // class ControllerBase

}  // namespace as2_motion_controller_plugin_base

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_BASE_HPP_
