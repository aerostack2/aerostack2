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
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_motion_controller/param_utils.hpp"
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
 * Plugins inherit from this class and implement the pure-virtual hooks.
 */
class ControllerBase
{
public:
  /**
   * @brief Construct the Controller Base object. The plugin is only usable
   * after initialize().
   */
  ControllerBase() = default;

  /**
   * @brief Destroy the Controller Base object.
   */
  virtual ~ControllerBase() = default;

  /**
   * @brief Not copyable: the plugin holds the node it was initialized with.
   */
  ControllerBase(const ControllerBase &) = delete;

  /**
   * @brief Not copy assignable, for the same reason.
   */
  ControllerBase & operator=(const ControllerBase &) = delete;

  // API for ControllerHandler / ControllerManager

  /**
   * @brief Initialize the plugin.
   *
   * Called by ControllerManager after the per-plugin setters have been
   * configured. Declares the frame parameters and runs ownInitialize().
   *
   * @param node_ptr Non-owning pointer to the controller node.
   */
  void initialize(as2::Node * node_ptr)
  {
    node_ptr_ = node_ptr;
    declareFrameParameters();
    deliverInitParameters();
    ownInitialize();
    const auto required = requiredParameters();
    missing_parameters_ = std::set<std::string>(required.begin(), required.end());
  }

  /**
   * @brief Inject the TfHandler owned by ControllerManager.
   *
   * @param tf_handler Non-owning pointer to the TfHandler instance.
   */
  void setTfHandler(as2::tf::TfHandler * tf_handler) {tf_handler_ = tf_handler;}

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
   * @brief Deliver every parameter of the plugin namespace to updateParameter().
   *
   * @param batch Parameter batch from rclcpp.
   */
  void dispatchParameters(const std::vector<rclcpp::Parameter> & batch)
  {
    const std::string prefix = plugin_param_namespace_ + ".";
    for (const auto & p : batch) {
      const std::string & name = p.get_name();
      if (name.compare(0, prefix.size(), prefix) != 0) {continue;}
      const std::string tail = name.substr(prefix.size());
      if (base_claimed_parameters_.count(tail) != 0) {continue;}
      if (init_parameters_.count(tail) != 0) {
        // The initial bulk repeats what deliverInitParameters() already applied.
        if (initial_dispatch_done_) {
          RCLCPP_WARN(
            node_ptr_->get_logger(),
            "Parameter '%s' is applied when the plugin is built, the change has no effect",
            tail.c_str());
        }
        continue;
      }
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Parameter %s := %s",
        name.c_str(), p.value_to_string().c_str());
      updateParameter(tail, p);
      missing_parameters_.erase(tail);
    }
    initial_dispatch_done_ = true;
  }

  /**
   * @brief Update the latest state (pose + twist) seen by the controller.
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
   * @brief Apply one parameter of the plugin to the controller.
   *
   * @param name Parameter name, without the plugin namespace.
   * @param param Parameter as delivered.
   */
  virtual void updateParameter(
    const std::string & name,
    const rclcpp::Parameter & param) = 0;

  /**
   * @brief Names of the parameters the plugin needs before it can control.
   *
   * @return Parameter names, or an empty list when the plugin needs none.
   */
  virtual std::vector<std::string> requiredParameters() const {return {};}

  /**
   * @brief Names of the parameters the plugin consumes when it builds itself.
   *
   * Delivered through updateParameter() before ownInitialize(), so whatever
   * depends on them is built with their value. They are not applied again: a
   * later change is reported and ignored, since the object that read them is
   * not rebuilt.
   *
   * @return Parameter names, without the plugin namespace.
   */
  virtual std::vector<std::string> initParameters() const {return {};}

  /**
   * @brief Plugin hook called by the base after frame validation.
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
  bool setMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out)
  {
    if (!missing_parameters_.empty()) {
      for (const auto & name : missing_parameters_) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Parameter '%s' is not provided by any configuration file", name.c_str());
      }
      return false;
    }

    if (!onSetMode(mode_in, mode_out)) {
      return false;
    }

    control_mode_in_ = mode_in;
    control_mode_out_ = mode_out;
    return true;
  }

  /**
   * @brief Mark whether the mode just set serves a hover request.
   *
   * @param enabled true when the accepted mode serves a hover request.
   */
  void setHoverEnabled(bool enabled) {hover_enabled_ = enabled;}

  /**
   * @brief Control mode the plugin runs to perform a hover request.
   *
   * @return Control mode the plugin does the hover with, UNSET when it cannot hover.
   */
  virtual as2_msgs::msg::ControlMode hoverMode() const
  {
    as2_msgs::msg::ControlMode mode;
    mode.control_mode = as2_msgs::msg::ControlMode::UNSET;
    return mode;
  }

  /**
   * @brief Plugin hook to accept or refuse a control mode pair.
   *
   * @param mode_in Input control mode requested.
   * @param mode_out Output control mode requested.
   * @return true if the plugin accepts the pair.
   */
  virtual bool onSetMode(
    const as2_msgs::msg::ControlMode & mode_in,
    const as2_msgs::msg::ControlMode & mode_out)
  {
    (void)mode_in;
    (void)mode_out;
    return true;
  }

  /**
   * @brief Reset the controller.
   *
   * Default implementation clears the per-mode flags maintained by the base.
   * Plugins should override and call ControllerBase::reset() so the base
   * state is also cleared.
   */
  virtual void reset()
  {
    state_received_ = false;
    reference_received_ = false;
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
   * @brief Create a publisher for an optional debug topic.
   *
   * Debug topics are opt-in: the plugin declares a `debug.<name>_topic`
   * parameter and the publisher only exists when it is set to a non-empty
   * topic name. The name is resolved by debugTopicName(), so the
   * configuration file carries the leaf name and not the debug namespace.
   *
   * @tparam MsgT Message type of the topic.
   * @param topic_param_tail Parameter name without the plugin namespace.
   * @param qos Quality of service of the publisher.
   * @return The publisher, or nullptr when the parameter is empty.
   */
  template<typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createDebugPublisher(
    const std::string & topic_param_tail,
    const rclcpp::QoS & qos = rclcpp::SensorDataQoS())
  {
    base_claimed_parameters_.insert(topic_param_tail);
    const auto topic = as2_motion_controller_param_utils::debugTopicName(
      node_ptr_->template getParameter<std::string>(param(topic_param_tail), ""));
    if (topic.empty()) {
      return nullptr;
    }
    return node_ptr_->template create_publisher<MsgT>(topic, qos);
  }

  /**
   * @brief Input control mode currently active.
   */
  const as2_msgs::msg::ControlMode & getControlModeIn() const {return control_mode_in_;}

  /**
   * @brief Output control mode negotiated with the platform.
   */
  const as2_msgs::msg::ControlMode & getControlModeOut() const {return control_mode_out_;}

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
   * @brief Whether the active mode is serving a hover request.
   */
  bool isHoverEnabled() const {return hover_enabled_;}

private:
  // Implementation details

  /**
   * @brief Deliver the parameters the plugin consumes when it builds itself.
   */
  void deliverInitParameters()
  {
    const auto names = initParameters();
    init_parameters_ = std::set<std::string>(names.begin(), names.end());
    for (const auto & tail : init_parameters_) {
      const std::string name = param(tail);
      if (!node_ptr_->has_parameter(name)) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "Parameter '%s' is not provided by any configuration file", tail.c_str());
        continue;
      }
      const rclcpp::Parameter parameter = node_ptr_->get_parameter(name);
      RCLCPP_INFO(
        node_ptr_->get_logger(), "Parameter %s := %s",
        name.c_str(), parameter.value_to_string().c_str());
      updateParameter(tail, parameter);
    }
  }

  /**
   * @brief Declare and read the desired_pose_frame_id / desired_twist_frame_id parameters.
   *
   * Empty takes the canonical frames of the node, which are already namespaced.
   * Stores their namespaced values in desired_pose_frame_id_ and
   * desired_twist_frame_id_.
   */
  void declareFrameParameters()
  {
    const std::string pose_param =
      node_ptr_->getParameter<std::string>("desired_pose_frame", "");
    const std::string twist_param =
      node_ptr_->getParameter<std::string>("desired_twist_frame", "");
    desired_pose_frame_id_ = pose_param.empty() ?
      node_ptr_->getOdomFrameId() : as2::tf::generateTfName(node_ptr_, pose_param);
    desired_twist_frame_id_ = twist_param.empty() ?
      node_ptr_->getBaseFrameId() : as2::tf::generateTfName(node_ptr_, twist_param);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Controller desired_pose_frame_id = '%s', desired_twist_frame_id = '%s'",
      desired_pose_frame_id_.c_str(), desired_twist_frame_id_.c_str());
  }

  // Node and configuration injected from outside the plugin.
  as2::Node * node_ptr_ = nullptr;
  as2::tf::TfHandler * tf_handler_ = nullptr;
  std::string plugin_param_namespace_;
  std::string desired_pose_frame_id_;
  std::string desired_twist_frame_id_;

  // Last validated state cached by updateState()
  geometry_msgs::msg::PoseStamped state_pose_;
  geometry_msgs::msg::TwistStamped state_twist_;

  // Plugin-side flags owned by the base.
  bool state_received_ = false;
  bool reference_received_ = false;
  bool hover_enabled_ = false;

  // Parameter to be read
  std::set<std::string> base_claimed_parameters_;

  // Parameter tails delivered once, before the plugin is built.
  std::set<std::string> init_parameters_;
  bool initial_dispatch_done_ = false;

  // Required parameter tails not delivered yet.
  std::set<std::string> missing_parameters_;

  // Control modes in use
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;
};   // class ControllerBase

}  // namespace as2_motion_controller_plugin_base

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_BASE_HPP_
