// Copyright 2025 Universidad Politécnica de Madrid
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

/**
* @file plugin_wrapper.hpp
*
* A wrapper for the plugins in the state estimation server for AeroStack2, focused in easing the
* implementation of metacontrol layers
*
* @authors Miguel Fernández Cortizas
*/

#ifndef AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_
#define AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_

#include <tf2/LinearMath/Transform.h>

#include <optional>
#include <string>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
#include <as2_core/utils/frame_utils.hpp>
#include <as2_core/utils/tf_utils.hpp>

#include "as2_state_estimator/robot_state.hpp"
#include "as2_state_estimator/plugin_base.hpp"

using StateComponent = std::variant<
  geometry_msgs::msg::PoseWithCovariance,
  geometry_msgs::msg::TwistWithCovariance>;

namespace as2_state_estimator
{

/**
 * @brief Owns one state estimator plugin and bridges it with the StateEstimator node.
 *
 * Keeps the plugin instance, the state it produces, its TF handler and its per-plugin debug
 * publishers together, so the node only ever deals with wrappers. Wrappers are obtained through
 * create(); a default constructed one holds no plugin and is not usable.
 */
class PluginWrapper
{
public:
  using SharedPtr = std::shared_ptr<PluginWrapper>;

  /// @brief Plugin class namespace, also the authority name this plugin is known by in the node.
  std::string plugin_name_;

  /// @brief Loaded plugin instance. The wrapper must remain its only owner, see ~PluginWrapper().
  std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase> plugin_ptr;

  /**
   * @brief Build a wrapper and load, configure and set up the named plugin.
   *
   * Instantiates "<plugin_name>::Plugin" through @p loader, gives it a TF handler whose listener
   * drops the estimator's own canonical transforms (see filterTransformRule()) and calls setup()
   * on the plugin. It then reads the "<plugin_name>.debug_publish_hz" parameter of the
   * StateEstimator node to configure the debug topics "state_estimation/<plugin_name>/pose" and
   * ".../twist": 0.0 disables them and creates no publishers, a positive value publishes them
   * from a wall timer at that rate, and a negative value (the default) publishes them on every
   * TWIST_IN_BASE update. Requires the StateEstimator singleton to already exist.
   *
   * @param plugin_name Plugin class namespace, without the "::Plugin" suffix.
   * @param loader Class loader used to instantiate the plugin, kept alive by the caller.
   * @return The wrapper, or std::nullopt if pluginlib failed to load the plugin. That failure is
   *         logged as FATAL and swallowed, so the caller gets an empty optional and never an
   *         exception.
   */
  static std::optional<PluginWrapper::SharedPtr> create(
    const std::string & plugin_name,
    std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
    loader);

  /**
   * @brief TF filter rule with inverted polarity: false means "drop this transform".
   *
   * The as2::tf::FilteredTransformListener behind the plugin's TF handler skips any transform for
   * which some rule returns false, and only feeds the buffer with those where every rule returns
   * true. This rule accordingly returns false for the three canonical links the estimator itself
   * publishes (earth->map, map->odom, odom->base_link, resolved from the node frame parameters)
   * and true for everything else, so that a plugin never reads its own output back from /tf.
   *
   * @param transform Transform received on /tf or /tf_static.
   * @return False for the three canonical estimator links, true for any other transform.
   */
  bool filterTransformRule(const geometry_msgs::msg::TransformStamped & transform);

  friend class PluginWrapperInterface;

  /**
   * @brief Notify the StateEstimator that this plugin has updated one part of the robot state.
   *
   * Forwards to StateEstimator::receiveStateUpdate(), which ignores the update if this plugin is
   * not the authority for @p type. When the debug topics are enabled and not rate limited, a
   * TWIST_IN_BASE update also publishes this plugin's own pose and twist beforehand.
   *
   * @param type Part of the robot state the plugin has just written into robot_state_.
   */
  void advertiseUpdate(TransformInformatonType type);

  /**
   * @brief Construct an empty wrapper, with no plugin, TF handler nor publishers.
   *
   * Exists for create(), which is the supported way to obtain a usable wrapper.
   */
  PluginWrapper() {}

  /**
   * @brief Release the plugin instance.
   *
   * Asserts that the wrapper is the plugin's only owner, since the plugin holds an interface
   * pointing back at the wrapper and would be left dangling if it outlived it.
   */
  ~PluginWrapper()
  {
    // assert is the only reference to the plugin before deleting it
    assert(plugin_ptr.use_count() == 1);
    plugin_ptr.reset();
  }

  /// @brief State produced by this plugin, merged into the node state by advertiseUpdate().
  RobotState robot_state_;

private:
  std::shared_ptr<StateEstimatorInterface> interface;
  std::shared_ptr<as2::tf::TfHandler> tf_handler;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr plugin_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr plugin_twist_pub;

  double debug_publish_hz_ = -1.0;
  rclcpp::TimerBase::SharedPtr debug_timer_;  // null = publish on every TWIST_IN_BASE update
};


}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_HPP_
