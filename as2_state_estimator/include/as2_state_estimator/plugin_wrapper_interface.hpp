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
* @file plugin_wrapper_interface.hpp
*
* A wrapper for the plugins in the state estimation server for AeroStack2, focused in easing the
* implementation of metacontrol layers
*
* @authors Miguel Fernández Cortizas
*          Rodrigo Da Silva Gómez
*/

#ifndef AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_INTERFACE_HPP_
#define AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_INTERFACE_HPP_

#include <memory>
#include <string>

#include "as2_state_estimator/as2_state_estimator.hpp"
#include "as2_state_estimator/plugin_base.hpp"
#include "as2_state_estimator/plugin_wrapper.hpp"


namespace as2_state_estimator
{

class PluginWrapperInterface : public StateEstimatorInterface
{
public:
  /**
   * @brief Bind the interface to the wrapper it reports through.
   *
   * @param plugin_wrapper Wrapper that owns this interface. It must outlive it, since the
   * pointer is kept raw.
   */
  explicit PluginWrapperInterface(
    PluginWrapper * plugin_wrapper)
  : plugin_wrapper_(plugin_wrapper)
  {}

  /**
   * @brief Global earth frame id, taken from the state estimator node.
   *
   * @return Frame id.
   */
  inline const std::string & getEarthFrame() override
  {
    return StateEstimator::getEarthFrame();
  }
  /**
   * @brief Namespaced map frame id, taken from the state estimator node.
   *
   * @return Frame id.
   */
  inline const std::string & getMapFrame() override
  {
    return StateEstimator::getMapFrame();
  }
  /**
   * @brief Namespaced odom frame id, taken from the state estimator node.
   *
   * @return Frame id.
   */
  inline const std::string & getOdomFrame() override
  {
    return StateEstimator::getOdomFrame();
  }
  /**
   * @brief Namespaced base_link frame id, taken from the state estimator node.
   *
   * @return Frame id.
   */
  inline const std::string & getBaseFrame() override
  {
    return StateEstimator::getBaseFrame();
  }

  /**
   * @brief Set the pose of the map frame (local for each robot) in the earth frame (global)
   * @param pose The pose of the map frame in the earth frame with covariance
   */
  void setEarthToMap(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) override
  {
    plugin_wrapper_->robot_state_.processStateComponent(
      plugin_wrapper_->plugin_name_, pose, TransformInformatonType::EARTH_TO_MAP, stamp, is_static);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::EARTH_TO_MAP);
  }

  /**
   * @brief Set the pose of the map frame in the earth frame, without covariance.
   *
   * Converts the transform and delegates on the overload above, so the covariance is left
   * at its default all-zeros value.
   *
   * @param pose Pose of map in earth.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  void setEarthToMap(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) override
  {
    geometry_msgs::msg::PoseWithCovariance pose_msg;
    tf2::toMsg(pose, pose_msg.pose);
    setEarthToMap(pose_msg, stamp, is_static);
  }

  /**
   * @brief Set the pose of the robot from the map frame to the odom frame
   * @param pose The pose of the robot from the map frame to the odom frame with covariance
   */
  void setMapToOdomPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) override
  {
    plugin_wrapper_->robot_state_.processStateComponent(
      plugin_wrapper_->plugin_name_, pose, TransformInformatonType::MAP_TO_ODOM, stamp, is_static);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::MAP_TO_ODOM);
  }
  /**
   * @brief Set the pose of the odom frame in the map frame, without covariance.
   *
   * Converts the transform and delegates on the overload above, so the covariance is left
   * at its default all-zeros value.
   *
   * @param pose Pose of odom in map.
   * @param stamp Time the pose refers to.
   * @param is_static True to broadcast it as a static transform.
   */
  void setMapToOdomPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) override
  {
    geometry_msgs::msg::PoseWithCovariance pose_msg;
    tf2::toMsg(pose, pose_msg.pose);
    setMapToOdomPose(pose_msg, stamp, is_static);
  }
  /**
   * @brief Set the pose of the robot from the odom frame to the base_link frame
   * @param pose The pose of the robot from the odom frame to the base_link frame with covariance
   */
  void setOdomToBaseLinkPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp) override
  {
    plugin_wrapper_->robot_state_.processStateComponent(
      plugin_wrapper_->plugin_name_, pose, TransformInformatonType::ODOM_TO_BASE, stamp);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::ODOM_TO_BASE);
  }
  /**
   * @brief Set the pose of the base_link frame in the odom frame, without covariance.
   *
   * Converts the transform and delegates on the overload above, so the covariance is left
   * at its default all-zeros value.
   *
   * @param pose Pose of base_link in odom.
   * @param stamp Time the pose refers to.
   */
  void setOdomToBaseLinkPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp
  ) override
  {
    geometry_msgs::msg::PoseWithCovariance pose_msg;
    tf2::toMsg(pose, pose_msg.pose);
    setOdomToBaseLinkPose(pose_msg, stamp);
  }
  /**
   * @brief Set the twist of the robot in the base_link frame
   * @param twist The twist of the robot in the base_link frame with covariance
   */
  void setTwistInBaseFrame(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp) override
  {
    plugin_wrapper_->robot_state_.processStateComponent(
      plugin_wrapper_->plugin_name_, twist, TransformInformatonType::TWIST_IN_BASE, stamp);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::TWIST_IN_BASE);
  }
  // TODO(miferco97): IMPROVE THIS TO ALLOW USING THE PLUGIN TO ACCESS BEST TRANSFORMS
  /**
   * @brief Earth to map transform as this plugin sees it.
   *
   * Read through the plugin RobotState, which falls back to the shared state when this
   * plugin does not estimate that link.
   *
   * @return Transform, the identity while nobody has written it.
   */
  tf2::Transform getEarthToMapTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::EARTH_TO_MAP);
  }
  /**
   * @brief Map to odom transform as this plugin sees it.
   *
   * Read through the plugin RobotState, which falls back to the shared state when this
   * plugin does not estimate that link.
   *
   * @return Transform, the identity while nobody has written it.
   */
  tf2::Transform getMapToOdomTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::MAP_TO_ODOM);
  }
  /**
   * @brief Odom to base_link transform as this plugin sees it.
   *
   * Read through the plugin RobotState, which falls back to the shared state when this
   * plugin does not estimate that link.
   *
   * @return Transform, the identity while nobody has written it.
   */
  tf2::Transform getOdomToBaseLinkTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::ODOM_TO_BASE);
  }

private:
  PluginWrapper * plugin_wrapper_;
};

}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_INTERFACE_HPP_
