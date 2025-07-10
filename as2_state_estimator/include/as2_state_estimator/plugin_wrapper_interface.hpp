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
  explicit PluginWrapperInterface(
    PluginWrapper * plugin_wrapper)
  : plugin_wrapper_(plugin_wrapper)
  {}

  inline const std::string & getEarthFrame() override
  {
    return StateEstimator::getEarthFrame();
  }
  inline const std::string & getMapFrame() override
  {
    return StateEstimator::getMapFrame();
  }
  inline const std::string & getOdomFrame() override
  {
    return StateEstimator::getOdomFrame();
  }
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
      plugin_wrapper_->plugin_name_, pose, TransformInformatonType::EARTH_TO_MAP, stamp);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::EARTH_TO_MAP);
  }

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
      plugin_wrapper_->plugin_name_, pose, TransformInformatonType::MAP_TO_ODOM, stamp);
    plugin_wrapper_->advertiseUpdate(TransformInformatonType::MAP_TO_ODOM);
  }
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
  tf2::Transform getEarthToMapTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::EARTH_TO_MAP);
  }
  tf2::Transform getMapToOdomTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::MAP_TO_ODOM);
  }
  tf2::Transform getOdomToBaseLinkTransform() override
  {
    return plugin_wrapper_->robot_state_.getTransform(TransformInformatonType::ODOM_TO_BASE);
  }

private:
  PluginWrapper * plugin_wrapper_;
};

}  // namespace as2_state_estimator

#endif  // AS2_STATE_ESTIMATOR__PLUGIN_WRAPPER_INTERFACE_HPP_
