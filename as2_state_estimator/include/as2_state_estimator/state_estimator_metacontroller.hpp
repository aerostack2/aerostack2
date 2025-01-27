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
* @file state_estimation_meta_controller.hpp
*
* An state estimation meta controller for AeroStack2 State Estimation
*
* @authors Miguel Fernández Cortizas
*/

#ifndef AS2_STATE_ESTIMATOR__STATE_ESTIMATION_META_CONTROLLER_HPP_
#define AS2_STATE_ESTIMATOR__STATE_ESTIMATION_META_CONTROLLER_HPP_

#include <string>

#include "as2_state_estimator/as2_state_estimator.hpp"
#include "as2_state_estimator/plugin_base.hpp"

namespace as2_state_estimator
{

class MetacontrollerInterface : public StateEstimatorInterface
{
public:
  explicit MetacontrollerInterface(
    StateEstimator * state_estimator, const std::string & plugin_name)
  : state_estimator_(state_estimator), plugin_name_(plugin_name)
  {}

  const std::string & getEarthFrame() override {return state_estimator_->earth_frame_id_;}
  const std::string & getMapFrame() override {return state_estimator_->map_frame_id_;}
  const std::string & getOdomFrame() override {return state_estimator_->odom_frame_id_;}
  const std::string & getBaseFrame() override {return state_estimator_->base_frame_id_;}

  /**
   * @brief Set the pose of the map frame (local for each robot) in the earth frame (global)
   * @param pose The pose of the map frame in the earth frame with covariance
   */
  void setEarthToMap(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) override
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = state_estimator_->earth_frame_id_;
    state_estimator_->processEarthToMap(plugin_name_, pose_stamped, is_static);
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
    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = state_estimator_->map_frame_id_;
    state_estimator_->processMapToOdom(plugin_name_, pose_stamped, is_static);
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
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) override
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = state_estimator_->odom_frame_id_;
    state_estimator_->processOdomToBase(plugin_name_, pose_stamped, is_static);
  }
  void setOdomToBaseLinkPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) override
  {
    geometry_msgs::msg::PoseWithCovariance pose_msg;
    tf2::toMsg(pose, pose_msg.pose);
    setOdomToBaseLinkPose(pose_msg, stamp, is_static);
  }
  /**
   * @brief Set the twist of the robot in the base_link frame
   * @param twist The twist of the robot in the base_link frame with covariance
   */
  void setTwistInBaseFrame(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp) override
  {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_stamped;
    twist_stamped.twist = twist;
    twist_stamped.header.stamp = stamp;
    twist_stamped.header.frame_id = state_estimator_->base_frame_id_;
    state_estimator_->processTwist(plugin_name_, twist_stamped);
  }

  tf2::Transform getEarthToMapTransform() override
  {
    return state_estimator_->earth_to_map_;
  }
  tf2::Transform getMapToOdomTransform() override
  {
    return state_estimator_->map_to_odom_;
  }
  tf2::Transform getOdomToBaseLinkTransform() override
  {
    return state_estimator_->odom_to_base_;
  }

private:
  StateEstimator * state_estimator_;
  std::string plugin_name_;
};

}  // namespace as2_state_estimator
#endif  // AS2_STATE_ESTIMATOR__STATE_ESTIMATION_META_CONTROLLER_HPP_
