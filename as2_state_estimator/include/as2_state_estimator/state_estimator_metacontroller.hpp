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
    StateEstimator * state_estimator)
  : state_estimator_(state_estimator) {}

  const std::string & getEarthFrame() override {return state_estimator_->earth_frame_id_;}
  const std::string & getMapFrame() override {return state_estimator_->map_frame_id_;}
  const std::string & getOdomFrame() override {return state_estimator_->odom_frame_id_;}
  const std::string & getBaseFrame() override {return state_estimator_->base_frame_id_;}

  /**
   * @brief Set the pose of the map frame (local for each robot) in the earth frame (global)
   * @param pose The pose of the map frame in the earth frame with covariance
   */
  virtual void setEarthToMap(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false)
  {

    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = state_estimator_->earth_frame_id_;
    state_estimator_->processEarthToMap(

    )


  }
  virtual void setEarthToMap(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;

  /**
   * @brief Set the pose of the robot from the map frame to the odom frame
   * @param pose The pose of the robot from the map frame to the odom frame with covariance
   */
  virtual void setMapToOdomPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  virtual void setMapToOdomPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  /**
   * @brief Set the pose of the robot from the odom frame to the base_link frame
   * @param pose The pose of the robot from the odom frame to the base_link frame with covariance
   */
  virtual void setOdomToBaseLinkPose(
    const geometry_msgs::msg::PoseWithCovariance & pose,
    const builtin_interfaces::msg::Time & stamp, bool is_static = false) = 0;
  virtual void setOdomToBaseLinkPose(
    const tf2::Transform & pose,
    const builtin_interfaces::msg::Time & stamp,
    bool is_static = false) = 0;
  /**
   * @brief Set the twist of the robot in the base_link frame
   * @param twist The twist of the robot in the base_link frame with covariance
   */
  virtual void setTwistInBaseFrame(
    const geometry_msgs::msg::TwistWithCovariance & twist,
    const builtin_interfaces::msg::Time & stamp) = 0;

  virtual tf2::Transform getEarthToMapTransform() = 0;
  virtual tf2::Transform getMapToOdomTransform() = 0;
  virtual tf2::Transform getOdomToBaseLinkTransform() = 0;

private:
  StateEstimator * state_estimator_;


};

}  // namespace as2_state_estimator
#endif  // AS2_STATE_ESTIMATOR__STATE_ESTIMATION_META_CONTROLLER_HPP_
