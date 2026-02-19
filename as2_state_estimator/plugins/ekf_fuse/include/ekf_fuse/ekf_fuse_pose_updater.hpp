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
* @file pose_updater.hpp
*
* Pose update handler for EKF fusion
*
* @authors Rodrigo da Silva
*/

#ifndef POSE_UPDATER_HPP_
#define POSE_UPDATER_HPP_

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>

#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

#include <array>
#include <vector>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <as2_msgs/msg/pose_with_covariance_stamped_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "ekf_fuse/ekf_fuse_utils.hpp"
#include "ekf_fuse/ekf_fuse_ransac_pose.hpp"

namespace ekf_fuse
{

/**
 * @class EKFFusePoseUpdater
 * @brief Handles pose measurements and updates for EKF fusion
 *
 * This class manages the reception and processing of pose measurements
 * from various sources, including accumulation strategies and fusion
 * of multiple pose measurements.
 */
class EKFFusePoseUpdater
{
public:
  EKFFusePoseUpdater() = default;

  /**
   * @brief Constructor
   * @param ekf_wrapper Reference to the EKF wrapper instance
   */
  EKFFusePoseUpdater(
    ekf::EKFWrapper & ekf_wrapper,
    as2::Node * node_ptr
  );

  /**
   * @brief Destructor
   */
  ~EKFFusePoseUpdater() = default;

  /**
   * @brief Sets the transformation matrix from earth to map frame
   * @param T_earth_to_map The transformation matrix
   */
  void setT_earth_to_map(
    const Eigen::Matrix4d & T_earth_to_map)
  {
    T_earth_to_map_ = T_earth_to_map;
  }

  /**
   * @brief Sets the TF buffer for pose transformations
   * @param tf_buffer The TF buffer
   */
  void setTFBuffer(tf2_ros::Buffer::SharedPtr tf_buffer)
  {
    tf_buffer_ = tf_buffer;
  }

  /**
   * @brief Sets the pose accumulation parameters
   * @param accumulation_type The type of accumulation ("", "time" or "number")
   * @param accumulation_time The time duration for accumulation (in seconds)
   * @param accumulation_number The number of poses to accumulate
   */
  void setPoseAccumulationParameters(
    const std::string & accumulation_type,
    double accumulation_time,
    int accumulation_number)
  {
    pose_accumulation_type_ = accumulation_type;
    pose_accumulation_time_ = accumulation_time;
    pose_accumulation_number_ = accumulation_number;
  }

  /**
   * @brief Prepares the initial pose for EKF initialization
   * @param pose_msg The pose with covariance message
   * @return The initial pose as a 4x4 transformation matrix and its covariance diagonal
   */
  std::pair<Eigen::Matrix4d, std::array<double, 6>> initialPreparePose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);

  /**
   * @brief Get movement increment from image delay as a transformation matrix
   * @param msg The pose message
   * @param state The current EKF state
   * @return The movement increment as a 4x4 transformation matrix
   */
  Eigen::Matrix4d getMovementIncrementFromImageDelay(
    std_msgs::msg::Header msg_header,
    const ekf::State & state);

  /**
   * @brief Prepares a pose measurement from a transformation matrix
   * @param T_map_base The transformation matrix from map to base_link
   * @param state The current EKF state
   * @param roll_pitch_fixed Whether to fix roll and pitch from the current state
   * @return The prepared pose measurement
   */
  ekf::PoseMeasurement preparePoseMeasurement(
    Eigen::Matrix4d T_map_base,
    const ekf::State & state,
    bool roll_pitch_fixed);

  /**
   * @brief Processes a pose measurement and its covariance
   * @param pose_meas The pose measurement
   * @param pose_cov The pose measurement covariance
   * @param state The current EKF state
   * @return A tuple indicating if it should update the EKF, the processed pose measurement, and its covariance
   */
  std::tuple<bool, ekf::PoseMeasurement, ekf::PoseMeasurementCovariance> processPoseMeasurement(
    ekf::PoseMeasurement pose_meas,
    ekf::PoseMeasurementCovariance pose_cov,
    std_msgs::msg::Header pose_header,
    const ekf::State & state);

  /**
   * @brief Clear accumulated poses and covariances
   */
  void clearAccumulatedPoses();

private:
  as2::Node * node_ptr_;
  ekf::EKFWrapper ekf_wrapper_;
  Eigen::Matrix4d T_earth_to_map_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  EKFFuseRansacPose ransac_filter_;

  std::vector<ekf::PoseMeasurement> accumulated_poses_;
  std::vector<ekf::PoseMeasurementCovariance> accumulated_poses_covariances_;
  std::vector<std_msgs::msg::Header> accumulated_poses_headers_;
  std::string pose_accumulation_type_ = "";  // "", "time" or "number"
  double pose_accumulation_time_ = 0.0;  // seconds
  int pose_accumulation_number_ = 0;  // number of accumulated poses

  int ransac_iterations_ = 0;
  double ransac_pos_thresh_ = 0.20;
  double ransac_ori_thresh_ = 8.0 * M_PI / 180.0;
};

}  // namespace ekf_fuse

#endif  // EKF_FUSE_POSE_UPDATER_HPP_
