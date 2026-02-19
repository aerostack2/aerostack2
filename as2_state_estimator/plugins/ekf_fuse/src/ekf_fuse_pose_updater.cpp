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
* @file pose_updater.cpp
*
* Pose update handler for EKF fusion implementation
*
* @authors Rodrigo da Silva
*/

#include "ekf_fuse/ekf_fuse_pose_updater.hpp"

#include <utility>
#include <string>
#include <iostream>
#include <tuple>
#include <vector>

namespace ekf_fuse
{

// Constructor
EKFFusePoseUpdater::EKFFusePoseUpdater(
  ekf::EKFWrapper & ekf_wrapper,
  as2::Node * node_ptr)
{
  ekf_wrapper_ = ekf_wrapper;
  T_earth_to_map_ = Eigen::Matrix4d::Identity();
  node_ptr_ = node_ptr;

  ransac_iterations_ = ekf_fuse::getParameter<int>(node_ptr_, "ekf_fuse.ransac.iterations");
  ransac_pos_thresh_ =
    ekf_fuse::getParameter<double>(node_ptr_, "ekf_fuse.ransac.ransac_pos_thresh");
  ransac_ori_thresh_ =
    ekf_fuse::getParameter<double>(node_ptr_, "ekf_fuse.ransac.ransac_ori_thresh");

  ransac_filter_.configure(ransac_iterations_, ransac_pos_thresh_, ransac_ori_thresh_);
}

/**
 * @brief Prepares the initial pose for EKF initialization
 * @param pose_msg The pose with covariance message
 * @return The initial pose as a 4x4 transformation matrix and its covariance diagonal
 */
std::pair<Eigen::Matrix4d, std::array<double, 6>> EKFFusePoseUpdater::initialPreparePose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
{
  ekf::PoseMeasurement pose_meas;
  Eigen::Quaterniond q(
    pose_msg->pose.pose.orientation.w,
    pose_msg->pose.pose.orientation.x,
    pose_msg->pose.pose.orientation.y,
    pose_msg->pose.pose.orientation.z);
  q.normalize();
  auto euler = EKFFuseUtils::quaternionToEuler(q);
  Eigen::Vector3d pos_frame_base = Eigen::Vector3d(
    pose_msg->pose.pose.position.x,
    pose_msg->pose.pose.position.y,
    pose_msg->pose.pose.position.z);
  Eigen::Vector3d ori_frame_base = Eigen::Vector3d(
    euler[0],
    euler[1],
    euler[2]);
  std::array<double, 6> pose_covariance_diagonal;
  for (size_t i = 0; i < 6; i++) {
    pose_covariance_diagonal[i] = pose_msg->pose.covariance[i * 6 + i];
  }
  Eigen::Matrix4d corr_T_map_oldBase = Eigen::Matrix4d::Identity();
  // Check frame id of pose
  if (pose_msg->header.frame_id == "earth") {
    // Transform from earth-base_link to map-base_link eigen matrix
    corr_T_map_oldBase = ekf_wrapper_.get_T_b_c(
      pos_frame_base,
      ori_frame_base,
      T_earth_to_map_);
  } else {
    // Transform from map-base_link to map-base_link eigen matrix
    corr_T_map_oldBase = ekf_wrapper_.get_T_b_c(
      pos_frame_base,
      ori_frame_base,
      Eigen::Matrix4d::Identity());
  }

  return {corr_T_map_oldBase, pose_covariance_diagonal};
}

/**
 * @brief Get movement increment from image delay as a transformation matrix
 * @param msg The pose message
 * @param state The current EKF state
 * @return The movement increment as a 4x4 transformation matrix
 */
Eigen::Matrix4d EKFFusePoseUpdater::getMovementIncrementFromImageDelay(
  std_msgs::msg::Header msg_header,
  const ekf::State & state)
{
  // Get transform from map to base_link in msg stamp
  std::string target_frame = "drone0/map";
  std::string source_frame = "drone0/base_link";
  rclcpp::Time desired_time = rclcpp::Time(msg_header.stamp);
  geometry_msgs::msg::TransformStamped tf_msg = geometry_msgs::msg::TransformStamped();
  try {
    if (tf_buffer_ == nullptr) {
      std::cout << "TF Buffer is null!" << std::endl;
      return Eigen::Matrix4d::Identity();
    }
    tf_msg = tf_buffer_->lookupTransform(
      target_frame,                    // target
      source_frame,                    // source
      desired_time,                    // time to query
      rclcpp::Duration::from_seconds(0.001)       // timeout
    );
  } catch (const std::exception & e) {
    std::cout << "TF lookup failed: " << e.what() << std::endl;
    return Eigen::Matrix4d::Identity();
  }

  Eigen::Matrix4d T_map_oldBase = EKFFuseUtils::transformStampedToEigen(tf_msg);

  // Get T_map_lastBase
  Eigen::Matrix4d T_map_lastBase =
    ekf_wrapper_.get_T_b_c(
    Eigen::Vector3d(state.get_position().data()),
    Eigen::Vector3d(state.get_orientation().data()),
    Eigen::Matrix4d::Identity());

  // Get T_oldBase_lastBase (increment)
  Eigen::Matrix4d T_oldBase_lastBase = T_map_oldBase.inverse() * T_map_lastBase;

  return T_oldBase_lastBase;
}

ekf::PoseMeasurement EKFFusePoseUpdater::preparePoseMeasurement(
  Eigen::Matrix4d T_map_base,
  const ekf::State & state,
  bool roll_pitch_fixed)
{
  Eigen::Vector<double, 7> pose_map_base = ekf::EKFWrapper::transform_to_pose(T_map_base);

  Eigen::Vector3d euler_map_base = EKFFuseUtils::quaternionToEuler(
    Eigen::Quaterniond(
      pose_map_base[6],
      pose_map_base[3],
      pose_map_base[4],
      pose_map_base[5]));

  // Correct angles to be near the current state
  std::array<double, 3> state_euler_array = {
    state.data[ekf::State::ROLL],
    state.data[ekf::State::PITCH],
    state.data[ekf::State::YAW]
  };
  Eigen::Vector3d corrected_euler_map_base = EKFFuseUtils::correct_measured_euler(
    Eigen::Vector3d(
      state_euler_array[0],
      state_euler_array[1],
      state_euler_array[2]),
    euler_map_base);

  // Get current state roll and pitch
  double current_roll = state.data[ekf::State::ROLL];
  double current_pitch = state.data[ekf::State::PITCH];
  if (!roll_pitch_fixed) {
    current_roll = corrected_euler_map_base[0];
    current_pitch = corrected_euler_map_base[1];
  }

  std::array<double, ekf::PoseMeasurement::size> pose_values = {
    pose_map_base[0],
    pose_map_base[1],
    pose_map_base[2],
    current_roll,     // roll
    current_pitch,     // pitch
    corrected_euler_map_base[2]     // yaw
  };

  return ekf::PoseMeasurement(pose_values);
}

/**
 * @brief Processes a pose measurement and its covariance
 * @param pose_meas The pose measurement
 * @param pose_cov The pose measurement covariance
 * @param state The current EKF state
 * @return A tuple indicating if it should update the EKF, the processed pose measurement, and its covariance
 */
std::tuple<bool, ekf::PoseMeasurement,
  ekf::PoseMeasurementCovariance> EKFFusePoseUpdater::processPoseMeasurement(
  ekf::PoseMeasurement pose_meas,
  ekf::PoseMeasurementCovariance pose_cov,
  std_msgs::msg::Header pose_header,
  const ekf::State & state)
{
  ekf::PoseMeasurement final_pose_meas = pose_meas;
  ekf::PoseMeasurementCovariance final_pose_cov = pose_cov;
  bool update_with_accumulated = true;
  if (pose_accumulation_type_ != "") {    // ----------------------------- Update with accumulation
    update_with_accumulated = false;
    if (pose_accumulation_type_ == "number") {      // Number accumulation
      // Get current state pose
      ekf::PoseMeasurement current_pose;
      current_pose.data[0] = state.data[ekf::State::X];
      current_pose.data[1] = state.data[ekf::State::Y];
      current_pose.data[2] = state.data[ekf::State::Z];
      current_pose.data[3] = state.data[ekf::State::ROLL];
      current_pose.data[4] = state.data[ekf::State::PITCH];
      current_pose.data[5] = state.data[ekf::State::YAW];
      // current_pose = getExternalStatePoseFromInternalState(current_pose);
      // Compute difference
      ekf::PoseMeasurement pose_difference;
      pose_difference.data[0] = pose_meas.data[0] - current_pose.data[0];
      pose_difference.data[1] = pose_meas.data[1] - current_pose.data[1];
      pose_difference.data[2] = pose_meas.data[2] - current_pose.data[2];
      pose_difference.data[3] = 0.0;
      pose_difference.data[4] = 0.0;
      pose_difference.data[5] = pose_meas.data[5] - current_pose.data[5];
      // Accumulate in buffer
      accumulated_poses_.push_back(pose_difference);
      accumulated_poses_covariances_.push_back(pose_cov);
      accumulated_poses_headers_.push_back(pose_header);

      // Check if we have enough poses to accumulate
      if (accumulated_poses_.size() >= pose_accumulation_number_) {
        update_with_accumulated = true;
      }
    } else {      // ------------------------------------------------------------ Time accumulation
      // Get current state pose
      ekf::PoseMeasurement current_pose;
      current_pose.data[0] = state.data[ekf::State::X];
      current_pose.data[1] = state.data[ekf::State::Y];
      current_pose.data[2] = state.data[ekf::State::Z];
      current_pose.data[3] = state.data[ekf::State::ROLL];
      current_pose.data[4] = state.data[ekf::State::PITCH];
      current_pose.data[5] = state.data[ekf::State::YAW];
      // current_pose = getExternalStatePoseFromInternalState(current_pose);
      // Compute difference
      ekf::PoseMeasurement pose_difference;
      pose_difference.data[0] = pose_meas.data[0] - current_pose.data[0];
      pose_difference.data[1] = pose_meas.data[1] - current_pose.data[1];
      pose_difference.data[2] = pose_meas.data[2] - current_pose.data[2];
      pose_difference.data[3] = 0.0;
      pose_difference.data[4] = 0.0;
      pose_difference.data[5] = pose_meas.data[5] - current_pose.data[5];
      // Accumulate in buffer
      accumulated_poses_.push_back(pose_difference);
      accumulated_poses_covariances_.push_back(pose_cov);
      accumulated_poses_headers_.push_back(pose_header);

      // Check if we have enough time to accumulate
      std_msgs::msg::Header first_header = accumulated_poses_headers_.front();
      std_msgs::msg::Header last_header = accumulated_poses_headers_.back();
      double time_difference = EKFFuseUtils::compute_time_difference(first_header, last_header);
      if (time_difference >= pose_accumulation_time_) {
        update_with_accumulated = true;
      }
    }
    if (update_with_accumulated) {
      // Accumulate poses
      ekf::PoseMeasurement accumulated_pose;
      ekf::PoseMeasurementCovariance accumulated_cov;

      if (ransac_iterations_ > 0) {
        auto t0 = std::chrono::steady_clock::now();

        // Filter outliers using ransac
        RCLCPP_INFO(
          node_ptr_->get_logger(), "  [RANSAC] Accumulated poses: %li",
          accumulated_poses_.size());
        std::vector<int> inliers_idx = ransac_filter_.getInliers(accumulated_poses_);
        ransac_filter_.removeOutliers(
          accumulated_poses_, accumulated_poses_covariances_,
          accumulated_poses_headers_, inliers_idx);
        RCLCPP_INFO(
          node_ptr_->get_logger(), "  [RANSAC] Poses after filtering: %li",
          accumulated_poses_.size());

        auto t1 = std::chrono::steady_clock::now();
        auto dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        RCLCPP_INFO(node_ptr_->get_logger(), "  [RANSAC] Filtering time: %.3f ms", dt_ms);
      }

      auto accumulated_result = EKFFuseUtils::fusePoseWithCovariance(
        accumulated_poses_,
        accumulated_poses_covariances_);
      accumulated_pose = accumulated_result.first;
      accumulated_cov = accumulated_result.second;

      // Add the current state to the accumulated pose error
      accumulated_pose.data[0] += state.data[ekf::State::X];
      accumulated_pose.data[1] += state.data[ekf::State::Y];
      accumulated_pose.data[2] += state.data[ekf::State::Z];
      accumulated_pose.data[3] = state.data[ekf::State::ROLL];
      accumulated_pose.data[4] = state.data[ekf::State::PITCH];
      accumulated_pose.data[5] += state.data[ekf::State::YAW];

      // Update with accumulated pose
      final_pose_meas = accumulated_pose;
      final_pose_cov = accumulated_cov;
    }
  }
  return {update_with_accumulated, final_pose_meas, final_pose_cov};
}

void EKFFusePoseUpdater::clearAccumulatedPoses()
{
  accumulated_poses_.clear();
  accumulated_poses_covariances_.clear();
  accumulated_poses_headers_.clear();
}

}  // namespace ekf_fuse
