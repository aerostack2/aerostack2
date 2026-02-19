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
* @file ekf_utils.cpp
*
* Utility functions for EKF fusion implementation
*
* @authors Rodrigo da Silva
*/

#include "ekf_fuse/ekf_fuse_utils.hpp"

#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>

namespace ekf_fuse
{

// Normalize an angle to [-pi, pi]
double EKFFuseUtils::normalizeAngle(double angle)
{
  // Use fmod to wrap angle to [-2*pi, 2*pi]
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);

  // Adjust to [-pi, pi]
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }

  return angle - M_PI;
}

/**
 * @brief Blends two poses using spherical linear interpolation (slerp) for orientation and linear interpolation for position.
 * @param prev The previous pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
 * @param next The next pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
 * @param alpha The blending factor in [0, 1], where 0 returns 'next' and 1 returns 'prev'
 * @return The blended pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
 */
Eigen::Vector<double, 7> EKFFuseUtils::blendPoses(
  const Eigen::Vector<double, 7> & prev,
  const Eigen::Vector<double, 7> & next,
  double alpha)
{
  // Clamp in case caller passes something slightly outside [0,1]
  alpha = std::clamp(alpha, 0.0, 1.0);
  const double beta = 1.0 - alpha;

  // --- translation: simple lerp
  Eigen::Vector3d p = alpha * prev.head<3>() + beta * next.head<3>();

  // --- orientation: slerp in quaternion space (shortest path)
  // Input order is [qx,qy,qz,qw], but Eigen::Quaterniond ctor takes (w,x,y,z)
  Eigen::Quaterniond q_prev(prev[6], prev[3], prev[4], prev[5]);
  Eigen::Quaterniond q_next(next[6], next[3], next[4], next[5]);
  q_prev.normalize();
  q_next.normalize();

  // Our formula weighs 'prev' by alpha and 'next' by (1 - alpha),
  // so we slerp from prev -> next with parameter (1 - alpha).
  Eigen::Quaterniond q = q_prev.slerp(beta, q_next);
  q.normalize();  // keep it tidy

  // Pack result back into [px,py,pz, qx,qy,qz,qw]
  Eigen::Vector<double, 7> out;
  out << p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w();
  return out;
}

std::pair<Eigen::Vector<double, 7>, double> EKFFuseUtils::blendPosesWithRatio(
  const Eigen::Vector<double, 7> & prev,
  const Eigen::Vector<double, 7> & next,
  double max_step_position)
{
  // --- translation: compute distance and step
  Eigen::Vector3d p_prev = prev.head<3>();
  Eigen::Vector3d p_next = next.head<3>();
  Eigen::Vector3d delta_p = p_next - p_prev;
  double distance = delta_p.norm();

  // Calculate step size and ratio based on distance
  double position_ratio = 0.0;
  Eigen::Vector3d p;
  if (distance > 1e-6) {
    // Calculate how much we move towards target
    double step_size = std::min(distance, max_step_position);
    position_ratio = step_size / distance;  // ratio of movement
    p = p_prev + delta_p * position_ratio;
  } else {
    // Already at target
    p = p_next;
    position_ratio = 1.0;
  }


  // --- orientation: slerp with adaptive parameter based on angular distance
  // Input order is [qx,qy,qz,qw], but Eigen::Quaterniond ctor takes (w,x,y,z)
  Eigen::Quaterniond q_prev(prev[6], prev[3], prev[4], prev[5]);

  Eigen::Quaterniond q_next(next[6], next[3], next[4], next[5]);

  // Transform to euler
  q_prev.normalize();
  Eigen::Vector3d euler_prev = EKFFuseUtils::quaternionToEuler(q_prev);

  q_next.normalize();
  Eigen::Vector3d euler_next = EKFFuseUtils::quaternionToEuler(q_next);

  double roll = euler_prev[0];
  double pitch = euler_prev[1];
  double yaw_prev = euler_prev[2];
  double yaw_next = euler_next[2];
  double delta_yaw = yaw_next - yaw_prev;

  double yaw = yaw_prev + delta_yaw * position_ratio;

  // Transform back to quaternion
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  q.normalize();

  // Eigen::Quaterniond q = q_prev.slerp(position_ratio, q_next);
  // q.normalize();

  // Pack result back into [px,py,pz, qx,qy,qz,qw]

  Eigen::Vector<double, 7> out;

  out << p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w();

  // Return the position ratio as the overall blend ratio
  // (velocity should follow position movement)
  return {out, position_ratio};
}

/**
 * @brief Blends two twists with a maximum step size for velocity changes.
 * @param prev The previous twist as a 3D vector [vx, vy, vz]
 * @param next The next twist as a 3D vector [vx, vy, vz]
 * @param max_step_velocity The maximum allowed step size for velocity changes
 * @return The blended twist as a 3D vector [vx, vy, vz]
 */
Eigen::Vector<double, 3> EKFFuseUtils::blendTwistsWithRatio(
  const Eigen::Vector<double, 3> & prev,
  const Eigen::Vector<double, 3> & next,
  double blend_ratio)
{
  // Clamp ratio to [0, 1] for safety
  blend_ratio = std::clamp(blend_ratio, 0.0, 1.0);

  // Interpolate using the same ratio as position
  Eigen::Vector<double, 3> out = prev + (next - prev) * blend_ratio;

  return out;
}

/**
 * @brief Blends two twists using linear interpolation.
 * @param prev The previous twist as a 3D vector [vx, vy, vz]
 * @param next The next twist as a 3D vector [vx, vy, vz]
 * @param alpha The blending factor in [0, 1], where 0 returns 'next' and 1 returns 'prev'
 * @return The blended twist as a 3D vector [vx, vy, vz]
 */
Eigen::Vector<double, 3> EKFFuseUtils::blendTwists(
  const Eigen::Vector<double, 3> & prev,
  const Eigen::Vector<double, 3> & next,
  double alpha)
{
  // Clamp in case caller passes something slightly outside [0,1]
  alpha = std::clamp(alpha, 0.0, 1.0);
  const double beta = 1.0 - alpha;

  Eigen::Vector<double, 3> out = alpha * prev + beta * next;
  return out;
}

/**
 * @brief Corrects measured Euler angles to be closest to the state Euler angles, accounting for angle wrapping.
 * @param state_euler The current state Euler angles as a 3D vector [roll, pitch, yaw]
 * @param measured_euler The measured Euler angles as a 3D vector [roll, pitch, yaw]
 * @return The corrected measured Euler angles as a 3D vector [roll, pitch, yaw]
 */
Eigen::Vector3d EKFFuseUtils::correct_measured_euler(
  const Eigen::Vector3d & state_euler,
  const Eigen::Vector3d & measured_euler)
{
  Eigen::Vector3d corrected_euler = measured_euler;
  for (size_t i = 0; i < 3; i++) {
    double diff = std::abs(state_euler[i] - measured_euler[i]);
    double diff_2 = 0.0;
    if (measured_euler[i] >= 0.0) {
      diff_2 = state_euler[i] - (measured_euler[i] - 2.0 * M_PI);
    } else {
      diff_2 = state_euler[i] - (measured_euler[i] + 2.0 * M_PI);
    }
    diff_2 = std::abs(diff_2);
    if (diff_2 < diff) {
      if (measured_euler[i] >= 0.0) {
        corrected_euler[i] = measured_euler[i] - 2.0 * M_PI;
      } else {
        corrected_euler[i] = measured_euler[i] + 2.0 * M_PI;
      }
    }
  }
  return corrected_euler;
}

/**
 * @brief Converts a quaternion to Euler angles (roll, pitch, yaw) in XYZ order.
 * @param q_in The input quaternion as Eigen::Quaterniond
 * @return The Euler angles as a 3D vector [roll, pitch, yaw] in radians
 */
Eigen::Vector3d EKFFuseUtils::quaternionToEuler(
  const Eigen::Quaterniond & q_in)
{
  // Normalize to ensure a valid rotation
  Eigen::Quaterniond q = q_in.normalized();

  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  // Roll (x-axis rotation)
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

  // Yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief Converts a geometry_msgs::TransformStamped message to a 4x4 Eigen transformation matrix.
 * @param tf_msg The input TransformStamped message
 * @return The corresponding 4x4 Eigen transformation matrix
 */
Eigen::Matrix4d EKFFuseUtils::transformStampedToEigen(
  const geometry_msgs::msg::TransformStamped & tf_msg)
{
  const auto & t = tf_msg.transform.translation;
  const auto & q = tf_msg.transform.rotation;

  // Eigen quaternion is (w, x, y, z)
  Eigen::Quaterniond q_eig(q.w, q.x, q.y, q.z);
  q_eig.normalize();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q_eig.toRotationMatrix();
  T(0, 3) = t.x;
  T(1, 3) = t.y;
  T(2, 3) = t.z;
  return T;
}

/**
 * @brief Converts an internal EKF pose measurement to an external pose measurement using the global map to odom transformation.
 * @param internal_pose The internal EKF pose measurement
 * @param global_map_to_odom The global map to odom transformation as a 7D vector [px, py, pz, qx, qy, qz, qw]
 * @param ekf_wrapper The EKFWrapper instance to use for transformations
 * @return The external pose measurement
 */
ekf::PoseMeasurement EKFFuseUtils::getExternalStatePoseFromInternalState(
  const ekf::PoseMeasurement & internal_pose,
  const Eigen::Vector<double, 7> global_map_to_odom,
  ekf::EKFWrapper & ekf_wrapper)
{
  ekf::PoseMeasurement external_pose;
  // Get map to odom
  Eigen::Matrix4d map_to_odom = ekf_wrapper.get_map_to_odom();
  // Get internal pose euler and position
  Eigen::Vector3d pos_internal(
    internal_pose.data[ekf::PoseMeasurement::X],
    internal_pose.data[ekf::PoseMeasurement::Y],
    internal_pose.data[ekf::PoseMeasurement::Z]);
  Eigen::Vector3d euler_internal(
    internal_pose.data[ekf::PoseMeasurement::ROLL],
    internal_pose.data[ekf::PoseMeasurement::PITCH],
    internal_pose.data[ekf::PoseMeasurement::YAW]);
  // Internal pose is map to base, we have to get only odom to base
  Eigen::Matrix4d T_odom_to_base_internal = ekf_wrapper.get_T_b_c(
    pos_internal,
    euler_internal,
    map_to_odom
  );

  // Get T_global_map_to_odom
  Eigen::Vector3d pos_global_map_odom(
    global_map_to_odom[0],
    global_map_to_odom[1],
    global_map_to_odom[2]);
  Eigen::Vector3d euler_global_map_odom = quaternionToEuler(
    Eigen::Quaterniond(
      global_map_to_odom[6],
      global_map_to_odom[3],
      global_map_to_odom[4],
      global_map_to_odom[5]
    )
  );
  Eigen::Matrix4d T_global_map_to_odom = ekf_wrapper.get_T_b_c(
    pos_global_map_odom,
    euler_global_map_odom,
    Eigen::Matrix4d::Identity()
  );
  // Apply global map to odom transform
  Eigen::Matrix4d T_global_map_to_base = T_global_map_to_odom * T_odom_to_base_internal;

  Eigen::Vector<double, 7> external_pose_vector =
    ekf::EKFWrapper::transform_to_pose(T_global_map_to_base);

  Eigen::Vector3d euler_external = quaternionToEuler(
    Eigen::Quaterniond(
      external_pose_vector[6],
      external_pose_vector[3],
      external_pose_vector[4],
      external_pose_vector[5]
    )
  );

  external_pose.data[ekf::PoseMeasurement::X] = external_pose_vector[0];
  external_pose.data[ekf::PoseMeasurement::Y] = external_pose_vector[1];
  external_pose.data[ekf::PoseMeasurement::Z] = external_pose_vector[2];
  external_pose.data[ekf::PoseMeasurement::ROLL] = euler_external[0];
  external_pose.data[ekf::PoseMeasurement::PITCH] = euler_external[1];
  external_pose.data[ekf::PoseMeasurement::YAW] = euler_external[2];

  return external_pose;
}

/**
   * @brief Computes the time difference in seconds between two std_msgs::msg::Header messages.
   * @param header1 The first header message
   * @param header2 The second header message
   * @return The time difference in seconds (header2 - header1)
   */
double EKFFuseUtils::compute_time_difference(
  const std_msgs::msg::Header & header1,
  const std_msgs::msg::Header & header2)
{
  double time1 = header1.stamp.sec + header1.stamp.nanosec * 1e-9;
  double time2 = header2.stamp.sec + header2.stamp.nanosec * 1e-9;

  return time2 - time1;
}

/**
   * @brief Fuses multiple pose measurements with their corresponding covariances into a single pose measurement and covariance.
   * @param accumulated_poses_ A vector of pose measurements to fuse
   * @param accumulated_poses_covariances_ A vector of corresponding pose measurement covariances
   * @return A pair containing the fused pose measurement and its covariance
   */
std::pair<ekf::PoseMeasurement,
  ekf::PoseMeasurementCovariance> EKFFuseUtils::fusePoseWithCovariance(
  const std::vector<ekf::PoseMeasurement> & accumulated_poses_,
  const std::vector<ekf::PoseMeasurementCovariance> & accumulated_poses_covariances_)
{
  ekf::PoseMeasurement fused_pose;
  ekf::PoseMeasurementCovariance fused_covariance;

  if (accumulated_poses_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("pose_fusion"), "Empty poses array, returning empty pose");
    return {fused_pose, fused_covariance};
  }

  // Verify that both vectors have the same size
  if (accumulated_poses_.size() != accumulated_poses_covariances_.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("pose_fusion"),
      "Poses and covariances vectors have different sizes");
    return {fused_pose, fused_covariance};
  }

  // If only one pose, return it directly
  if (accumulated_poses_.size() == 1) {
    return {accumulated_poses_[0], accumulated_poses_covariances_[0]};
  }

  // ============= POSITION FUSION (3D) =============
  Eigen::Matrix3d info_pos = Eigen::Matrix3d::Zero();
  Eigen::Vector3d info_weighted_pos = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < accumulated_poses_.size(); ++i) {
    const auto & pose = accumulated_poses_[i];
    const auto & cov = accumulated_poses_covariances_[i];

    // Extract position
    Eigen::Vector3d pos(
      pose.data[ekf::PoseMeasurement::X],
      pose.data[ekf::PoseMeasurement::Y],
      pose.data[ekf::PoseMeasurement::Z]);

    // Build diagonal position covariance matrix (3x3)
    Eigen::Matrix3d P_pos = Eigen::Matrix3d::Zero();
    P_pos(0, 0) = cov.data[ekf::PoseMeasurementCovariance::X];
    P_pos(1, 1) = cov.data[ekf::PoseMeasurementCovariance::Y];
    P_pos(2, 2) = cov.data[ekf::PoseMeasurementCovariance::Z];

    // Regularization
    P_pos += Eigen::Matrix3d::Identity() * 1e-9;

    // Information matrix
    Eigen::Matrix3d info_matrix = P_pos.inverse();
    info_pos += info_matrix;
    info_weighted_pos += info_matrix * pos;
  }

  // Fused position and covariance
  Eigen::Matrix3d P_fused_pos = info_pos.inverse();
  Eigen::Vector3d x_fused_pos = P_fused_pos * info_weighted_pos;

  // ============= YAW FUSION =============
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  double sum_weight = 0.0;

  for (size_t i = 0; i < accumulated_poses_.size(); ++i) {
    const auto & pose = accumulated_poses_[i];
    const auto & cov = accumulated_poses_covariances_[i];

    // Extract yaw
    double yaw = pose.data[ekf::PoseMeasurement::YAW];

    // Extract yaw covariance
    double yaw_variance = cov.data[ekf::PoseMeasurementCovariance::YAW];
    yaw_variance = std::max(yaw_variance, 1e-9);    // Regularization

    // Weight inversely proportional to variance
    double weight = 1.0 / yaw_variance;

    // Weighted circular average
    sum_cos += weight * std::cos(yaw);
    sum_sin += weight * std::sin(yaw);
    sum_weight += weight;
  }

  // Fused yaw using atan2 for angle wrapping
  double fused_yaw = std::atan2(sum_sin / sum_weight, sum_cos / sum_weight);

  // Fused yaw covariance (inverse of sum of weights)
  double fused_yaw_variance = 1.0 / sum_weight;

  // ============= BUILD FINAL RESULT =============

  // Fused position
  fused_pose.data[ekf::PoseMeasurement::X] = x_fused_pos.x();
  fused_pose.data[ekf::PoseMeasurement::Y] = x_fused_pos.y();
  fused_pose.data[ekf::PoseMeasurement::Z] = x_fused_pos.z();

  // Fused orientation (only yaw, roll and pitch set to 0)
  fused_pose.data[ekf::PoseMeasurement::ROLL] = 0.0;
  fused_pose.data[ekf::PoseMeasurement::PITCH] = 0.0;
  fused_pose.data[ekf::PoseMeasurement::YAW] = fused_yaw;

  // Fused covariance (diagonal elements)
  fused_covariance.data[ekf::PoseMeasurementCovariance::X] = P_fused_pos(0, 0);
  fused_covariance.data[ekf::PoseMeasurementCovariance::Y] = P_fused_pos(1, 1);
  fused_covariance.data[ekf::PoseMeasurementCovariance::Z] = P_fused_pos(2, 2);
  fused_covariance.data[ekf::PoseMeasurementCovariance::ROLL] = fused_yaw_variance;    // Only yaw
  fused_covariance.data[ekf::PoseMeasurementCovariance::PITCH] = fused_yaw_variance;    // Only yaw
  fused_covariance.data[ekf::PoseMeasurementCovariance::YAW] = fused_yaw_variance;

  RCLCPP_INFO(
    rclcpp::get_logger("pose_fusion"),
    "Fused %zu poses: pos=[%.3f, %.3f, %.3f], yaw=%.3f rad, "
    "pos_uncertainty=%.6f, yaw_uncertainty=%.6f",
    accumulated_poses_.size(),
    x_fused_pos.x(), x_fused_pos.y(), x_fused_pos.z(),
    fused_yaw,
    P_fused_pos.trace(), fused_yaw_variance);

  return {fused_pose, fused_covariance};
}

}  // namespace ekf_fuse
