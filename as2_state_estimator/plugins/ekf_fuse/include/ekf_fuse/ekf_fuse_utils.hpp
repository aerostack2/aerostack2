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
* @file ekf_utils.hpp
*
* Utility functions for EKF fusion
*
* @authors Rodrigo da Silva
*/

#ifndef EKF_FUSE_UTILS_HPP_
#define EKF_FUSE_UTILS_HPP_

#include <ekf/ekf_datatype.hpp>
#include <ekf/ekf_wrapper.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <utility>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <as2_msgs/msg/pose_with_covariance_stamped_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <as2_core/node.hpp>


namespace ekf_fuse
{

/**
 * @class EKFUtils
 * @brief Utility functions for EKF fusion operations
 *
 * This class provides static utility methods for common operations
 * in EKF fusion, such as coordinate transformations, covariance
 * calculations, and data conversions.
 */
class EKFFuseUtils
{
public:
  /**
   * @brief Deleted constructor - utility class with only static methods
   */
  EKFFuseUtils() = delete;

  /**
   * @brief Deleted destructor - utility class with only static methods
   */
  ~EKFFuseUtils() = delete;

  /**
   * @brief Example static method: normalize an angle to [-pi, pi]
   * @param angle The angle to normalize (in radians)
   * @return The normalized angle in the range [-pi, pi]
   */
  static double normalizeAngle(
    double angle);

  /**
   * @brief Blends two poses using spherical linear interpolation (slerp) for orientation and linear interpolation for position.
   * @param prev The previous pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
   * @param next The next pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
   * @param alpha The blending factor in [0, 1], where 0 returns 'next' and 1 returns 'prev'
   * @return The blended pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
   */
  static Eigen::Vector<double, 7> blendPoses(
    const Eigen::Vector<double, 7> & prev,
    const Eigen::Vector<double, 7> & next,
    double alpha);

  /**
   * @brief Blends two twists using linear interpolation.
   * @param prev The previous twist as a 3D vector [vx, vy, vz]
   * @param next The next twist as a 3D vector [vx, vy, vz]
   * @param alpha The blending factor in [0, 1], where 0 returns 'next' and 1 returns 'prev'
   * @return The blended twist as a 3D vector [vx, vy, vz]
   */
  static Eigen::Vector<double, 3> blendTwists(
    const Eigen::Vector<double, 3> & prev,
    const Eigen::Vector<double, 3> & next, double alpha);

  /**
   * @brief Blends two poses with a maximum step size for position changes.
   * @param prev The previous pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
   * @param next The next pose as a 7D vector [px, py, pz, qx, qy, qz, qw]
   * @param max_step_position The maximum allowed step size for position changes
   * @return A pair containing the blended pose as a 7D vector [px, py, pz, qx, qy, qz, qw] and the blending ratio used
   */
  static std::pair<Eigen::Vector<double, 7>, double> blendPosesWithRatio(
    const Eigen::Vector<double, 7> & prev,
    const Eigen::Vector<double, 7> & next,
    double max_step_position);

  /**
   * @brief Blends two twists with a maximum step size for velocity changes.
   * @param prev The previous twist as a 3D vector [vx, vy, vz]
   * @param next The next twist as a 3D vector [vx, vy, vz]
   * @param max_step_velocity The maximum allowed step size for velocity changes
   * @return The blended twist as a 3D vector [vx, vy, vz]
   */
  static Eigen::Vector<double, 3> blendTwistsWithRatio(
    const Eigen::Vector<double, 3> & prev,
    const Eigen::Vector<double, 3> & next,
    double blend_ratio);

  /**
   * @brief Corrects measured Euler angles to be closest to the state Euler angles, accounting for angle wrapping.
   * @param state_euler The current state Euler angles as a 3D vector [roll, pitch, yaw]
   * @param measured_euler The measured Euler angles as a 3D vector [roll, pitch, yaw]
   * @return The corrected measured Euler angles as a 3D vector [roll, pitch, yaw]
   */
  static Eigen::Vector3d correct_measured_euler(
    const Eigen::Vector3d & state_euler,
    const Eigen::Vector3d & measured_euler);

  /**
   * @brief Converts a quaternion to Euler angles (roll, pitch, yaw) in XYZ order.
   * @param q_in The input quaternion as Eigen::Quaterniond
   * @return The Euler angles as a 3D vector [roll, pitch, yaw] in radians
   */
  static Eigen::Vector3d quaternionToEuler(
    const Eigen::Quaterniond & q_in);

  /**
   * @brief Converts a geometry_msgs::TransformStamped message to a 4x4 Eigen transformation matrix.
   * @param tf_msg The input TransformStamped message
   * @return The corresponding 4x4 Eigen transformation matrix
   */
  static Eigen::Matrix4d transformStampedToEigen(
    const geometry_msgs::msg::TransformStamped & tf_msg);

  /**
   * @brief Converts an internal EKF pose measurement to an external pose measurement using the global map to odom transformation.
   * @param internal_pose The internal EKF pose measurement
   * @param global_map_to_odom The global map to odom transformation as a 7D vector [px, py, pz, qx, qy, qz, qw]
   * @param ekf_wrapper The EKFWrapper instance to use for transformations
   * @return The external pose measurement
   */
  static ekf::PoseMeasurement getExternalStatePoseFromInternalState(
    const ekf::PoseMeasurement & internal_pose,
    const Eigen::Vector<double, 7> global_map_to_odom,
    ekf::EKFWrapper & ekf_wrapper);

  /**
   * @brief Computes the time difference in seconds between two std_msgs::msg::Header messages.
   * @param header1 The first header message
   * @param header2 The second header message
   * @return The time difference in seconds (header2 - header1)
   */
  static double compute_time_difference(
    const std_msgs::msg::Header & header1,
    const std_msgs::msg::Header & header2);

  /**
   * @brief Fuses multiple pose measurements with their corresponding covariances into a single pose measurement and covariance.
   * @param accumulated_poses_ A vector of pose measurements to fuse
   * @param accumulated_poses_covariances_ A vector of corresponding pose measurement covariances
   * @return A pair containing the fused pose measurement and its covariance
   */
  static std::pair<ekf::PoseMeasurement, ekf::PoseMeasurementCovariance> fusePoseWithCovariance(
    const std::vector<ekf::PoseMeasurement> & accumulated_poses_,
    const std::vector<ekf::PoseMeasurementCovariance> & accumulated_poses_covariances_);
};

// Template for params declaration
template<typename T> T getParameter(as2::Node * node_ptr, const std::string & param_name)
{
  T param_value;
  try {
    if (!node_ptr->has_parameter(param_name)) {
      param_value = node_ptr->declare_parameter<T>(param_name);
    } else {
      node_ptr->get_parameter(param_name, param_value);
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      node_ptr->get_logger(), "Launch argument <%s> not defined or malformed: %s",
      param_name.c_str(), e.what());
    node_ptr->~Node();
  }
  std::ostringstream oss;
  oss << param_value;
  RCLCPP_INFO(node_ptr->get_logger(), "%s = %s", param_name.c_str(), oss.str().c_str());
  return param_value;
}

}  // namespace ekf_fuse

#endif  // EKF_FUSE_UTILS_HPP_
