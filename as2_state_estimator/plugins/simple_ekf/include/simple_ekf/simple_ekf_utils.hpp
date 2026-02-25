// Copyright 2024 Universidad Politécnica de Madrid
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
* @file simple_ekf_utils.hpp
*
* Utility functions for simple_ekf plugin
*
* @authors David Pérez Saura
*          Rafael Pérez Seguí
*          Javier Melero Deza
*          Miguel Fernández Cortizas
*          Pedro Arias Pérez
*/

#ifndef SIMPLE_EKF_UTILS_HPP_
#define SIMPLE_EKF_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <string>

#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ekf/ekf_datatype.hpp>

namespace simple_ekf
{

/**
 * @brief Configuration for a pose topic subscription
 *
 * This structure holds all configuration parameters for a single pose topic,
 * including whether to use message covariances and custom covariance values.
 */
struct PoseTopicConfig
{
  std::string topic;                         ///< Topic name to subscribe to
  std::string type;                          ///< Message type (e.g,"geometry_msgs/msg/PoseStamped")
  bool set_earth_map;                        ///< Set earth-to-map transform from this topic
  bool use_message_covariance;               ///< Use message covariance or custom values
  std::array<double, 3> position_values;     ///< Position covariance or multiplier [x, y, z]
  std::array<double, 3> orientation_values;  ///< Orientation covariance/multiplier [r, p, y]
};

/**
 * @brief Structure holding the TF transforms for the state estimator
 *
 * This structure contains the three main transforms needed for the
 * state estimation: map to base, map to odom, and odom to base.
 */
struct StateTransforms
{
  tf2::Transform map_to_base;   ///< Transform from map frame to base_link frame
  tf2::Transform map_to_odom;   ///< Transform from map frame to odom frame
  tf2::Transform odom_to_base;  ///< Transform from odom frame to base_link frame

  /**
   * @brief Default constructor - initializes all transforms to identity
   */
  StateTransforms()
  : map_to_base(tf2::Transform::getIdentity()),
    map_to_odom(tf2::Transform::getIdentity()),
    odom_to_base(tf2::Transform::getIdentity())
  {}

  /**
   * @brief Constructor from ekf::State
   *
   * Extracts position and orientation from the EKF state and creates
   * the necessary transforms. The map_to_base transform is created directly
   * from the state, while map_to_odom is set to identity (can be customized),
   * and odom_to_base is computed as the relative transform.
   *
   * @param state The EKF state
   * @param map_to_odom_transform The map to odom transform (defaults to identity)
   */
  StateTransforms(
    const ekf::State & state,
    const tf2::Transform & map_to_odom_transform = tf2::Transform::getIdentity())
  {
    // Get position and orientation from state
    auto position = state.get_position();
    auto orientation_quat = state.get_orientation_quaternion();

    // Create map_to_base transform from state
    tf2::Vector3 translation(position[0], position[1], position[2]);
    tf2::Quaternion rotation(
      orientation_quat[0],  // qx
      orientation_quat[1],  // qy
      orientation_quat[2],  // qz
      orientation_quat[3]);  // qw
    rotation.normalize();

    map_to_base.setOrigin(translation);
    map_to_base.setRotation(rotation);

    // Set map_to_odom (provided or identity)
    map_to_odom = map_to_odom_transform;

    // Compute odom_to_base as: odom_to_base = map_to_odom^-1 * map_to_base
    odom_to_base = map_to_odom.inverse() * map_to_base;
  }
};

/**
 * @brief Convert Eigen::Matrix4d homogeneous transformation to tf2::Transform
 *
 * Extracts translation and rotation from a 4x4 homogeneous transformation
 * matrix and creates a tf2::Transform object.
 *
 * @param matrix The 4x4 homogeneous transformation matrix
 * @return tf2::Transform The equivalent tf2 transform
 */
inline tf2::Transform eigenMatrix4dToTf2Transform(const Eigen::Matrix4d & matrix)
{
  // Extract translation (last column, first 3 rows)
  tf2::Vector3 translation(
    matrix(0, 3),
    matrix(1, 3),
    matrix(2, 3));

  // Extract rotation matrix (top-left 3x3 block)
  Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);

  // Convert rotation matrix to quaternion
  Eigen::Quaterniond quat(rotation_matrix);
  tf2::Quaternion rotation(quat.x(), quat.y(), quat.z(), quat.w());
  rotation.normalize();

  // Create and return the transform
  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);

  return transform;
}

/**
 * @brief Convert tf2::Transform to Eigen::Matrix4d homogeneous transformation
 *
 * Creates a 4x4 homogeneous transformation matrix from a tf2::Transform.
 *
 * @param transform The tf2 transform
 * @return Eigen::Matrix4d The equivalent 4x4 homogeneous transformation matrix
 */
inline Eigen::Matrix4d tf2TransformToEigenMatrix4d(const tf2::Transform & transform)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  // Get translation
  const tf2::Vector3 & translation = transform.getOrigin();
  matrix(0, 3) = translation.x();
  matrix(1, 3) = translation.y();
  matrix(2, 3) = translation.z();

  // Get rotation and convert to matrix
  const tf2::Quaternion & rotation = transform.getRotation();
  Eigen::Quaterniond quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

  // Set rotation part (top-left 3x3 block)
  matrix.block<3, 3>(0, 0) = rotation_matrix;

  return matrix;
}

/**
 * @brief Convert EKF state and IMU message to twist message in base frame
 *
 * Extracts velocity from the EKF state, transforms it to the base frame,
 * and computes angular velocity by removing gyroscope bias from IMU measurements.
 *
 * @param state The EKF state containing velocity and gyroscope bias
 * @param map_to_base The transform from map frame to base_link frame
 * @param imu_msg The IMU message containing angular velocity measurements
 * @return geometry_msgs::msg::TwistWithCovariance The twist in base frame
 */
inline geometry_msgs::msg::TwistWithCovariance ekfStateToTwist(
  const ekf::State & state,
  const tf2::Transform & map_to_base,
  const sensor_msgs::msg::Imu & imu_msg)
{
  geometry_msgs::msg::TwistWithCovariance twist_msg;

  // Get velocity from state (in map frame)
  auto velocity = state.get_velocity();
  tf2::Vector3 vel_map(velocity[0], velocity[1], velocity[2]);

  // Transform velocity from map frame to base frame
  // vel_base = R_base_map * vel_map = R_map_base^T * vel_map
  tf2::Vector3 vel_base = map_to_base.getBasis().transpose() * vel_map;

  // Set linear velocity in base frame
  twist_msg.twist.linear.x = vel_base[0];
  twist_msg.twist.linear.y = vel_base[1];
  twist_msg.twist.linear.z = vel_base[2];

  // Set angular velocity (IMU measurement - gyroscope bias)
  twist_msg.twist.angular.x = imu_msg.angular_velocity.x - state.data[ekf::State::WBX];
  twist_msg.twist.angular.y = imu_msg.angular_velocity.y - state.data[ekf::State::WBY];
  twist_msg.twist.angular.z = imu_msg.angular_velocity.z - state.data[ekf::State::WBZ];

  return twist_msg;
}

/**
 * @brief Generate covariance array from pose topic configuration
 *
 * Creates a 6x6 covariance matrix (stored as 36-element array) with position
 * and orientation variances from the configuration. The matrix is diagonal with
 * position variances at (0,0), (1,1), (2,2) and orientation variances at (3,3), (4,4), (5,5).
 *
 * @param config The pose topic configuration containing position and orientation values
 * @return std::array<double, 36> The 6x6 covariance matrix in row-major order
 */
inline std::array<double, 36> generateCovarianceFromConfig(const PoseTopicConfig & config)
{
  std::array<double, 36> covariance = {};  // Initialize all to zero

  if (config.use_message_covariance) {
    // If using message covariance, we will modify the input covariance in getCovarianceWithConfig
    return covariance;  // Return zero covariance as placeholder
  }

  // Position covariance (diagonal elements: xx, yy, zz at indices 0, 7, 14)
  covariance[0] = config.position_values[0];   // x variance
  covariance[7] = config.position_values[1];   // y variance
  covariance[14] = config.position_values[2];  // z variance

  // Orientation covariance (diagonal elements: rr, pp, yy at indices 21, 28, 35)
  covariance[21] = config.orientation_values[0];  // roll variance
  covariance[28] = config.orientation_values[1];  // pitch variance
  covariance[35] = config.orientation_values[2];  // yaw variance

  return covariance;
}

/**
 * @brief Get covariance array based on configuration and optional existing covariance
 *
 * Generates a 6x6 covariance matrix (stored as 36-element array) based on
 * the configuration settings. If use_message_covariance is false, returns
 * fixed covariance values from config. If true, multiplies the input covariance
 * diagonal elements by the configured multiplier values.
 *
 * @param input_covariance The input covariance array (used when use_message_covariance is true)
 * @param config The pose topic configuration containing position and orientation values
 * @return std::array<double, 36> The resulting 6x6 covariance matrix in row-major order
 */
inline std::array<double, 36> getCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const PoseTopicConfig & config)
{
  if (!config.use_message_covariance) {
    // Return fixed values from config
    return generateCovarianceFromConfig(config);
  } else {
    // Apply multipliers to the diagonal elements of input covariance
    std::array<double, 36> covariance = input_covariance;

    // Position covariance (diagonal elements: 0, 7, 14)
    covariance[0] *= config.position_values[0];   // x variance
    covariance[7] *= config.position_values[1];   // y variance
    covariance[14] *= config.position_values[2];  // z variance

    // Orientation covariance (diagonal elements: 21, 28, 35)
    covariance[21] *= config.orientation_values[0];  // roll variance
    covariance[28] *= config.orientation_values[1];  // pitch variance
    covariance[35] *= config.orientation_values[2];  // yaw variance

    return covariance;
  }
}

/**
 * @class SimpleEKFUtils
 * @brief Utility functions for simple EKF operations
 *
 * This class provides static utility methods for common operations
 * in the simple EKF plugin, such as coordinate transformations
 * and data conversions.
 */
class SimpleEKFUtils
{
public:
  /**
   * @brief Deleted constructor - utility class with only static methods
   */
  SimpleEKFUtils() = delete;

  /**
   * @brief Deleted destructor - utility class with only static methods
   */
  ~SimpleEKFUtils() = delete;
};

}  // namespace simple_ekf

#endif  // SIMPLE_EKF_UTILS_HPP_
