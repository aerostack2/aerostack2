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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <string>

#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ekf/ekf_datatype.hpp"

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
   * @brief Constructor from explicit transforms
   *
   * @param map_to_base_transform The transform from map to base_link
   * @param map_to_odom_transform The transform from map to odom
   * @param odom_to_base_transform The transform from odom to base_link
   */
  StateTransforms(
    const tf2::Transform & map_to_base_transform,
    const tf2::Transform & map_to_odom_transform,
    const tf2::Transform & odom_to_base_transform
  )
  : map_to_base(map_to_base_transform),
    map_to_odom(map_to_odom_transform),
    odom_to_base(odom_to_base_transform)
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
 * @brief Transform pose with covariance to map frame
 *
 * Transforms a PoseWithCovarianceStamped from any frame (earth, map, odom, or base)
 * to the map frame. The transformation is determined by checking the frame_id in the
 * message header. The covariance is also rotated to match the new frame orientation.
 *
 * @param transforms Current state transforms (map_to_base, map_to_odom, odom_to_base)
 * @param earth_to_map Transform from earth frame to map frame
 * @param pose_msg Input pose with covariance in any frame
 * @return geometry_msgs::msg::PoseWithCovarianceStamped Pose in map frame with rotated covariance
 */
inline geometry_msgs::msg::PoseWithCovarianceStamped transformPoseToMapFrame(
  const StateTransforms & transforms,
  const tf2::Transform & earth_to_map,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped result = pose_msg;

  // Convert input pose to tf2::Transform
  tf2::Transform pose_transform;
  tf2::fromMsg(pose_msg.pose.pose, pose_transform);

  // Transform to map frame based on source frame
  tf2::Transform pose_in_map;
  tf2::Transform rotation_transform;

  std::string frame_id = pose_msg.header.frame_id;

  // Remove leading slash if present
  if (!frame_id.empty() && frame_id[0] == '/') {
    frame_id = frame_id.substr(1);
  }

  // Check frame and apply appropriate transformation
  if (frame_id.find("earth") != std::string::npos) {
    // Pose is in earth frame: map_pose = earth_to_map^-1 * earth_pose
    pose_in_map = earth_to_map.inverse() * pose_transform;
    rotation_transform = earth_to_map.inverse();
  } else if (frame_id.find("map") != std::string::npos) {
    // Pose is already in map frame
    pose_in_map = pose_transform;
    rotation_transform = tf2::Transform::getIdentity();
  } else if (frame_id.find("odom") != std::string::npos) {
    // Pose is in odom frame: map_pose = map_to_odom * odom_pose
    pose_in_map = transforms.map_to_odom * pose_transform;
    rotation_transform = transforms.map_to_odom;
  } else if (frame_id.find("base") != std::string::npos) {
    // Pose is in base frame: map_pose = map_to_base * base_pose
    pose_in_map = transforms.map_to_base * pose_transform;
    rotation_transform = transforms.map_to_base;
  } else {
    // Unknown frame, assume it's already in map frame
    pose_in_map = pose_transform;
    rotation_transform = tf2::Transform::getIdentity();
  }

  // Convert back to geometry_msgs::Pose
  // Manually convert tf2::Transform to geometry_msgs::Pose
  tf2::Vector3 position = pose_in_map.getOrigin();
  result.pose.pose.position.x = position.x();
  result.pose.pose.position.y = position.y();
  result.pose.pose.position.z = position.z();

  tf2::Quaternion rotation = pose_in_map.getRotation();
  result.pose.pose.orientation.x = rotation.x();
  result.pose.pose.orientation.y = rotation.y();
  result.pose.pose.orientation.z = rotation.z();
  result.pose.pose.orientation.w = rotation.w();

  result.header.frame_id = "map";

  // Rotate covariance to map frame
  // Extract rotation matrix (3x3) from the transform
  tf2::Matrix3x3 rotation_matrix = rotation_transform.getBasis();

  // Convert covariance array to Eigen matrices for easier manipulation
  Eigen::Matrix3d pos_cov_in, pos_cov_out;
  Eigen::Matrix3d rot_cov_in, rot_cov_out;

  // Extract position covariance (upper-left 3x3 block)
  pos_cov_in << pose_msg.pose.covariance[0],
    pose_msg.pose.covariance[1],
    pose_msg.pose.covariance[2],
    pose_msg.pose.covariance[6],
    pose_msg.pose.covariance[7],
    pose_msg.pose.covariance[8],
    pose_msg.pose.covariance[12],
    pose_msg.pose.covariance[13],
    pose_msg.pose.covariance[14];

  // Extract orientation covariance (lower-right 3x3 block)
  rot_cov_in << pose_msg.pose.covariance[21],
    pose_msg.pose.covariance[22],
    pose_msg.pose.covariance[23],
    pose_msg.pose.covariance[27],
    pose_msg.pose.covariance[28],
    pose_msg.pose.covariance[29],
    pose_msg.pose.covariance[33],
    pose_msg.pose.covariance[34],
    pose_msg.pose.covariance[35];

  // Convert tf2::Matrix3x3 to Eigen::Matrix3d
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R(i, j) = rotation_matrix[i][j];
    }
  }

  // Rotate covariance: Cov_out = R * Cov_in * R^T
  pos_cov_out = R * pos_cov_in * R.transpose();
  rot_cov_out = R * rot_cov_in * R.transpose();

  // Write back to result (keeping cross-covariance terms as zero)
  result.pose.covariance = {};  // Initialize all to zero

  // Position covariance (upper-left 3x3)
  result.pose.covariance[0] = pos_cov_out(0, 0);
  result.pose.covariance[1] = pos_cov_out(0, 1);
  result.pose.covariance[2] = pos_cov_out(0, 2);
  result.pose.covariance[6] = pos_cov_out(1, 0);
  result.pose.covariance[7] = pos_cov_out(1, 1);
  result.pose.covariance[8] = pos_cov_out(1, 2);
  result.pose.covariance[12] = pos_cov_out(2, 0);
  result.pose.covariance[13] = pos_cov_out(2, 1);
  result.pose.covariance[14] = pos_cov_out(2, 2);

  // Orientation covariance (lower-right 3x3)
  result.pose.covariance[21] = rot_cov_out(0, 0);
  result.pose.covariance[22] = rot_cov_out(0, 1);
  result.pose.covariance[23] = rot_cov_out(0, 2);
  result.pose.covariance[27] = rot_cov_out(1, 0);
  result.pose.covariance[28] = rot_cov_out(1, 1);
  result.pose.covariance[29] = rot_cov_out(1, 2);
  result.pose.covariance[33] = rot_cov_out(2, 0);
  result.pose.covariance[34] = rot_cov_out(2, 1);
  result.pose.covariance[35] = rot_cov_out(2, 2);

  return result;
}

/**
 * @brief Convert PoseWithCovarianceStamped to EKF pose measurement
 *
 * Extracts position and orientation from a PoseWithCovarianceStamped message
 * and converts it to an EKF pose measurement format (6-element vector: position + Euler angles).
 * The Euler angles are unwrapped to be closest to the current EKF state angles, preventing
 * discontinuity spikes when angles cross the ±π boundary.
 *
 * @param pose_msg Input pose with covariance stamped message
 * @param current_state Current EKF state used to unwrap the measured Euler angles
 * @return ekf::PoseMeasurement 6-element measurement vector [x, y, z, roll, pitch, yaw]
 */
inline ekf::PoseMeasurement poseWithCovarianceToEkfMeasurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg,
  const ekf::State & current_state)
{
  ekf::PoseMeasurement measurement;

  // Position (first 3 elements)
  measurement.data[ekf::PoseMeasurement::X] = pose_msg.pose.pose.position.x;
  measurement.data[ekf::PoseMeasurement::Y] = pose_msg.pose.pose.position.y;
  measurement.data[ekf::PoseMeasurement::Z] = pose_msg.pose.pose.position.z;

  // Convert quaternion to Euler angles (roll, pitch, yaw)
  tf2::Quaternion q(
    pose_msg.pose.pose.orientation.x,
    pose_msg.pose.pose.orientation.y,
    pose_msg.pose.pose.orientation.z,
    pose_msg.pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Unwrap measured angles to be closest to the current EKF state angles.
  // getRPY always returns values in [-π, π], but the EKF state tracks orientation
  // continuously and may have accumulated past ±π. Without unwrapping, a crossing
  // of the ±π boundary would look like a ~2π jump to the EKF, causing a spike.
  auto unwrap_angle = [](double state_angle, double meas_angle) -> double {
      double diff = meas_angle - state_angle;
      diff -= 2.0 * M_PI * std::round(diff / (2.0 * M_PI));
      return state_angle + diff;
    };

  roll = unwrap_angle(current_state.data[ekf::State::ROLL], roll);
  pitch = unwrap_angle(current_state.data[ekf::State::PITCH], pitch);
  yaw = unwrap_angle(current_state.data[ekf::State::YAW], yaw);

  // Orientation as Euler angles (last 3 elements)
  measurement.data[ekf::PoseMeasurement::ROLL] = roll;
  measurement.data[ekf::PoseMeasurement::PITCH] = pitch;
  measurement.data[ekf::PoseMeasurement::YAW] = yaw;

  return measurement;
}

/**
 * @brief Convert PoseWithCovariance to EKF pose measurement covariance
 *
 * Extracts the diagonal covariance values from a PoseWithCovariance message and converts
 * it to an EKF pose measurement covariance. The EKF uses a diagonal covariance representation
 * with 6 variance values for position and orientation.
 *
 * @param pose_cov Input pose with covariance
 * @return ekf::PoseMeasurementCovariance The diagonal covariance for the pose measurement
 */
inline ekf::PoseMeasurementCovariance poseWithCovarianceToEkfMeasurementCovariance(
  const geometry_msgs::msg::PoseWithCovariance & pose_cov)
{
  ekf::PoseMeasurementCovariance measurement_cov;

  // Extract diagonal elements from the 6x6 covariance matrix
  // The covariance is stored in row-major order in the geometry_msgs

  // Position variance (diagonal elements: 0, 7, 14)
  measurement_cov.data[ekf::PoseMeasurementCovariance::X] = pose_cov.covariance[0];   // σ²_x
  measurement_cov.data[ekf::PoseMeasurementCovariance::Y] = pose_cov.covariance[7];   // σ²_y
  measurement_cov.data[ekf::PoseMeasurementCovariance::Z] = pose_cov.covariance[14];  // σ²_z

  // Orientation variance (diagonal elements: 21, 28, 35)
  measurement_cov.data[ekf::PoseMeasurementCovariance::ROLL] =
    pose_cov.covariance[21];  // σ²_roll
  measurement_cov.data[ekf::PoseMeasurementCovariance::PITCH] =
    pose_cov.covariance[28];  // σ²_pitch
  measurement_cov.data[ekf::PoseMeasurementCovariance::YAW] =
    pose_cov.covariance[35];  // σ²_yaw

  return measurement_cov;
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
