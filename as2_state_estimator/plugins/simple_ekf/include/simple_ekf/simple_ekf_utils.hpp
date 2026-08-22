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

#ifndef SIMPLE_EKF__SIMPLE_EKF_UTILS_HPP_
#define SIMPLE_EKF__SIMPLE_EKF_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ekf/ekf_datatype.hpp"

namespace simple_ekf
{

/**
 * @brief Variance standing in for a component a measurement does not observe.
 *
 * Messages mark an unobserved component with a non-positive variance, which no filter can
 * use: zero means exact and a negative number is not a variance. The textbook answer is a
 * variance so large that the gain vanishes, and it does not survive contact with this
 * filter: the gain comes from inverting the whole innovation covariance, and once that
 * holds position-to-orientation correlations, mixing 1e9 with 1e-3 leaves an inverse with
 * no significant digits and the filter diverges within a few samples.
 *
 * What does keep an unobserved component out of the correction is giving it no innovation:
 * @ref neutraliseUnobservedComponents replaces the measured value with the predicted one,
 * so the correction is zero however the gain comes out. This variance then only has to be
 * large enough not to shrink that component's covariance — a hundred, against state
 * variances around 1e-2, leaves it untouched to four decimals — and small enough to keep
 * the innovation covariance well conditioned.
 */
constexpr double kUnobservedVariance = 1.0e2;

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
  std::string rigid_body_name;               ///< Rigid body name for mocap4r2_msgs/msg/RigidBodies
  double update_rate_hz = 0.0;               ///< Max EKF update rate for topic, Hz (0=no limit)
  bool is_odometry = false;                  ///< Correction absorbed by odom->base (true) or
                                             ///< allowed to move map->odom (false)
  bool reject_repeated_positions = false;    ///< Drop messages repeating this topic's last
                                             ///< received position
  std::array<double, 3> linear_values{};     ///< Linear velocity covariance or multiplier
                                             ///< [x, y, z], for twist topics
  bool is_body_frame = false;                ///< Twist topics: the velocity is expressed
                                             ///< in the frame of the vehicle, not the map
  double innovation_gate = 0.0;              ///< Reject a measurement further than this
                                             ///< many standard deviations from the
                                             ///< prediction. 0 disables the gate
  double innovation_gate_timeout = 1.0;      ///< Seconds of uninterrupted rejection after
                                             ///< which the next measurement is accepted
};

/**
 * @brief Whether a topic's type carries a velocity rather than a pose.
 *
 * @param type Message type string from the topic's `type` parameter
 * @return true for the twist types, which correct the velocity states
 */
inline bool isVelocityType(const std::string & type)
{
  return type == "geometry_msgs/msg/TwistWithCovarianceStamped";
}

/**
 * @brief Default value of a topic's `is_odometry` flag when the user does not set it.
 *
 * Odometry messages are, by convention, a dead-reckoned pose that drifts with respect to
 * the map: its correction should be absorbed by odom->base rather than jumping map->odom.
 * Every other supported type carries an absolute pose, so its correction moves map->odom.
 * Users can override this per topic with the `is_odometry` parameter.
 *
 * @param type Message type string from the topic's `type` parameter
 * @return true if the topic should default to odometry semantics
 */
inline bool defaultIsOdometryForType(const std::string & type)
{
  return type == "nav_msgs/msg/Odometry";
}

/**
 * @brief Default value of a topic's `reject_repeated_positions` flag when the user does not
 *        set it.
 *
 * A motion capture system keeps publishing the last known pose when its cameras lose the
 * rigid body, so an exactly repeated position means the tracking was lost rather than the
 * robot being still: a tracked body always jitters. Feeding those repeats to the filter
 * makes it increasingly confident about a position nobody is measuring any more, so mocap
 * topics reject them by default.
 *
 * Every other supported type may legitimately report the exact same position twice, which
 * simply means the robot is not moving, so they accept repeats by default. Users can
 * override this per topic with the `reject_repeated_positions` parameter.
 *
 * @param type Message type string from the topic's `type` parameter
 * @return true if the topic should reject repeated positions by default
 */
inline bool defaultRejectRepeatedPositionsForType(const std::string & type)
{
  return type == "mocap4r2_msgs/msg/RigidBodies";
}

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
 * @brief Exponential moving average blend of two transforms
 *
 * Position is linearly interpolated and orientation is interpolated with slerp
 * (tf2's slerp already takes the shortest path, so a prev/next pair straddling
 * +-pi interpolates the short way round).
 *
 * @param prev The previous (external) transform
 * @param next The new (internal) transform
 * @param alpha Weight of `next` in [0, 1]: 1 returns `next`, 0 returns `prev`
 * @return tf2::Transform The blended transform
 */
inline tf2::Transform blendTransforms(
  const tf2::Transform & prev,
  const tf2::Transform & next,
  double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);

  tf2::Quaternion rotation = prev.getRotation().slerp(next.getRotation(), alpha);
  rotation.normalize();

  return tf2::Transform(rotation, prev.getOrigin().lerp(next.getOrigin(), alpha));
}

/**
 * @brief Exponential moving average blend of two vectors
 *
 * @param prev The previous (external) vector
 * @param next The new (internal) vector
 * @param alpha Weight of `next` in [0, 1]: 1 returns `next`, 0 returns `prev`
 * @return tf2::Vector3 The blended vector
 */
inline tf2::Vector3 blendVectors(
  const tf2::Vector3 & prev,
  const tf2::Vector3 & next,
  double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  return prev.lerp(next, alpha);
}

/**
 * @brief Convert EKF state and IMU message to twist message in base frame
 *
 * Transforms the given map-frame velocity to the base frame, and computes angular
 * velocity by removing gyroscope bias from IMU measurements.
 *
 * The linear velocity is passed in rather than read from `state` so the caller can
 * supply either the raw EKF velocity or one corrected for the smoothed map to odom
 * velocity (see Plugin::updateStateFromEkf).
 *
 * @param state The EKF state containing the gyroscope bias
 * @param map_to_base The transform from map frame to base_link frame
 * @param imu_msg The IMU message containing angular velocity measurements
 * @param velocity_in_map The linear velocity expressed in the map frame
 * @return geometry_msgs::msg::TwistWithCovariance The twist in base frame
 */
inline geometry_msgs::msg::TwistWithCovariance ekfStateToTwist(
  const ekf::State & state,
  const tf2::Transform & map_to_base,
  const sensor_msgs::msg::Imu & imu_msg,
  const tf2::Vector3 & velocity_in_map)
{
  geometry_msgs::msg::TwistWithCovariance twist_msg;

  // Transform velocity from map frame to base frame
  // vel_base = R_base_map * vel_map = R_map_base^T * vel_map
  tf2::Vector3 vel_base = map_to_base.getBasis().transpose() * velocity_in_map;

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
 * @brief Convert PoseWithCovarianceStamped to a raw EKF pose measurement
 *
 * Extracts position and orientation from a PoseWithCovarianceStamped message
 * and converts it to an EKF pose measurement format (6-element vector: position + Euler angles).
 * The Euler angles are returned as-is from tf2::Matrix3x3::getRPY(), i.e. wrapped to
 * [-π, π] — NOT unwrapped relative to any reference state. Use unwrapPoseMeasurement()
 * to unwrap them before feeding them to the EKF update.
 *
 * @param pose_msg Input pose with covariance stamped message
 * @return ekf::PoseMeasurement 6-element measurement vector [x, y, z, roll, pitch, yaw],
 *         with roll/pitch/yaw in [-π, π]
 */
inline ekf::PoseMeasurement poseWithCovarianceToRawEkfMeasurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg)
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

  // Orientation as Euler angles (last 3 elements), in [-π, π]
  measurement.data[ekf::PoseMeasurement::ROLL] = roll;
  measurement.data[ekf::PoseMeasurement::PITCH] = pitch;
  measurement.data[ekf::PoseMeasurement::YAW] = yaw;

  return measurement;
}

/**
 * @brief Unwrap a raw pose measurement's Euler angles relative to a reference state
 *
 * getRPY() always returns values in [-π, π], but the EKF state tracks orientation
 * continuously and may have accumulated past ±π. Without unwrapping, a crossing
 * of the ±π boundary would look like a ~2π jump to the EKF, causing a spike. This
 * picks the representative raw_angle + 2π·k closest to the reference state's angle.
 *
 * @param raw Raw pose measurement with roll/pitch/yaw in [-π, π]
 * @param reference_state EKF state whose ROLL/PITCH/YAW (always in [-π, π], see
 *        EKFWrapper::correct_state()) are used as the unwrap reference
 * @return ekf::PoseMeasurement with x/y/z copied unchanged and roll/pitch/yaw unwrapped
 *         to be within [-π, π] of reference_state
 */
inline ekf::PoseMeasurement unwrapPoseMeasurement(
  const ekf::PoseMeasurement & raw,
  const ekf::State & reference_state)
{
  ekf::PoseMeasurement unwrapped = raw;

  auto unwrap_angle = [](double state_angle, double meas_angle) -> double {
      double diff = meas_angle - state_angle;
      diff -= 2.0 * M_PI * std::round(diff / (2.0 * M_PI));
      return state_angle + diff;
    };

  unwrapped.data[ekf::PoseMeasurement::ROLL] = unwrap_angle(
    reference_state.data[ekf::State::ROLL], raw.data[ekf::PoseMeasurement::ROLL]);
  unwrapped.data[ekf::PoseMeasurement::PITCH] = unwrap_angle(
    reference_state.data[ekf::State::PITCH], raw.data[ekf::PoseMeasurement::PITCH]);
  unwrapped.data[ekf::PoseMeasurement::YAW] = unwrap_angle(
    reference_state.data[ekf::State::YAW], raw.data[ekf::PoseMeasurement::YAW]);

  return unwrapped;
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
 * @brief Which components of a measurement carry no information.
 *
 * @param covariance 6x6 covariance in row-major order
 * @return One flag per component, true where the variance is non-positive
 */
inline std::array<bool, 6> unobservedComponents(const std::array<double, 36> & covariance)
{
  std::array<bool, 6> unobserved{};
  for (std::size_t index = 0; index < 6; ++index) {
    unobserved[index] = covariance[index * 6 + index] <= 0.0;
  }
  return unobserved;
}

/**
 * @brief Replace every non-positive diagonal variance with @ref kUnobservedVariance.
 *
 * Applied to a measurement before anything else touches it, so that the rest of the
 * pipeline — the rotation into the map frame included — only ever sees usable numbers.
 *
 * @param covariance 6x6 covariance in row-major order, modified in place
 */
inline void resolveUnobservedVariances(std::array<double, 36> & covariance)
{
  for (std::size_t index = 0; index < 6; ++index) {
    double & variance = covariance[index * 6 + index];
    if (variance <= 0.0) {
      variance = kUnobservedVariance;
    }
  }
}

/**
 * @brief Give the unobserved components of a pose measurement nothing to say.
 *
 * Each flagged component is overwritten with the filter's own prediction and given
 * @ref kUnobservedVariance, so its innovation is zero and the correction it produces is
 * zero with it, whatever the gain turns out to be. The observed components are untouched.
 *
 * The flags are read in the frame the measurement arrived in and applied after the rotation
 * into the map frame, which is exact whenever the unobserved set survives that rotation:
 * all three position components, none of them, or the horizontal pair under this tree's
 * yaw-only rotations. Nothing here can make it exact for an arbitrary rotation of a
 * partially observed position — the information is no longer aligned with the axes.
 *
 * @param measurement Pose measurement in the map frame, modified in place
 * @param covariance Its covariance, modified in place
 * @param unobserved One flag per component, from @ref unobservedComponents
 * @param state State the predicted values are taken from
 */
inline void neutraliseUnobservedComponents(
  ekf::PoseMeasurement & measurement,
  ekf::PoseMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state)
{
  static constexpr std::array<int, 6> kStateIndices = {
    ekf::State::X, ekf::State::Y, ekf::State::Z,
    ekf::State::ROLL, ekf::State::PITCH, ekf::State::YAW};

  for (std::size_t index = 0; index < 6; ++index) {
    if (!unobserved[index]) {
      continue;
    }
    measurement.data[index] = state.data[kStateIndices[index]];
    covariance.data[index] = kUnobservedVariance;
  }
}

/**
 * @brief Give the unobserved components of a velocity measurement nothing to say.
 *
 * The velocity counterpart of @ref neutraliseUnobservedComponents.
 *
 * @param measurement Velocity measurement in the map frame, modified in place
 * @param covariance Its covariance, modified in place
 * @param unobserved One flag per component, from @ref unobservedComponents
 * @param state State the predicted values are taken from
 */
inline void neutraliseUnobservedVelocityComponents(
  ekf::VelocityMeasurement & measurement,
  ekf::VelocityMeasurementCovariance & covariance,
  const std::array<bool, 6> & unobserved,
  const ekf::State & state)
{
  static constexpr std::array<int, 3> kStateIndices = {
    ekf::State::VX, ekf::State::VY, ekf::State::VZ};

  for (std::size_t index = 0; index < 3; ++index) {
    if (!unobserved[index]) {
      continue;
    }
    measurement.data[index] = state.data[kStateIndices[index]];
    covariance.data[index] = kUnobservedVariance;
  }
}

/**
 * @brief Apply a twist topic's configured covariance to the linear diagonal.
 *
 * The counterpart of @ref getCovarianceWithConfig for a velocity source: either the
 * configured variances replace the message's, or they scale them. The angular block is
 * left untouched, since no velocity correction reads it.
 *
 * @param input_covariance 6x6 twist covariance from the message, row-major
 * @param config Topic configuration providing `linear_values`
 * @return The resulting 6x6 covariance
 */
inline std::array<double, 36> getLinearCovarianceWithConfig(
  const std::array<double, 36> & input_covariance,
  const PoseTopicConfig & config)
{
  std::array<double, 36> covariance = input_covariance;

  if (config.use_message_covariance) {
    covariance[0] *= config.linear_values[0];
    covariance[7] *= config.linear_values[1];
    covariance[14] *= config.linear_values[2];
  } else {
    covariance[0] = config.linear_values[0];
    covariance[7] = config.linear_values[1];
    covariance[14] = config.linear_values[2];
  }

  return covariance;
}

/**
 * @brief Transform a twist and its covariance into the map frame.
 *
 * A velocity is a free vector, so only the rotation of the source frame applies, and the
 * frame is picked from the message header exactly as @ref transformPoseToMapFrame does.
 *
 * The rotated covariance is not diagonal in general, while the velocity update takes one
 * variance per axis, so only the diagonal is kept and the correlation between map frame
 * axes is dropped. For a source whose axes are similarly noisy — an optical flow sensor's
 * two horizontal axes are the same measurement — that is small at the tilts a multirotor
 * flies at, and no variance is ever understated by it.
 *
 * @param transforms Current state transforms (map_to_base, map_to_odom, odom_to_base)
 * @param earth_to_map Transform from earth frame to map frame
 * @param twist_msg Input twist with covariance in any frame
 * @return The twist in the map frame, with its covariance rotated
 */
inline geometry_msgs::msg::TwistWithCovarianceStamped transformTwistToMapFrame(
  const StateTransforms & transforms,
  const tf2::Transform & earth_to_map,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg)
{
  geometry_msgs::msg::TwistWithCovarianceStamped result = twist_msg;

  std::string frame_id = twist_msg.header.frame_id;
  if (!frame_id.empty() && frame_id[0] == '/') {
    frame_id = frame_id.substr(1);
  }

  tf2::Transform rotation_transform = tf2::Transform::getIdentity();
  if (frame_id.find("earth") != std::string::npos) {
    rotation_transform = earth_to_map.inverse();
  } else if (frame_id.find("odom") != std::string::npos) {
    rotation_transform = transforms.map_to_odom;
  } else if (frame_id.find("map") == std::string::npos) {
    // Anything that is not one of the tree's own frames is the vehicle's, since that is
    // the frame a twist is normally expressed in.
    rotation_transform = transforms.map_to_base;
  }

  const tf2::Matrix3x3 & rotation_matrix = rotation_transform.getBasis();
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      rotation(row, column) = rotation_matrix[row][column];
    }
  }

  const tf2::Vector3 linear = rotation_matrix *
    tf2::Vector3(
    twist_msg.twist.twist.linear.x, twist_msg.twist.twist.linear.y,
    twist_msg.twist.twist.linear.z);
  result.twist.twist.linear.x = linear.x();
  result.twist.twist.linear.y = linear.y();
  result.twist.twist.linear.z = linear.z();

  Eigen::Matrix3d linear_covariance;
  linear_covariance <<
    twist_msg.twist.covariance[0], twist_msg.twist.covariance[1], twist_msg.twist.covariance[2],
    twist_msg.twist.covariance[6], twist_msg.twist.covariance[7], twist_msg.twist.covariance[8],
    twist_msg.twist.covariance[12], twist_msg.twist.covariance[13],
    twist_msg.twist.covariance[14];

  const Eigen::Matrix3d rotated = rotation * linear_covariance * rotation.transpose();
  result.twist.covariance[0] = rotated(0, 0);
  result.twist.covariance[7] = rotated(1, 1);
  result.twist.covariance[14] = rotated(2, 2);

  result.header.frame_id = "map";
  return result;
}

/**
 * @brief Convert the linear part of a twist into an EKF velocity measurement.
 *
 * @param twist_msg Twist already expressed in the map frame
 * @return The 3-element velocity measurement
 */
inline ekf::VelocityMeasurement twistToEkfVelocityMeasurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg)
{
  ekf::VelocityMeasurement measurement;
  measurement.data[ekf::VelocityMeasurement::VX] = twist_msg.twist.twist.linear.x;
  measurement.data[ekf::VelocityMeasurement::VY] = twist_msg.twist.twist.linear.y;
  measurement.data[ekf::VelocityMeasurement::VZ] = twist_msg.twist.twist.linear.z;
  return measurement;
}

/**
 * @brief Extract the linear diagonal of a twist covariance as an EKF measurement covariance.
 *
 * @param twist_msg Twist already expressed in the map frame
 * @return The diagonal covariance of the velocity measurement
 */
inline ekf::VelocityMeasurementCovariance twistToEkfVelocityCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg)
{
  ekf::VelocityMeasurementCovariance measurement_cov;
  measurement_cov.data[ekf::VelocityMeasurementCovariance::VX] = twist_msg.twist.covariance[0];
  measurement_cov.data[ekf::VelocityMeasurementCovariance::VY] = twist_msg.twist.covariance[7];
  measurement_cov.data[ekf::VelocityMeasurementCovariance::VZ] = twist_msg.twist.covariance[14];
  return measurement_cov;
}

/**
 * @brief Whether a measurement is close enough to the prediction to be believed.
 *
 * The measurement models select states directly, so a component's innovation is the
 * difference between measurement and prediction and its variance is the sum of the two.
 * Further than @p gate standard deviations away is not a measurement of this vehicle: an
 * obstacle under the rangefinder, a motion capture frame that swapped bodies, a sensor gone
 * wrong. A component the measurement does not observe passes on its own, since
 * @ref kUnobservedVariance dwarfs any innovation it could produce.
 *
 * @param innovations Per-component difference between measurement and prediction
 * @param state_variances Variance of the predicted state for each component
 * @param measurement_variances Variance of the measurement for each component
 * @param gate Number of standard deviations allowed. Non-positive disables the check
 * @return true when every component is within the gate
 */
template<std::size_t N>
bool isWithinInnovationGate(
  const std::array<double, N> & innovations,
  const std::array<double, N> & state_variances,
  const std::array<double, N> & measurement_variances,
  double gate)
{
  if (gate <= 0.0) {
    return true;
  }

  for (std::size_t index = 0; index < N; ++index) {
    const double innovation_variance = state_variances[index] + measurement_variances[index];
    if (innovation_variance <= 0.0) {
      continue;
    }
    if (innovations[index] * innovations[index] >
      gate * gate * innovation_variance)
    {
      return false;
    }
  }
  return true;
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

#endif  // SIMPLE_EKF__SIMPLE_EKF_UTILS_HPP_
