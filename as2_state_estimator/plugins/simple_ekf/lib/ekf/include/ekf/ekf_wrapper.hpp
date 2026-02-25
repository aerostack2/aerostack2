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
* @file ekf_wrapper.hpp
*
* An EKF Wrapper implementation
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef EKF__EKF_WRAPPER_HPP
#define EKF__EKF_WRAPPER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

#include <ekf/ekf_c_code.h>

#include "Eigen/src/Core/Matrix.h"
#include "ekf/ekf_datatype.hpp"

namespace ekf
{

/**
 * @brief EKFData
 *
 * Data structure to hold the EKF data.
 */
struct EKFData
{
  State state; // Current state of the EKF
  Covariance covariance; // Current covariance of the EKF
  Gravity gravity; // Gravity vector
  Eigen::Matrix4d map_to_odom; // Transformation matrix from map to odometry frame
  Eigen::Vector3d map_to_odom_velocity; // Velocity of the map to odom frame
};

/**
 * @brief EKFWrapper
 *
 * Class to wrap the EKF functionality.
 */
class EKFWrapper
{
public:
  /**
   * @brief Default constructor for EKFWrapper
   */
  EKFWrapper();

  /**
   * @brief Constructor for EKFWrapper with initial state and covariance.
   *
   * @param initial_state (State) The initial state vector.
   * @param initial_covariance (Covariance) The initial covariance matrix.
   * @param imu_noise (Eigen::Vector<double, 6>) The IMU noise vector.
   * @param accelerometer_noise_density (double) The accelerometer noise density.
   * @param gyroscope_noise_density (double) The gyroscope noise density.
   * @param accelerometer_random_walk (double) The accelerometer random walk.
   * @param gyroscope_random_walk (double) The gyroscope random walk.
   */
  EKFWrapper(
    State initial_state,
    Covariance initial_covariance,
    Eigen::Vector<double, 6> imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);

  /**
   * @brief EKFWrapper destructor
   */
  ~EKFWrapper();


  /**
   * @brief Reset the EKF with a new state and covariance.
   *
   * @param initial_state (State) The new initial state vector.
   * @param initial_covariance (Covariance) The new initial covariance.
   */
  void reset(
    const State & initial_state,
    const Covariance & initial_covariance);


  /**
   * @brief Set the IMU noise parameters.
   * @param imu_noise (Eigen::Vector<double, 6>) The IMU noise vector.
   * @param accelerometer_noise_density (double) The accelerometer noise density.
   * @param gyroscope_noise_density (double) The gyroscope noise density.
   * @param accelerometer_random_walk (double) The accelerometer random walk.
   * @param gyroscope_random_walk (double) The gyroscope random walk.
   * */
  void set_noise_parameters(
    const Eigen::Vector<double, 6> & imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);


  /**
   * @brief Set gravity vector.
   * @param gravity (Gravity) The gravity vector.
   */
  void set_gravity(const Gravity & gravity);


  /**
   * @brief Set map to odom transformation.
   * @param map_to_odom (Eigen::Matrix4d) The transformation matrix from map to odometry frame.
   */
  void set_map_to_odom(const Eigen::Matrix4d & map_to_odom);

  /**
   * @brief Set map to odom velocity.
   * @param map_to_odom_velocity (Eigen::Vector3d) The velocity of the map to odom frame.
   */
  void set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity);

  /**
   * @brief Get the current state.
   *
   * @return The current state vector.
   */
  State get_state();

  /**
   * @brief Set the current state.
   *
   * @param state (State) The new state vector.
   */
  void set_state(const State & state);


  /**
   * @brief Get the current state covariance.
   *
   * @return The current state covariance matrix.
   */
  Covariance get_state_covariance();


  /**
   * @brief Get the current map to odom transformation.
   *
   * @return The current map to odom transformation matrix.
   */
  Eigen::Matrix4d get_map_to_odom();


  /**
   * #brief Get the current map to odom velocity.
   * @return The current map to odom velocity vector.
   */
  Eigen::Vector3d get_map_to_odom_velocity();


  /**
     * @brief Get the gravity vector.
     *
     * @return The gravity vector.
     */
  Gravity get_gravity();


  /**
   * @brief Get the IMU noise vector.
   *
   * @return The IMU noise vector.
   */
  Eigen::Vector<double, 6> get_imu_noise();


  /**
   * @brief Get the noise parameters.
   *
   * @return The noise parameters as a vector.
   */
  Eigen::Vector<double, 4> get_noise_parameters();


  /**
   * @brief Compute the process noise covariance matrix.
   *
   * @param dt (double) The time step.
   * @return The process noise covariance matrix.
   */
  Covariance compute_process_noise_covariance(double dt);


  /**
   * @brief Pose to transform.
   * @param position (Eigen::Vector3d) The position vector.
   * @param euler_rpy (Eigen::Vector3d) The Euler angles (roll, pitch, yaw).
   * @return The transformation matrix.
   */
  static Eigen::Matrix4d pose_to_transform(
    const Eigen::Vector3d & position,
    const Eigen::Vector3d & euler_rpy);


  /**
   * @brief Compute map to odom transformation.
   * @param state (State) The current state vector.
   * @param new_state (State) The new state vector.
   * @param prev_map_to_odom (Eigen::Matrix4d) The previous map to odom transformation matrix.
   * @return The new map to odom transformation matrix.
   */
  static Eigen::Matrix4d compute_map_to_odom(
    const State & state,
    const State & new_state,
    const Eigen::Matrix4d & prev_map_to_odom);
  
  /**
   * @brief Compute map to odom velocity.
   * @param state (State) The current state vector.
   * @param new_state (State) The new state vector.
   * @param prev_map_to_odom_velocity (Eigen::Vector3d) The previous map to odom velocity vector.
   * @return The new map to odom velocity vector.
   */
  static Eigen::Vector3d compute_map_to_odom_velocity(
    const State & state,
    const State & new_state,
    const Eigen::Vector3d & prev_map_to_odom_velocity);


  /**
   * @brief Get the transformation from b to c from state T_a_c and T_a_b.
   * @param position_a_c (Eigen::Vector3d) The position of c in a.
   * @param rotation_a_c (Eigen::Vector3d) The rotation of c in a (Euler angles).
   * @param T_a_b (Eigen::Matrix4d) The transformation from a to b.
   * @return The transformation from b to c.
   */
  Eigen::Matrix4d get_T_b_c(
    Eigen::Vector3d position_a_c,
    Eigen::Vector3d rotation_a_c,
    Eigen::Matrix4d T_a_b);

  /**
   * @brief Get the transformation from a to c from state T_b_c and T_a_b.
   * @param position (Eigen::Vector3d) The position of c in b.
   * @param rotation (Eigen::Vector3d) The rotation of c in b (Euler angles).
   * @param T_a_b (Eigen::Matrix4d) The transformation from a to b.
   * @return The transformation from b to c.
   */
  Eigen::Matrix4d get_T_a_c(
    Eigen::Vector3d position_b_c,
    Eigen::Vector3d rotation_b_c,
    Eigen::Matrix4d T_a_b);


  /**
   * @brief Project a matrix to SO(3).
   * @param M (Eigen::Matrix3d) The matrix to project.
   * @return The projected matrix in SO(3).
   */
  static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d & M);


  /**
   * @brief Transform to pose.
   * @param (Eigen::Matrix4d) transform The transformation matrix.
   * @return The pose as a vector of size 7.
   */
  static Eigen::Vector<double, 7> transform_to_pose(const Eigen::Matrix4d & transform);


  /**
   * @brief Predict the next state.
   *
   * @param imu_measurement (Input) The IMU measurement vector.
   * @param dt (double) The time step.
   */
  void predict(
    const Input & imu_measurement,
    const double & dt);


  /**
   * @brief Update the state with a new pose measurement.
   *
   * @param z (PoseMeasurement) The measurement (pose) vector.
   * @param measurement_noise_covariance (PoseMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_pose(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Update the state with a new pose measurement from odometry.
   *
   * @param z (PoseMeasurement) The measurement (pose) vector.
   * @param measurement_noise_covariance (PoseMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_pose_odom(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Update the state with a new velocity measurement.
   *
   * @param z (VelocityMeasurement) The measurement (velocity) vector.
   * @param measurement_noise_covariance (VelocityMeasurementCovariance) The measurement noise covariance matrix.
   */
  void update_velocity(
    const VelocityMeasurement & z,
    const VelocityMeasurementCovariance & measurement_noise_covariance);


  /**
   * @brief Correct the state to be within valid bounds.
   * Ensures angles are within [-pi, pi].
   */
  void correct_state();

private:
  EKFData ekf_data_;   // EKF data structure
  Eigen::Vector<double, 6> imu_noise_;   // IMU noise vector
  double accelerometer_noise_density_;   // Accelerometer noise density
  double gyroscope_noise_density_;   // Gyroscope noise density
  double accelerometer_random_walk_;   // Accelerometer random walk
  double gyroscope_random_walk_;   // Gyroscope random walk
  Gravity acc_in_world;

  const double * arg_[predict_function_SZ_ARG];   // Arguments for the predict functionality
  double * res_[predict_function_SZ_RES];   // Results for the predict functionality
  const double * update_pose_arg_[update_pose_function_SZ_ARG];   // Arguments for the update pose functionality
  double * update_pose_res_[update_pose_function_SZ_RES];   // Results for the update pose functionality
  const double * update_velocity_arg_[update_velocity_function_SZ_ARG];   // Arguments for the update pose velocity functionality
  double * update_velocity_res_[update_velocity_function_SZ_RES];   // Results for the update pose velocity functionality


  /**
   * @brief Initialize the arguments and results for the C code interface.
   */
  void initialize_args_and_results();
};

} // namespace ekf

#endif // EKF__EKF_WRAPPER_HPP
