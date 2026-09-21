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
* C++ interface to the EKF whose equations are generated with CasADi
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef EKF__EKF_WRAPPER_HPP_
#define EKF__EKF_WRAPPER_HPP_

#include <ekf/ekf_c_code.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ekf/ekf_datatype.hpp"

namespace ekf
{

/**
 * @brief Everything the filter carries from one step to the next
 */
struct EKFData
{
  State state;
  Covariance covariance;
  Gravity gravity;
  Eigen::Matrix4d map_to_odom = Eigen::Matrix4d::Identity();
  Eigen::Vector3d map_to_odom_velocity = Eigen::Vector3d::Zero();
};

/**
 * @brief The EKF: prediction from the IMU, correction from pose and velocity measurements
 *
 * The generated functions take their arguments and results as tables of raw pointers, and
 * those tables point into this object's own members. That is why the class can be neither
 * copied nor moved: a copy's tables would keep pointing into the original.
 */
class EKFWrapper
{
public:
  EKFWrapper();

  EKFWrapper(
    const State & initial_state,
    const Covariance & initial_covariance,
    const Eigen::Vector<double, 6> & imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);

  EKFWrapper(const EKFWrapper &) = delete;
  EKFWrapper & operator=(const EKFWrapper &) = delete;

  /**
   * @brief Restart from a state and covariance. map -> odom is left as it is.
   */
  void reset(const State & initial_state, const Covariance & initial_covariance);

  /**
   * @brief Set the IMU noise the process noise is built from.
   *
   * @param imu_noise Noise vector the generated functions take directly
   * @param accelerometer_noise_density Accelerometer white noise, per sqrt(Hz)
   * @param gyroscope_noise_density Gyroscope white noise, per sqrt(Hz)
   * @param accelerometer_random_walk Accelerometer bias drift, per sqrt(Hz)
   * @param gyroscope_random_walk Gyroscope bias drift, per sqrt(Hz)
   */
  void set_noise_parameters(
    const Eigen::Vector<double, 6> & imu_noise,
    double accelerometer_noise_density,
    double gyroscope_noise_density,
    double accelerometer_random_walk,
    double gyroscope_random_walk);

  void set_gravity(const Gravity & gravity);
  void set_map_to_odom(const Eigen::Matrix4d & map_to_odom);
  void set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity);
  void set_state(const State & state);

  const State & get_state() const {return ekf_data_.state;}
  const Covariance & get_state_covariance() const {return ekf_data_.covariance;}
  const Eigen::Matrix4d & get_map_to_odom() const {return ekf_data_.map_to_odom;}
  const Eigen::Vector3d & get_map_to_odom_velocity() const
  {
    return ekf_data_.map_to_odom_velocity;
  }
  const Gravity & get_gravity() const {return ekf_data_.gravity;}
  const Eigen::Vector<double, 6> & get_imu_noise() const {return imu_noise_;}

  /**
   * @brief The four noise densities: accelerometer and gyroscope noise, then their random walks.
   */
  Eigen::Vector<double, 4> get_noise_parameters() const;

  /**
   * @brief Process noise Q accumulated over one prediction step of length dt.
   */
  Covariance compute_process_noise_covariance(double dt) const;

  /**
   * @brief Homogeneous transform from a position and roll, pitch, yaw (Z-Y-X order).
   */
  static Eigen::Matrix4d pose_to_transform(
    const Eigen::Vector3d & position,
    const Eigen::Vector3d & euler_rpy);

  /**
   * @brief map -> odom after a correction, moved by exactly as much as the correction moved
   *        map -> base, so that odom -> base is left as it was.
   *
   * @param state State before the correction
   * @param new_state State after the correction
   * @param prev_map_to_odom map -> odom before the correction
   */
  static Eigen::Matrix4d compute_map_to_odom(
    const State & state,
    const State & new_state,
    const Eigen::Matrix4d & prev_map_to_odom);

  /**
   * @brief The velocity counterpart of @ref compute_map_to_odom.
   */
  static Eigen::Vector3d compute_map_to_odom_velocity(
    const State & state,
    const State & new_state,
    const Eigen::Vector3d & prev_map_to_odom_velocity);

  /**
   * @brief T_b_c from the pose of c in a and T_a_b.
   *
   * @param position_a_c Position of c in a
   * @param rotation_a_c Roll, pitch, yaw of c in a
   * @param T_a_b Transform from a to b
   */
  static Eigen::Matrix4d get_T_b_c(
    const Eigen::Vector3d & position_a_c,
    const Eigen::Vector3d & rotation_a_c,
    const Eigen::Matrix4d & T_a_b);

  /**
   * @brief T_a_c from the pose of c in b and T_a_b.
   *
   * @param position_b_c Position of c in b
   * @param rotation_b_c Roll, pitch, yaw of c in b
   * @param T_a_b Transform from a to b
   */
  static Eigen::Matrix4d get_T_a_c(
    const Eigen::Vector3d & position_b_c,
    const Eigen::Vector3d & rotation_b_c,
    const Eigen::Matrix4d & T_a_b);

  /**
   * @brief The rotation closest to a matrix that has drifted numerically from one.
   */
  static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d & M);

  /**
   * @brief Position and quaternion (x, y, z, qx, qy, qz, qw) of a homogeneous transform.
   */
  static Eigen::Vector<double, 7> transform_to_pose(const Eigen::Matrix4d & transform);

  /**
   * @brief Predict the state forward by dt with an IMU reading.
   */
  void predict(const Input & imu_measurement, double dt);

  /**
   * @brief Correct with a pose measurement, moving map -> odom by the correction.
   */
  void update_pose(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);

  /**
   * @brief Correct with a pose measurement, leaving map -> odom as it is.
   */
  void update_pose_odom(
    const PoseMeasurement & z,
    const PoseMeasurementCovariance & measurement_noise_covariance);

  /**
   * @brief Correct with a velocity measurement, leaving map -> odom as it is.
   *
   * A velocity says how the vehicle moves, not where the map is, so the correction is
   * absorbed by odom -> base.
   */
  void update_velocity(
    const VelocityMeasurement & z,
    const VelocityMeasurementCovariance & measurement_noise_covariance);

  /**
   * @brief Wrap roll, pitch and yaw to [-pi, pi).
   */
  void correct_state();

private:
  EKFData ekf_data_;
  Eigen::Vector<double, 6> imu_noise_ = Eigen::Vector<double, 6>::Zero();
  double accelerometer_noise_density_ = 0.0;
  double gyroscope_noise_density_ = 0.0;
  double accelerometer_random_walk_ = 0.0;
  double gyroscope_random_walk_ = 0.0;

  // Third output of the generated predict function, which nothing reads
  Gravity acc_in_world_;

  // Argument and result tables of the generated functions. The entries that never change
  // point into the members above; the rest are filled in on every call.
  const double * arg_[predict_function_SZ_ARG];
  double * res_[predict_function_SZ_RES];
  const double * update_pose_arg_[update_pose_function_SZ_ARG];
  double * update_pose_res_[update_pose_function_SZ_RES];
  const double * update_velocity_arg_[update_velocity_function_SZ_ARG];
  double * update_velocity_res_[update_velocity_function_SZ_RES];

  void initialize_args_and_results();
};

}  // namespace ekf

#endif  // EKF__EKF_WRAPPER_HPP_
