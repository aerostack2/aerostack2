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

#include "ekf/ekf_wrapper.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ekf/ekf_datatype.hpp"
#include <algorithm>

namespace ekf
{


EKFWrapper::EKFWrapper()
{
  // Initialize the EKF data
  ekf_data_ = EKFData();
  ekf_data_.map_to_odom = Eigen::Matrix4d::Identity();
  ekf_data_.map_to_odom_velocity = Eigen::Vector3d::Zero();
  imu_noise_ = Eigen::Vector<double, 6>::Zero();
  accelerometer_noise_density_ = 0.0;
  gyroscope_noise_density_ = 0.0;
  accelerometer_random_walk_ = 0.0;
  gyroscope_random_walk_ = 0.0;
  // Predict arguments and results for the C code interface
  initialize_args_and_results();
}


EKFWrapper::EKFWrapper(
  State initial_state,
  Covariance initial_covariance,
  Eigen::Vector<double, 6> imu_noise,
  double accelerometer_noise_density,
  double gyroscope_noise_density,
  double accelerometer_random_walk,
  double gyroscope_random_walk)
{
  // Initialize the EKF data with provided parameters
  ekf_data_ = EKFData();
  ekf_data_.state = initial_state;
  ekf_data_.covariance = initial_covariance;
  ekf_data_.map_to_odom = Eigen::Matrix4d::Identity();
  ekf_data_.map_to_odom_velocity = Eigen::Vector3d::Zero();
  imu_noise_ = imu_noise;
  accelerometer_noise_density_ = accelerometer_noise_density;
  gyroscope_noise_density_ = gyroscope_noise_density;
  accelerometer_random_walk_ = accelerometer_random_walk;
  gyroscope_random_walk_ = gyroscope_random_walk;
  // Initialize the arguments and results for the C code interface
  initialize_args_and_results();
}


EKFWrapper::~EKFWrapper()
{
  // Destructor logic if needed
}


void EKFWrapper::initialize_args_and_results()
{
  // Initialize the arguments and results for the C code interface
  arg_[0] = ekf_data_.state.data.data();
  arg_[2] = imu_noise_.data();
  arg_[4] = ekf_data_.covariance.data.data();
  arg_[6] = ekf_data_.gravity.data.data();
  res_[0] = ekf_data_.state.data.data();
  res_[1] = ekf_data_.covariance.data.data();
  res_[2] = acc_in_world.data.data();
  // Update pose arguments and results for the C code interface
  update_pose_arg_[0] = ekf_data_.state.data.data();
  update_pose_arg_[1] = imu_noise_.data();
  update_pose_arg_[3] = ekf_data_.covariance.data.data();
  update_pose_res_[0] = ekf_data_.state.data.data();
  update_pose_res_[1] = ekf_data_.covariance.data.data();
  // Update velocity arguments and results for the C code interface
  update_velocity_arg_[0] = ekf_data_.state.data.data();
  update_velocity_arg_[1] = imu_noise_.data();
  update_velocity_arg_[3] = ekf_data_.covariance.data.data();
  update_velocity_res_[0] = ekf_data_.state.data.data();
  update_velocity_res_[1] = ekf_data_.covariance.data.data();
}


void EKFWrapper::reset(
  const State & initial_state,
  const Covariance & initial_covariance)
{
  ekf_data_.state = initial_state;
  ekf_data_.covariance = initial_covariance;
}


void EKFWrapper::set_noise_parameters(
  const Eigen::Vector<double, 6> & imu_noise,
  double accelerometer_noise_density,
  double gyroscope_noise_density,
  double accelerometer_random_walk,
  double gyroscope_random_walk)
{
  imu_noise_ = imu_noise;
  accelerometer_noise_density_ = accelerometer_noise_density;
  gyroscope_noise_density_ = gyroscope_noise_density;
  accelerometer_random_walk_ = accelerometer_random_walk;
  gyroscope_random_walk_ = gyroscope_random_walk;

  arg_[2] = imu_noise_.data();
  update_pose_arg_[1] = imu_noise_.data();
  update_velocity_arg_[1] = imu_noise_.data();
}


void EKFWrapper::set_gravity(const Gravity & gravity)
{
  ekf_data_.gravity = gravity;
  arg_[6] = ekf_data_.gravity.data.data();
}


void EKFWrapper::set_map_to_odom(const Eigen::Matrix4d & map_to_odom)
{
  ekf_data_.map_to_odom = map_to_odom;
}


void EKFWrapper::set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity)
{
  ekf_data_.map_to_odom_velocity = map_to_odom_velocity;
}


State EKFWrapper::get_state()
{
  return ekf_data_.state;
}

void EKFWrapper::set_state(const State & state)
{
  ekf_data_.state.set(state.data);
}


Covariance EKFWrapper::get_state_covariance()
{
  return ekf_data_.covariance;
}


Eigen::Matrix4d EKFWrapper::get_map_to_odom()
{
  return ekf_data_.map_to_odom;
}


Eigen::Vector3d EKFWrapper::get_map_to_odom_velocity()
{
  return ekf_data_.map_to_odom_velocity;
}


Gravity EKFWrapper::get_gravity()
{
  return ekf_data_.gravity;
}


Eigen::Vector<double, 6> EKFWrapper::get_imu_noise()
{
  return imu_noise_;
}


Eigen::Vector<double, 4> EKFWrapper::get_noise_parameters()
{
  return Eigen::Vector<double, 4>(
    accelerometer_noise_density_,
    gyroscope_noise_density_,
    accelerometer_random_walk_,
    gyroscope_random_walk_);
}


Covariance EKFWrapper::compute_process_noise_covariance(
  double dt)
{
  Eigen::Matrix<double, 15, 15> process_noise_covariance =
    Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix3d q_pp =
    pow(accelerometer_noise_density_, 2) *
    pow(dt, 3) /
    3.0 *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_pv =
    pow(accelerometer_noise_density_, 2) *
    pow(dt, 2) /
    2.0 *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_vv =
    pow(accelerometer_noise_density_, 2) *
    dt *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_ww =
    pow(gyroscope_noise_density_, 2) *
    dt *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_baba =
    pow(accelerometer_random_walk_, 2) *
    dt *
    Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_bwbw =
    pow(gyroscope_random_walk_, 2) *
    dt *
    Eigen::Matrix3d::Identity();

  process_noise_covariance.block<3, 3>(0, 0) = q_pp;
  process_noise_covariance.block<3, 3>(0, 3) = q_pv;
  process_noise_covariance.block<3, 3>(3, 0) = q_pv;
  process_noise_covariance.block<3, 3>(3, 3) = q_vv;
  process_noise_covariance.block<3, 3>(6, 6) = q_ww;
  process_noise_covariance.block<3, 3>(9, 9) = q_baba;
  process_noise_covariance.block<3, 3>(12, 12) = q_bwbw;
  Covariance pnc = Covariance();
  std::array<double, Covariance::size> process_noise_covariance_array;
  for (std::size_t i = 0; i < Covariance::size; ++i) {
    process_noise_covariance_array[i] = process_noise_covariance(i / 15, i % 15);
  }
  pnc.set(process_noise_covariance_array);
  return pnc;
}


Eigen::Matrix4d EKFWrapper::pose_to_transform(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & euler_rpy)
{
  double roll = euler_rpy[0];
  double pitch = euler_rpy[1];
  double yaw = euler_rpy[2];
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  Eigen::Matrix3d R_x;
  R_x << 1, 0, 0,
    0, cr, -sr,
    0, sr, cr;
  Eigen::Matrix3d R_y;
  R_y << cp, 0, sp,
    0, 1, 0,
    -sp, 0, cp;
  Eigen::Matrix3d R_z;
  R_z << cy, -sy, 0,
    sy, cy, 0,
    0, 0, 1;
  Eigen::Matrix3d R = R_z * R_y * R_x;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = R;
  transform(0, 3) = position[0];
  transform(1, 3) = position[1];
  transform(2, 3) = position[2];
  return transform;
}


Eigen::Matrix4d EKFWrapper::compute_map_to_odom(
  const State & state,
  const State & new_state,
  const Eigen::Matrix4d & prev_map_to_odom)
{
  Eigen::Vector3d p_prev = Eigen::Vector3d(state.get_position().data());
  Eigen::Vector3d r_prev = Eigen::Vector3d(state.get_orientation().data());
  Eigen::Vector3d p_new = Eigen::Vector3d(new_state.get_position().data());
  Eigen::Vector3d r_new = Eigen::Vector3d(new_state.get_orientation().data());

  Eigen::Matrix4d T_map_base_prev =
    pose_to_transform(p_prev, r_prev);
  Eigen::Matrix4d T_base_map_prev = T_map_base_prev.inverse();
  Eigen::Matrix4d T_map_base_new =
    pose_to_transform(p_new, r_new);
  Eigen::Matrix4d delta = T_map_base_new * T_base_map_prev;

  Eigen::Matrix4d T_map_odom_new = delta * prev_map_to_odom;
  return T_map_odom_new;
}


Eigen::Vector3d EKFWrapper::compute_map_to_odom_velocity(
  const State & state,
  const State & new_state,
  const Eigen::Vector3d & prev_map_to_odom_velocity)
{
  Eigen::Vector3d v_prev = Eigen::Vector3d(state.get_velocity().data());
  Eigen::Vector3d v_new = Eigen::Vector3d(new_state.get_velocity().data());

  Eigen::Vector3d delta_v = v_new - v_prev;
  Eigen::Vector3d map_to_odom_velocity_new = prev_map_to_odom_velocity + delta_v;
  return map_to_odom_velocity_new;
}


Eigen::Matrix4d EKFWrapper::get_T_b_c(
  Eigen::Vector3d position_a_c,
  Eigen::Vector3d rotation_a_c,
  Eigen::Matrix4d T_a_b)
{
  // Eigen::Vector3d p = Eigen::Vector3d(state_T_a_c.get_position().data());
  // Eigen::Vector3d r = Eigen::Vector3d(state_T_a_c.get_orientation().data());
  Eigen::Matrix4d T_a_c = pose_to_transform(position_a_c, rotation_a_c);
  Eigen::Matrix4d T_b_c = T_a_b.inverse() * T_a_c;
  return T_b_c;
}


Eigen::Matrix4d EKFWrapper::get_T_a_c(
  Eigen::Vector3d position_b_c,
  Eigen::Vector3d rotation_b_c,
  Eigen::Matrix4d T_a_b)
{
  // Eigen::Vector3d p = Eigen::Vector3d(state_T_a_c.get_position().data());
  // Eigen::Vector3d r = Eigen::Vector3d(state_T_a_c.get_orientation().data());
  Eigen::Matrix4d T_b_c = pose_to_transform(position_b_c, rotation_b_c);
  Eigen::Matrix4d T_a_c = T_a_b * T_b_c;
  return T_a_c;
}


Eigen::Matrix3d EKFWrapper::projectToSO3(const Eigen::Matrix3d & M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  // Enforce det(R) = +1 (avoid reflections)
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }
  return R;
}


Eigen::Vector<double, 7> EKFWrapper::transform_to_pose(const Eigen::Matrix4d & transform)
{

  // Extract raw rotation and translation
  Eigen::Matrix3d M = transform.block<3, 3>(0, 0);
  Eigen::Vector3d t = transform.block<3, 1>(0, 3);

  // Project to nearest rotation to remove tiny scale/shear
  Eigen::Matrix3d R = projectToSO3(M);

  // Convert to quaternion (Eigen stores [x,y,z,w] in coeffs())
  Eigen::Quaterniond q(R);
  q.normalize();

  Eigen::Vector<double, 7> pose;
  pose[0] = t[0]; // x
  pose[1] = t[1]; // y
  pose[2] = t[2]; // z
  pose[3] = q.x(); // qx
  pose[4] = q.y(); // qy
  pose[5] = q.z(); // qz
  pose[6] = q.w(); // qw
  return pose;
}


void EKFWrapper::correct_state()
{
  // Ensure the orientation angles are within valid ranges
  double & roll = ekf_data_.state.data[State::ROLL];
  double & pitch = ekf_data_.state.data[State::PITCH];
  double & yaw = ekf_data_.state.data[State::YAW];

  // If any angle is outside the range [-pi, pi], wrap it around
  // roll = std::atan2(std::sin(roll), std::cos(roll));
  // pitch = std::atan2(std::sin(pitch), std::cos(pitch));
  // yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  roll = std::fmod(roll + M_PI, 2.0 * M_PI);
  if (roll < 0.0) {
    roll += 2.0 * M_PI;
  }
  roll -= M_PI;
  pitch = std::fmod(pitch + M_PI, 2.0 * M_PI);
  if (pitch < 0.0) {
    pitch += 2.0 * M_PI;
  }
  pitch -= M_PI;
  yaw = std::fmod(yaw + M_PI, 2.0 * M_PI);
  if (yaw < 0.0) {
    yaw += 2.0 * M_PI;
  }
  yaw -= M_PI;
}


void EKFWrapper::predict(
  const Input & input,
  const double & dt)
{
  Covariance process_noise_covariance =
    compute_process_noise_covariance(dt);

  arg_[1] = input.data.data();
  arg_[3] = &dt;
  arg_[5] = process_noise_covariance.data.data();

  predict_function(
    arg_,
    res_,
    nullptr,
    nullptr,
    0);
  correct_state();
}


void EKFWrapper::update_pose(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  update_pose_arg_[2] = z.data.data();
  update_pose_arg_[4] = measurement_noise_covariance.data.data();

  State prev_state = get_state();

  update_pose_function(
    update_pose_arg_,
    update_pose_res_,
    nullptr,
    nullptr,
    0);
  correct_state();

  // Update the map to odom Transformation
  set_map_to_odom(
    compute_map_to_odom(
      prev_state,
      get_state(),
      get_map_to_odom()));
  // Update the map to odom Velocity
  set_map_to_odom_velocity(
    compute_map_to_odom_velocity(
      prev_state,
      get_state(),
      get_map_to_odom_velocity()));
}


void EKFWrapper::update_pose_odom(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  update_pose_arg_[2] = z.data.data();
  update_pose_arg_[4] = measurement_noise_covariance.data.data();

  State prev_state = get_state();

  update_pose_function(
    update_pose_arg_,
    update_pose_res_,
    nullptr,
    nullptr,
    0);
  correct_state();
}


void EKFWrapper::update_velocity(
  const VelocityMeasurement & z,
  const VelocityMeasurementCovariance & measurement_noise_covariance)
{
  update_velocity_arg_[2] = z.data.data();
  update_velocity_arg_[4] = measurement_noise_covariance.data.data();

  State prev_state = get_state();
  Covariance prev_covariance = get_state_covariance();

  update_velocity_function(
    update_velocity_arg_,
    update_velocity_res_,
    nullptr,
    nullptr,
    0);
  correct_state(); // Correct the state angles

  // // Check mahalanobis distance to detect outliers
  // Eigen::Vector<double, 15> state_diff;
  // for (std::size_t i = 0; i < 15; ++i) {
  //   state_diff[i] = get_state().data[i] - prev_state.data[i];
  // }
  // // Check if the position difference is greater than 5 meters
  // if (state_diff.head<3>().norm() > 5.0) {
  //   // If so, revert to previous state and covariance
  //   reset(
  //     prev_state,
  //     prev_covariance);
  //   return;
  // }
  // // Check if the velocity difference is greater than 3 m/s
  // if (state_diff.segment<3>(3).norm() > 3.0) {
  //   // If so, revert to previous state and covariance
  //   reset(
  //     prev_state,
  //     prev_covariance);
  //   return;
  // }

  // Update the map to odom Transformation
  set_map_to_odom(
    compute_map_to_odom(
      prev_state,
      get_state(),
      get_map_to_odom()));
}


}  // namespace ekf
