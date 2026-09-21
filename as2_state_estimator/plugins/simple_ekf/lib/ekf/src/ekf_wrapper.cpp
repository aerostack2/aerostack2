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
* @file ekf_wrapper.cpp
*
* C++ interface to the EKF whose equations are generated with CasADi
*
* @authors Rodrigo Da Silva Gómez
*/

#include "ekf/ekf_wrapper.hpp"

#include <cmath>

namespace ekf
{

namespace
{

double wrap_to_pi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

}  // namespace

EKFWrapper::EKFWrapper()
{
  initialize_args_and_results();
}

EKFWrapper::EKFWrapper(
  const State & initial_state,
  const Covariance & initial_covariance,
  const Eigen::Vector<double, 6> & imu_noise,
  double accelerometer_noise_density,
  double gyroscope_noise_density,
  double accelerometer_random_walk,
  double gyroscope_random_walk)
: EKFWrapper()
{
  reset(initial_state, initial_covariance);
  set_noise_parameters(
    imu_noise, accelerometer_noise_density, gyroscope_noise_density,
    accelerometer_random_walk, gyroscope_random_walk);
}

void EKFWrapper::initialize_args_and_results()
{
  arg_[0] = ekf_data_.state.data.data();
  arg_[2] = imu_noise_.data();
  arg_[4] = ekf_data_.covariance.data.data();
  arg_[6] = ekf_data_.gravity.data.data();
  res_[0] = ekf_data_.state.data.data();
  res_[1] = ekf_data_.covariance.data.data();
  res_[2] = acc_in_world_.data.data();

  update_pose_arg_[0] = ekf_data_.state.data.data();
  update_pose_arg_[1] = imu_noise_.data();
  update_pose_arg_[3] = ekf_data_.covariance.data.data();
  update_pose_res_[0] = ekf_data_.state.data.data();
  update_pose_res_[1] = ekf_data_.covariance.data.data();

  update_velocity_arg_[0] = ekf_data_.state.data.data();
  update_velocity_arg_[1] = imu_noise_.data();
  update_velocity_arg_[3] = ekf_data_.covariance.data.data();
  update_velocity_res_[0] = ekf_data_.state.data.data();
  update_velocity_res_[1] = ekf_data_.covariance.data.data();
}

void EKFWrapper::reset(const State & initial_state, const Covariance & initial_covariance)
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
}

void EKFWrapper::set_gravity(const Gravity & gravity)
{
  ekf_data_.gravity = gravity;
}

void EKFWrapper::set_map_to_odom(const Eigen::Matrix4d & map_to_odom)
{
  ekf_data_.map_to_odom = map_to_odom;
}

void EKFWrapper::set_map_to_odom_velocity(const Eigen::Vector3d & map_to_odom_velocity)
{
  ekf_data_.map_to_odom_velocity = map_to_odom_velocity;
}

void EKFWrapper::set_state(const State & state)
{
  ekf_data_.state = state;
}

Eigen::Vector<double, 4> EKFWrapper::get_noise_parameters() const
{
  return Eigen::Vector<double, 4>(
    accelerometer_noise_density_,
    gyroscope_noise_density_,
    accelerometer_random_walk_,
    gyroscope_random_walk_);
}

Covariance EKFWrapper::compute_process_noise_covariance(double dt) const
{
  const double accelerometer_variance = std::pow(accelerometer_noise_density_, 2);
  const double gyroscope_variance = std::pow(gyroscope_noise_density_, 2);
  const double accelerometer_bias_variance = std::pow(accelerometer_random_walk_, 2);
  const double gyroscope_bias_variance = std::pow(gyroscope_random_walk_, 2);
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  Covariance process_noise;
  Eigen::Map<Eigen::Matrix<double, Covariance::rows, Covariance::cols, Eigen::RowMajor>> q(
    process_noise.data.data());

  // White acceleration noise integrated into velocity and, once more, into position
  const Eigen::Matrix3d q_position_velocity =
    accelerometer_variance * std::pow(dt, 2) / 2.0 * identity;
  q.block<3, 3>(State::X, State::X) = accelerometer_variance * std::pow(dt, 3) / 3.0 * identity;
  q.block<3, 3>(State::X, State::VX) = q_position_velocity;
  q.block<3, 3>(State::VX, State::X) = q_position_velocity;
  q.block<3, 3>(State::VX, State::VX) = accelerometer_variance * dt * identity;
  q.block<3, 3>(State::ROLL, State::ROLL) = gyroscope_variance * dt * identity;
  q.block<3, 3>(State::ABX, State::ABX) = accelerometer_bias_variance * dt * identity;
  q.block<3, 3>(State::WBX, State::WBX) = gyroscope_bias_variance * dt * identity;

  return process_noise;
}

Eigen::Matrix4d EKFWrapper::pose_to_transform(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & euler_rpy)
{
  const double cr = std::cos(euler_rpy[0]);
  const double sr = std::sin(euler_rpy[0]);
  const double cp = std::cos(euler_rpy[1]);
  const double sp = std::sin(euler_rpy[1]);
  const double cy = std::cos(euler_rpy[2]);
  const double sy = std::sin(euler_rpy[2]);

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

  const Eigen::Matrix3d R = R_z * R_y * R_x;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = position;
  return transform;
}

Eigen::Matrix4d EKFWrapper::compute_map_to_odom(
  const State & state,
  const State & new_state,
  const Eigen::Matrix4d & prev_map_to_odom)
{
  const Eigen::Matrix4d T_map_base_prev = pose_to_transform(
    Eigen::Vector3d(state.get_position().data()),
    Eigen::Vector3d(state.get_orientation().data()));
  const Eigen::Matrix4d T_map_base_new = pose_to_transform(
    Eigen::Vector3d(new_state.get_position().data()),
    Eigen::Vector3d(new_state.get_orientation().data()));

  const Eigen::Matrix4d T_base_map_prev = T_map_base_prev.inverse();
  const Eigen::Matrix4d delta = T_map_base_new * T_base_map_prev;
  return delta * prev_map_to_odom;
}

Eigen::Vector3d EKFWrapper::compute_map_to_odom_velocity(
  const State & state,
  const State & new_state,
  const Eigen::Vector3d & prev_map_to_odom_velocity)
{
  const Eigen::Vector3d v_prev(state.get_velocity().data());
  const Eigen::Vector3d v_new(new_state.get_velocity().data());
  return prev_map_to_odom_velocity + (v_new - v_prev);
}

Eigen::Matrix4d EKFWrapper::get_T_b_c(
  const Eigen::Vector3d & position_a_c,
  const Eigen::Vector3d & rotation_a_c,
  const Eigen::Matrix4d & T_a_b)
{
  return T_a_b.inverse() * pose_to_transform(position_a_c, rotation_a_c);
}

Eigen::Matrix4d EKFWrapper::get_T_a_c(
  const Eigen::Vector3d & position_b_c,
  const Eigen::Vector3d & rotation_b_c,
  const Eigen::Matrix4d & T_a_b)
{
  return T_a_b * pose_to_transform(position_b_c, rotation_b_c);
}

Eigen::Matrix3d EKFWrapper::projectToSO3(const Eigen::Matrix3d & M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  // A determinant of -1 is a reflection, not a rotation
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }
  return R;
}

Eigen::Vector<double, 7> EKFWrapper::transform_to_pose(const Eigen::Matrix4d & transform)
{
  Eigen::Quaterniond q(projectToSO3(transform.block<3, 3>(0, 0)));
  q.normalize();

  Eigen::Vector<double, 7> pose;
  pose << transform.block<3, 1>(0, 3), q.x(), q.y(), q.z(), q.w();
  return pose;
}

void EKFWrapper::correct_state()
{
  for (const int angle : {State::ROLL, State::PITCH, State::YAW}) {
    ekf_data_.state.data[angle] = wrap_to_pi(ekf_data_.state.data[angle]);
  }
}

void EKFWrapper::predict(const Input & imu_measurement, double dt)
{
  const Covariance process_noise_covariance = compute_process_noise_covariance(dt);

  arg_[1] = imu_measurement.data.data();
  arg_[3] = &dt;
  arg_[5] = process_noise_covariance.data.data();

  predict_function(arg_, res_, nullptr, nullptr, 0);
  correct_state();
}

void EKFWrapper::update_pose(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  const State prev_state = ekf_data_.state;

  update_pose_odom(z, measurement_noise_covariance);

  set_map_to_odom(compute_map_to_odom(prev_state, ekf_data_.state, ekf_data_.map_to_odom));
  set_map_to_odom_velocity(
    compute_map_to_odom_velocity(prev_state, ekf_data_.state, ekf_data_.map_to_odom_velocity));
}

void EKFWrapper::update_pose_odom(
  const PoseMeasurement & z,
  const PoseMeasurementCovariance & measurement_noise_covariance)
{
  update_pose_arg_[2] = z.data.data();
  update_pose_arg_[4] = measurement_noise_covariance.data.data();

  update_pose_function(update_pose_arg_, update_pose_res_, nullptr, nullptr, 0);
  correct_state();
}

void EKFWrapper::update_velocity(
  const VelocityMeasurement & z,
  const VelocityMeasurementCovariance & measurement_noise_covariance)
{
  update_velocity_arg_[2] = z.data.data();
  update_velocity_arg_[4] = measurement_noise_covariance.data.data();

  update_velocity_function(update_velocity_arg_, update_velocity_res_, nullptr, nullptr, 0);
  correct_state();
}

}  // namespace ekf
