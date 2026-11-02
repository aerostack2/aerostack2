// Copyright 2026 Universidad Politécnica de Madrid
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
 * @file model.hpp
 *
 * Turns ground-truth poses into odometry that drifts like VIO or LiDAR
 * odometry.
 *
 * Only x, y, z and yaw drift, the directions such systems cannot observe.
 * Roll and pitch stay exact, since gravity makes them observable.
 *
 * Each step, the true displacement is measured in the heading frame (yaw
 * only, z up), corrupted with a scale error, a bias and white noise, then
 * rotated back with the estimated yaw. A yaw error therefore also bends the
 * path. The biases are random walks, so the error grows as sqrt(t) from
 * white noise and as t^1.5 from the biases.
 *
 * A linearised covariance is propagated alongside, and the reported
 * velocities carry the same scale and biases as the pose.
 *
 * Depends only on Eigen, so it can be unit tested without ROS or Gazebo.
 * The math is explained in MODEL.md, next to this file.
 */

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__MODEL_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <random>

namespace as2_gazebo_assets
{

/// When the odometry starts drifting. Until then it follows ground truth.
enum class DriftStart
{
  kSimulation,  ///< From the first sample.
  kTakeoff,     ///< Once the body rises takeoff_height above where it rested.
};

/// Drift settings. Noise values are standard deviations. The first four are
/// densities (std dev after 1 s), scaled by sqrt(dt) each step so the drift
/// does not depend on the update rate. The twist noises are per sample.
struct DriftOdometryParams
{
  /// White noise on each displacement [m/s/sqrt(Hz)]: position random walk.
  double linear_velocity_noise = 0.02;
  /// Random walk of the velocity bias [m/s^2/sqrt(Hz)]: slow position creep.
  double linear_bias_noise = 0.001;
  /// White noise on each yaw increment [rad/s/sqrt(Hz)]: yaw random walk.
  double yaw_rate_noise = 0.002;
  /// Random walk of the yaw rate bias [rad/s^2/sqrt(Hz)]: slow yaw creep.
  double yaw_rate_bias_noise = 0.00005;
  /// Fixed stretch of every displacement (0.01 = 1 % too long). Not in the covariance.
  double scale_error = 0.0;
  /// White noise on each reported linear velocity [m/s]. Does not reach the pose.
  double twist_linear_noise = 0.02;
  /// White noise on each reported angular velocity [rad/s]. Does not reach the pose.
  double twist_angular_noise = 0.01;
  /// When false, x and y follow ground truth, as with GPS or motion capture.
  bool drift_horizontal = true;
  /// When false, z follows ground truth, as with a barometer.
  bool drift_vertical = true;
  /// When false, yaw follows ground truth, as with a compass.
  bool drift_yaw = true;
  DriftStart drift_start = DriftStart::kSimulation;
  /// Climb above the lowest resting height that counts as takeoff [m].
  double takeoff_height = 0.1;
};

/// Position and orientation in the world frame.
struct Pose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/// Velocities in the body frame.
struct Twist
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

/// Covariance in ROS order (x, y, z, rot x, rot y, rot z). Row-major, so
/// data() has the layout of a ROS covariance array.
using Covariance6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

/// Drifted odometry, with the content of a nav_msgs/Odometry message.
struct Odometry
{
  Pose pose;
  Twist twist;
  Covariance6 pose_covariance = Covariance6::Zero();
  Covariance6 twist_covariance = Covariance6::Zero();
};

class DriftOdometryModel
{
private:
  // Error state: position (3), yaw (1), velocity bias (3), yaw rate bias (1).
  // The true biases are zero, so the biases are their own errors.
  static constexpr int kStateSize = 8;

  static constexpr int kPx = 0;
  static constexpr int kPy = 1;
  static constexpr int kPz = 2;
  static constexpr int kYaw = 3;
  static constexpr int kBx = 4;
  static constexpr int kBy = 5;
  static constexpr int kBz = 6;
  static constexpr int kBYaw = 7;

  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;

  // Variance floor for exact axes (roll, pitch, axes without drift, and all
  // axes before drift starts). Keeps the covariance positive definite.
  static constexpr double kExactVariance = 1e-6;

  DriftOdometryParams params_;

  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  bool initialized_ = false;
  bool drift_started_ = false;
  double lowest_z_ = 0.0;

  // Ground truth at the previous update.
  double last_time_ = 0.0;
  Pose last_true_pose_;

  // Drifted estimate. Yaw is kept as an offset from the true yaw.
  Eigen::Vector3d estimated_position_ = Eigen::Vector3d::Zero();
  double yaw_error_ = 0.0;

  // Random-walk biases. The velocity bias is in the heading frame.
  Eigen::Vector3d velocity_bias_ = Eigen::Vector3d::Zero();
  double yaw_rate_bias_ = 0.0;

  // Biases used in the last step, so the reported velocities integrate to
  // the pose drift.
  Eigen::Vector3d step_velocity_bias_ = Eigen::Vector3d::Zero();
  double step_yaw_rate_bias_ = 0.0;

  StateMatrix covariance_ = StateMatrix::Zero();

public:
  /// A seed of 0 picks a random seed.
  explicit DriftOdometryModel(const DriftOdometryParams & params, std::uint64_t seed = 0)
  : params_(params),
    rng_(seed == 0 ? std::random_device{}() : seed)
  {
  }

  /// Forget the estimate. The next update starts again from ground truth.
  void reset() {initialized_ = false;}

  bool initialized() const {return initialized_;}

  /// Current yaw error [rad]. Exposed for testing.
  double yawError() const {return yaw_error_;}

  /// Whether drift has started (see DriftOdometryParams::drift_start).
  bool driftStarted() const {return drift_started_;}

  /**
   * Feed one ground-truth sample and get the drifted odometry back.
   *
   * @param time Simulation time [s].
   * @param true_pose Ground-truth pose in the world frame.
   * @param true_twist Ground-truth velocities in the body frame.
   */
  Odometry update(double time, const Pose & true_pose, const Twist & true_twist)
  {
    // Start from ground truth on the first sample, and again after a
    // simulation reset (time went backwards).
    if (!initialized_ || time < last_time_) {
      initialize(time, true_pose);
      drift_started_ = params_.drift_start == DriftStart::kSimulation;
      lowest_z_ = true_pose.position.z();
      return makeOdometry(true_pose, true_twist);
    }

    // Until takeoff, follow ground truth and track where the body rests.
    if (!drift_started_) {
      lowest_z_ = std::min(lowest_z_, true_pose.position.z());
      if (true_pose.position.z() - lowest_z_ < params_.takeoff_height) {
        initialize(time, true_pose);
        return makeOdometry(true_pose, true_twist);
      }
      drift_started_ = true;
    }

    const double dt = time - last_time_;
    if (dt > 0.0) {
      propagate(dt, true_pose);
      last_time_ = time;
      last_true_pose_ = true_pose;
    }
    return makeOdometry(true_pose, true_twist);
  }

private:
  /// Yaw of an orientation (z of its z-y-x Euler angles) [rad].
  static double yawOf(const Eigen::Quaterniond & q)
  {
    return std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  }

  /// R(yaw): rotation about z, from the heading frame to the world frame.
  static Eigen::Quaterniond yawRotation(double yaw)
  {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  void initialize(double time, const Pose & true_pose)
  {
    estimated_position_ = true_pose.position;
    yaw_error_ = 0.0;
    velocity_bias_.setZero();
    yaw_rate_bias_ = 0.0;
    step_velocity_bias_.setZero();
    step_yaw_rate_bias_ = 0.0;
    covariance_.setZero();
    last_time_ = time;
    last_true_pose_ = true_pose;
    initialized_ = true;
  }

  double gaussian() {return normal_(rng_);}

  /// Three N(0, 1) draws in a fixed order (x, y, z), so a seed is reproducible.
  Eigen::Vector3d gaussian3()
  {
    const double x = gaussian();
    const double y = gaussian();
    const double z = gaussian();
    return Eigen::Vector3d(x, y, z);
  }

  /// One Euler-Maruyama step of the drift. Noise is scaled by sqrt(dt), so
  /// its variance sigma^2 dt adds up to sigma^2 t at any update rate.
  void propagate(double dt, const Pose & true_pose)
  {
    const double sqrt_dt = std::sqrt(dt);
    const double scale = 1.0 + params_.scale_error;

    // R(psi) with the true yaw, and R(psi + e) with the yaw the estimate believes.
    const double true_yaw = yawOf(last_true_pose_.orientation);
    const Eigen::Matrix3d r_true = yawRotation(true_yaw).toRotationMatrix();
    const Eigen::Matrix3d r_est = yawRotation(true_yaw + yaw_error_).toRotationMatrix();

    // Measure the step in the heading frame and corrupt it, then rotate it
    // back with the estimated yaw:
    //   h = s R(psi)^T dp + b dt + sigma_v sqrt(dt) n
    //   w = R(psi + e) h
    // Since R(psi + e) R(psi)^T = R(e), every true step is turned by e.
    const Eigen::Vector3d delta = true_pose.position - last_true_pose_.position;
    const Eigen::Vector3d measured = scale * r_true.transpose() * delta +
      velocity_bias_ * dt + params_.linear_velocity_noise * sqrt_dt * gaussian3();
    const Eigen::Vector3d step = r_est * measured;
    estimated_position_ += step;

    if (!params_.drift_horizontal) {
      estimated_position_.head<2>() = true_pose.position.head<2>();
    }
    if (!params_.drift_vertical) {
      estimated_position_.z() = true_pose.position.z();
    }

    propagateCovariance(dt, step, r_est);

    step_velocity_bias_ = velocity_bias_;
    step_yaw_rate_bias_ = yaw_rate_bias_;

    // Advance the random walks after the step, so the step used the old values.
    const Eigen::Vector3d bias_step = params_.linear_bias_noise * sqrt_dt * gaussian3();
    if (params_.drift_horizontal) {
      velocity_bias_.head<2>() += bias_step.head<2>();
    }
    if (params_.drift_vertical) {
      velocity_bias_.z() += bias_step.z();
    }
    if (params_.drift_yaw) {
      yaw_error_ += yaw_rate_bias_ * dt + params_.yaw_rate_noise * sqrt_dt * gaussian();
      yaw_rate_bias_ += params_.yaw_rate_bias_noise * sqrt_dt * gaussian();
    }
  }

  /// P = F P F^T + Q, linearised around this step.
  void propagateCovariance(double dt, const Eigen::Vector3d & step, const Eigen::Matrix3d & r_est)
  {
    StateMatrix f = StateMatrix::Identity();
    // A small yaw error e moves the step sideways by e (z x step).
    f.block<3, 1>(kPx, kYaw) = Eigen::Vector3d::UnitZ().cross(step);
    // The bias acts in the heading frame, so it enters through R(psi + e).
    f.block<3, 3>(kPx, kBx) = r_est * dt;
    f(kYaw, kBYaw) = dt;

    // Variance added by each sqrt(dt)-scaled draw this step.
    StateVector q;
    q.segment<3>(kPx).setConstant(std::pow(params_.linear_velocity_noise, 2) * dt);
    q(kYaw) = std::pow(params_.yaw_rate_noise, 2) * dt;
    q.segment<3>(kBx).setConstant(std::pow(params_.linear_bias_noise, 2) * dt);
    q(kBYaw) = std::pow(params_.yaw_rate_bias_noise, 2) * dt;

    covariance_ = f * covariance_ * f.transpose();
    covariance_.diagonal() += q;

    // Axes that follow ground truth carry no uncertainty.
    const auto clear = [this](std::initializer_list<int> states) {
        for (int index : states) {
          covariance_.row(index).setZero();
          covariance_.col(index).setZero();
        }
      };
    if (!params_.drift_horizontal) {
      clear({kPx, kPy, kBx, kBy});
    }
    if (!params_.drift_vertical) {
      clear({kPz, kBz});
    }
    if (!params_.drift_yaw) {
      clear({kYaw, kBYaw});
    }
  }

  Odometry makeOdometry(const Pose & true_pose, const Twist & true_twist)
  {
    Odometry odom;

    // The yaw error rotates about world z, so roll and pitch stay exact.
    odom.pose.position = estimated_position_;
    odom.pose.orientation = yawRotation(yaw_error_) * true_pose.orientation;

    makeTwist(true_pose, true_twist, odom);

    // Error state (x, y, z, yaw) to ROS order (x, y, z, rx, ry, rz).
    constexpr std::array<int, 4> kStateIndex{kPx, kPy, kPz, kYaw};
    constexpr std::array<int, 4> kRosIndex{0, 1, 2, 5};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        odom.pose_covariance(kRosIndex[i], kRosIndex[j]) =
          covariance_(kStateIndex[i], kStateIndex[j]);
      }
    }
    odom.pose_covariance.diagonal() = odom.pose_covariance.diagonal().cwiseMax(kExactVariance);
    odom.twist_covariance.diagonal() = odom.twist_covariance.diagonal().cwiseMax(kExactVariance);
    return odom;
  }

  /// Body-frame velocities with the same scale and biases as the pose, plus
  /// white noise, and their covariance.
  void makeTwist(const Pose & true_pose, const Twist & true_twist, Odometry & odom)
  {
    // Attitude = heading (yaw) * tilt (roll, pitch). The bias acts in the heading frame.
    const Eigen::Quaterniond & attitude = true_pose.orientation;
    const Eigen::Quaterniond tilt =  // body to heading
      yawRotation(yawOf(attitude)).inverse() * attitude;

    const double scale = 1.0 + params_.scale_error;
    Eigen::Vector3d v = tilt * true_twist.linear;  // heading frame
    if (params_.drift_horizontal) {
      v.head<2>() = scale * v.head<2>() + step_velocity_bias_.head<2>();
    }
    if (params_.drift_vertical) {
      v.z() = scale * v.z() + step_velocity_bias_.z();
    }
    v = tilt.inverse() * v;  // body frame

    // The yaw rate bias is about world z, expressed here in the body frame.
    Eigen::Vector3d w = true_twist.angular;
    if (params_.drift_yaw) {
      w += attitude.inverse() * Eigen::Vector3d(0.0, 0.0, step_yaw_rate_bias_);
    }

    const double sl = params_.twist_linear_noise;
    const double sa = params_.twist_angular_noise;
    odom.twist.linear = v + sl * gaussian3();
    odom.twist.angular = w + sa * gaussian3();

    // Linear: bias covariance rotated into the body frame, plus white noise.
    const Eigen::Matrix3d r_tilt = tilt.toRotationMatrix();
    odom.twist_covariance.topLeftCorner<3, 3>() =
      r_tilt.transpose() * covariance_.block<3, 3>(kBx, kBx) * r_tilt +
      sl * sl * Eigen::Matrix3d::Identity();
    // Angular: yaw rate bias variance along world z (in the body frame), plus white noise.
    const Eigen::Vector3d up = attitude.inverse() * Eigen::Vector3d::UnitZ();
    odom.twist_covariance.bottomRightCorner<3, 3>() =
      covariance_(kBYaw, kBYaw) * up * up.transpose() + sa * sa * Eigen::Matrix3d::Identity();
  }
};

}  // namespace as2_gazebo_assets

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__MODEL_HPP_
