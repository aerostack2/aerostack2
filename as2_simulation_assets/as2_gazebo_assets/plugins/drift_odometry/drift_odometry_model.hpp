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
 * @file drift_odometry_model.hpp
 *
 * Turns ground-truth poses into odometry that drifts over time, the way a
 * VIO or LiDAR odometry source does.
 *
 * Drift is modelled on the four degrees of freedom that such sources cannot
 * observe: position (x, y, z) and yaw. Roll and pitch are taken from ground
 * truth, since gravity keeps them observable in practice.
 *
 * Each step, the true displacement is expressed in the heading frame, then
 * corrupted with a scale error, a slowly wandering bias (random walk) and
 * white noise, and finally added to the estimate using the estimated yaw.
 * Yaw error grows the same way, with its own bias and white noise. Because
 * the estimated yaw is used to rotate each step, yaw error also turns into
 * position error, as it does on real systems.
 *
 * Horizontal position, altitude and yaw can each be kept exact, to mimic a
 * system that corrects them with another sensor (e.g. a barometer or a
 * compass).
 *
 * The pose covariance is propagated alongside with a linearised model of the
 * error state, so consumers such as an EKF get a consistent uncertainty.
 *
 * This header has no ROS or Gazebo dependencies so it can be unit tested on
 * its own.
 */

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFT_ODOMETRY_MODEL_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFT_ODOMETRY_MODEL_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <random>

namespace as2_gazebo_assets
{

/// Noise settings. Noise densities are continuous-time values, so the drift
/// does not depend on the rate at which ground truth arrives.
struct DriftOdometryParams
{
  /// White noise on the measured displacement [m/s/sqrt(Hz)].
  double linear_velocity_noise = 0.02;
  /// Random walk of the velocity bias [m/s^2/sqrt(Hz)].
  double linear_bias_noise = 0.001;
  /// White noise on the yaw increment [rad/s/sqrt(Hz)].
  double yaw_rate_noise = 0.002;
  /// Random walk of the yaw rate bias [rad/s^2/sqrt(Hz)].
  double yaw_rate_bias_noise = 0.00005;
  /// Relative scale error on distance travelled (0.01 = 1 % too long).
  double scale_error = 0.0;
  /// White noise added to the reported linear velocity [m/s].
  double twist_linear_noise = 0.02;
  /// White noise added to the reported angular velocity [rad/s].
  double twist_angular_noise = 0.01;
  /// Whether x and y drift. When false they follow ground truth exactly, and
  /// a yaw error no longer shifts the position.
  bool drift_horizontal = true;
  /// Whether z drifts. When false it follows ground truth exactly.
  bool drift_vertical = true;
  /// Whether yaw drifts. When false it follows ground truth exactly.
  bool drift_yaw = true;
};

struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose
{
  Vector3 position;
  Quaternion orientation;
};

/// Velocities expressed in the body frame.
struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

/// Row-major 6x6 covariance in ROS order: x, y, z, rot x, rot y, rot z.
using Covariance6 = std::array<double, 36>;

struct DriftOdometrySample
{
  Pose pose;
  Twist twist;
  Covariance6 pose_covariance{};
  Covariance6 twist_covariance{};
};

inline double yawFromQuaternion(const Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  Quaternion r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

class DriftOdometryModel
{
public:
  /// A seed of 0 picks a random seed.
  explicit DriftOdometryModel(const DriftOdometryParams & params, std::uint64_t seed = 0)
  : params_(params),
    rng_(seed == 0 ? std::random_device{}() : seed)
  {
  }

  /// Forget the current estimate. The next update starts again from ground truth.
  void reset() {initialized_ = false;}

  bool initialized() const {return initialized_;}

  /// Current yaw error of the estimate [rad]. Exposed for testing.
  double yawError() const {return yaw_error_;}

  /**
   * Feed one ground-truth sample and get the drifted odometry back.
   *
   * @param time Simulation time [s].
   * @param true_pose Ground-truth pose in the world frame.
   * @param true_twist Ground-truth velocities in the body frame.
   */
  DriftOdometrySample update(double time, const Pose & true_pose, const Twist & true_twist)
  {
    // Start from ground truth on the first sample, and again if the
    // simulation was reset and time went backwards.
    if (!initialized_ || time < last_time_) {
      initialize(time, true_pose);
      return makeSample(true_pose, true_twist);
    }

    const double dt = time - last_time_;
    if (dt > 0.0) {
      propagate(dt, true_pose);
      last_time_ = time;
      last_true_pose_ = true_pose;
    }
    return makeSample(true_pose, true_twist);
  }

private:
  // Error state: position (3), yaw (1), velocity bias (3), yaw rate bias (1).
  static constexpr int kStateSize = 8;
  static constexpr int kPx = 0;
  static constexpr int kPy = 1;
  static constexpr int kPz = 2;
  static constexpr int kYaw = 3;
  static constexpr int kBx = 4;
  static constexpr int kBy = 5;
  static constexpr int kBz = 6;
  static constexpr int kBYaw = 7;
  using Matrix = std::array<std::array<double, kStateSize>, kStateSize>;

  // Variance reported for axes that follow ground truth (roll, pitch and any
  // axis with drift turned off). Small, but keeps the covariance positive
  // definite.
  static constexpr double kExactVariance = 1e-6;

  void initialize(double time, const Pose & true_pose)
  {
    estimated_position_ = true_pose.position;
    yaw_error_ = 0.0;
    velocity_bias_ = Vector3{};
    yaw_rate_bias_ = 0.0;
    covariance_ = Matrix{};
    last_time_ = time;
    last_true_pose_ = true_pose;
    initialized_ = true;
  }

  double gaussian() {return normal_(rng_);}

  void propagate(double dt, const Pose & true_pose)
  {
    const double sqrt_dt = std::sqrt(dt);

    // True displacement, expressed in the heading frame of the previous pose.
    const double true_yaw = yawFromQuaternion(last_true_pose_.orientation);
    const double dx = true_pose.position.x - last_true_pose_.position.x;
    const double dy = true_pose.position.y - last_true_pose_.position.y;
    const double dz = true_pose.position.z - last_true_pose_.position.z;
    const double c = std::cos(true_yaw);
    const double s = std::sin(true_yaw);
    const double scale = 1.0 + params_.scale_error;
    const double sigma_v = params_.linear_velocity_noise * sqrt_dt;

    const double estimated_yaw = true_yaw + yaw_error_;
    const double ce = std::cos(estimated_yaw);
    const double se = std::sin(estimated_yaw);
    double wx = 0.0;
    double wy = 0.0;
    if (params_.drift_horizontal) {
      // Measured displacement in the heading frame, rotated into the
      // odometry frame with the estimated yaw.
      const double hx = scale * (c * dx + s * dy) + velocity_bias_.x * dt + sigma_v * gaussian();
      const double hy = scale * (-s * dx + c * dy) + velocity_bias_.y * dt + sigma_v * gaussian();
      wx = ce * hx - se * hy;
      wy = se * hx + ce * hy;
      estimated_position_.x += wx;
      estimated_position_.y += wy;
    } else {
      estimated_position_.x = true_pose.position.x;
      estimated_position_.y = true_pose.position.y;
    }
    if (params_.drift_vertical) {
      estimated_position_.z += scale * dz + velocity_bias_.z * dt + sigma_v * gaussian();
    } else {
      estimated_position_.z = true_pose.position.z;
    }

    propagateCovariance(dt, wx, wy, ce, se);

    // Advance yaw error and biases.
    const double sigma_b = params_.linear_bias_noise * sqrt_dt;
    if (params_.drift_horizontal) {
      velocity_bias_.x += sigma_b * gaussian();
      velocity_bias_.y += sigma_b * gaussian();
    }
    if (params_.drift_vertical) {
      velocity_bias_.z += sigma_b * gaussian();
    }
    if (params_.drift_yaw) {
      yaw_error_ += yaw_rate_bias_ * dt + params_.yaw_rate_noise * sqrt_dt * gaussian();
      yaw_rate_bias_ += params_.yaw_rate_bias_noise * sqrt_dt * gaussian();
    }
  }

  /// Remove a state from the covariance, for axes that follow ground truth.
  static void clearState(Matrix & m, int index)
  {
    for (int i = 0; i < kStateSize; ++i) {
      m[index][i] = 0.0;
      m[i][index] = 0.0;
    }
  }

  /// P = F P F^T + Q for the linearised error dynamics of one step.
  void propagateCovariance(double dt, double wx, double wy, double ce, double se)
  {
    Matrix f{};
    for (int i = 0; i < kStateSize; ++i) {
      f[i][i] = 1.0;
    }
    // A yaw error rotates this step's displacement.
    f[kPx][kYaw] = -wy;
    f[kPy][kYaw] = wx;
    // Velocity bias is in the heading frame.
    f[kPx][kBx] = ce * dt;
    f[kPx][kBy] = -se * dt;
    f[kPy][kBx] = se * dt;
    f[kPy][kBy] = ce * dt;
    f[kPz][kBz] = dt;
    f[kYaw][kBYaw] = dt;

    Matrix fp{};
    for (int i = 0; i < kStateSize; ++i) {
      for (int k = 0; k < kStateSize; ++k) {
        if (f[i][k] == 0.0) {
          continue;
        }
        for (int j = 0; j < kStateSize; ++j) {
          fp[i][j] += f[i][k] * covariance_[k][j];
        }
      }
    }
    Matrix next{};
    for (int i = 0; i < kStateSize; ++i) {
      for (int j = 0; j < kStateSize; ++j) {
        for (int k = 0; k < kStateSize; ++k) {
          next[i][j] += fp[i][k] * f[j][k];
        }
      }
    }

    const double q_position = params_.linear_velocity_noise * params_.linear_velocity_noise * dt;
    const double q_yaw = params_.yaw_rate_noise * params_.yaw_rate_noise * dt;
    const double q_bias = params_.linear_bias_noise * params_.linear_bias_noise * dt;
    const double q_yaw_bias = params_.yaw_rate_bias_noise * params_.yaw_rate_bias_noise * dt;
    next[kPx][kPx] += q_position;
    next[kPy][kPy] += q_position;
    next[kPz][kPz] += q_position;
    next[kYaw][kYaw] += q_yaw;
    next[kBx][kBx] += q_bias;
    next[kBy][kBy] += q_bias;
    next[kBz][kBz] += q_bias;
    next[kBYaw][kBYaw] += q_yaw_bias;

    if (!params_.drift_horizontal) {
      for (int index : {kPx, kPy, kBx, kBy}) {
        clearState(next, index);
      }
    }
    if (!params_.drift_vertical) {
      for (int index : {kPz, kBz}) {
        clearState(next, index);
      }
    }
    if (!params_.drift_yaw) {
      for (int index : {kYaw, kBYaw}) {
        clearState(next, index);
      }
    }

    covariance_ = next;
  }

  DriftOdometrySample makeSample(const Pose & true_pose, const Twist & true_twist)
  {
    DriftOdometrySample sample;

    sample.pose.position = estimated_position_;
    // Apply the yaw error as a rotation about the world z axis, which keeps
    // roll and pitch relative to gravity unchanged.
    const Quaternion yaw_error{std::cos(yaw_error_ / 2.0), 0.0, 0.0, std::sin(yaw_error_ / 2.0)};
    sample.pose.orientation = multiply(yaw_error, true_pose.orientation);

    const double sl = params_.twist_linear_noise;
    const double sa = params_.twist_angular_noise;
    sample.twist.linear.x = true_twist.linear.x + sl * gaussian();
    sample.twist.linear.y = true_twist.linear.y + sl * gaussian();
    sample.twist.linear.z = true_twist.linear.z + sl * gaussian();
    sample.twist.angular.x = true_twist.angular.x + sa * gaussian();
    sample.twist.angular.y = true_twist.angular.y + sa * gaussian();
    sample.twist.angular.z = true_twist.angular.z + sa * gaussian();

    // Map the error state (x, y, z, yaw) to ROS order (x, y, z, rx, ry, rz).
    constexpr std::array<int, 4> kStateIndex{kPx, kPy, kPz, kYaw};
    constexpr std::array<int, 4> kRosIndex{0, 1, 2, 5};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        sample.pose_covariance[kRosIndex[i] * 6 + kRosIndex[j]] =
          covariance_[kStateIndex[i]][kStateIndex[j]];
      }
    }
    sample.pose_covariance[3 * 6 + 3] = kExactVariance;
    sample.pose_covariance[4 * 6 + 4] = kExactVariance;
    if (!params_.drift_horizontal) {
      sample.pose_covariance[0 * 6 + 0] = kExactVariance;
      sample.pose_covariance[1 * 6 + 1] = kExactVariance;
    }
    if (!params_.drift_vertical) {
      sample.pose_covariance[2 * 6 + 2] = kExactVariance;
    }
    if (!params_.drift_yaw) {
      sample.pose_covariance[5 * 6 + 5] = kExactVariance;
    }

    for (int i = 0; i < 3; ++i) {
      sample.twist_covariance[i * 6 + i] = sl * sl;
      sample.twist_covariance[(i + 3) * 6 + (i + 3)] = sa * sa;
    }
    return sample;
  }

  DriftOdometryParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  bool initialized_ = false;
  double last_time_ = 0.0;
  Pose last_true_pose_;

  Vector3 estimated_position_;
  double yaw_error_ = 0.0;
  Vector3 velocity_bias_;
  double yaw_rate_bias_ = 0.0;
  Matrix covariance_{};
};

}  // namespace as2_gazebo_assets

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__DRIFT_ODOMETRY__DRIFT_ODOMETRY_MODEL_HPP_
