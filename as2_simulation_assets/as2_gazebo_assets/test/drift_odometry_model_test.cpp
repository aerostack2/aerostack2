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

#include <gtest/gtest.h>

#include <cmath>

#include "drift_odometry_model.hpp"

namespace
{

using as2_gazebo_assets::DriftOdometryModel;
using as2_gazebo_assets::DriftOdometryParams;
using as2_gazebo_assets::DriftOdometrySample;
using as2_gazebo_assets::Pose;
using as2_gazebo_assets::Quaternion;
using as2_gazebo_assets::Twist;

Quaternion fromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  return Quaternion{
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy};
}

double roll(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

double pitch(const Quaternion & q)
{
  return std::asin(2.0 * (q.w * q.y - q.z * q.x));
}

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// A drone flying a tilted circle while turning.
Pose circlePose(double t)
{
  Pose pose;
  pose.position.x = 5.0 * std::cos(0.2 * t);
  pose.position.y = 5.0 * std::sin(0.2 * t);
  pose.position.z = 2.0 + 0.5 * std::sin(0.1 * t);
  pose.orientation = fromRpy(0.1, -0.05, 0.2 * t + M_PI / 2.0);
  return pose;
}

DriftOdometryParams noiseless()
{
  DriftOdometryParams params;
  params.linear_velocity_noise = 0.0;
  params.linear_bias_noise = 0.0;
  params.yaw_rate_noise = 0.0;
  params.yaw_rate_bias_noise = 0.0;
  params.scale_error = 0.0;
  params.twist_linear_noise = 0.0;
  params.twist_angular_noise = 0.0;
  return params;
}

TEST(DriftOdometryModel, MatchesGroundTruthWithoutNoise)
{
  DriftOdometryModel model(noiseless(), 1);
  const double dt = 0.01;
  DriftOdometrySample sample;
  Pose truth;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(i * dt);
    sample = model.update(i * dt, truth, Twist{});
  }
  EXPECT_NEAR(sample.pose.position.x, truth.position.x, 1e-9);
  EXPECT_NEAR(sample.pose.position.y, truth.position.y, 1e-9);
  EXPECT_NEAR(sample.pose.position.z, truth.position.z, 1e-9);
  EXPECT_NEAR(std::abs(sample.pose.orientation.w), std::abs(truth.orientation.w), 1e-9);
  EXPECT_NEAR(sample.pose_covariance[0], 0.0, 1e-12);
}

TEST(DriftOdometryModel, ScaleErrorStretchesDistance)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 1);
  Pose truth;
  DriftOdometrySample sample;
  for (int i = 0; i <= 100; ++i) {
    truth.position.x = 0.1 * i;
    sample = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_NEAR(sample.pose.position.x, 10.0 * 1.05, 1e-9);
  EXPECT_NEAR(sample.pose.position.y, 0.0, 1e-9);
}

TEST(DriftOdometryModel, DriftsInPositionAndYawButKeepsRollAndPitch)
{
  DriftOdometryParams params;
  params.yaw_rate_noise = 0.01;
  DriftOdometryModel model(params, 42);
  const double dt = 0.01;
  DriftOdometrySample sample;
  Pose truth;
  for (int i = 0; i <= 12000; ++i) {
    truth = circlePose(i * dt);
    sample = model.update(i * dt, truth, Twist{});
  }
  const double position_error = std::hypot(
    sample.pose.position.x - truth.position.x,
    sample.pose.position.y - truth.position.y);
  EXPECT_GT(position_error, 1e-3);
  EXPECT_GT(std::abs(model.yawError()), 1e-4);

  // The yaw error is a rotation about the world z axis, so tilt is unchanged.
  EXPECT_NEAR(roll(sample.pose.orientation), roll(truth.orientation), 1e-9);
  EXPECT_NEAR(pitch(sample.pose.orientation), pitch(truth.orientation), 1e-9);
  const double yaw_est = as2_gazebo_assets::yawFromQuaternion(sample.pose.orientation);
  const double yaw_true = as2_gazebo_assets::yawFromQuaternion(truth.orientation);
  EXPECT_NEAR(wrap(yaw_est - yaw_true), wrap(model.yawError()), 1e-9);
}

TEST(DriftOdometryModel, RestartsFromGroundTruthWhenTimeGoesBack)
{
  DriftOdometryParams params;
  params.linear_velocity_noise = 0.5;
  DriftOdometryModel model(params, 7);
  for (int i = 0; i <= 1000; ++i) {
    model.update(0.01 * i, circlePose(0.01 * i), Twist{});
  }
  const Pose truth = circlePose(0.0);
  const DriftOdometrySample sample = model.update(0.0, truth, Twist{});
  EXPECT_DOUBLE_EQ(sample.pose.position.x, truth.position.x);
  EXPECT_DOUBLE_EQ(sample.pose.position.y, truth.position.y);
  EXPECT_DOUBLE_EQ(sample.pose_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
}

// Run many independent flights and check that the reported covariance
// matches the spread of the actual errors.
void expectCovarianceMatchesSpread(const DriftOdometryParams & params, double rate_hz)
{
  const int runs = 400;
  const double duration = 30.0;
  const int steps = static_cast<int>(duration * rate_hz);
  double sum_x2 = 0.0;
  double sum_z2 = 0.0;
  double sum_yaw2 = 0.0;
  DriftOdometrySample sample;
  for (int run = 0; run < runs; ++run) {
    DriftOdometryModel model(params, 1000 + run);
    Pose truth;
    for (int i = 0; i <= steps; ++i) {
      const double t = i / rate_hz;
      truth = circlePose(t);
      sample = model.update(t, truth, Twist{});
    }
    sum_x2 += std::pow(sample.pose.position.x - truth.position.x, 2);
    sum_z2 += std::pow(sample.pose.position.z - truth.position.z, 2);
    sum_yaw2 += std::pow(model.yawError(), 2);
  }
  // The covariance is deterministic for a given trajectory up to the noise
  // in the displacement, so the last run's value is representative.
  const double var_x = sample.pose_covariance[0];
  const double var_z = sample.pose_covariance[2 * 6 + 2];
  const double var_yaw = sample.pose_covariance[5 * 6 + 5];
  EXPECT_NEAR(sum_x2 / runs / var_x, 1.0, 0.25);
  if (params.drift_vertical && var_z > 0.0) {
    EXPECT_NEAR(sum_z2 / runs / var_z, 1.0, 0.25);
  }
  if (params.drift_yaw) {
    EXPECT_NEAR(sum_yaw2 / runs / var_yaw, 1.0, 0.25);
  }
}

DriftOdometryParams allNoiseSources()
{
  DriftOdometryParams params;
  params.linear_velocity_noise = 0.05;
  params.linear_bias_noise = 0.005;
  params.yaw_rate_noise = 0.005;
  params.yaw_rate_bias_noise = 0.0005;
  return params;
}

TEST(DriftOdometryModel, CovarianceMatchesSpreadAt100Hz)
{
  expectCovarianceMatchesSpread(allNoiseSources(), 100.0);
}

TEST(DriftOdometryModel, CovarianceMatchesSpreadAt25Hz)
{
  expectCovarianceMatchesSpread(allNoiseSources(), 25.0);
}

// With only yaw noise, all position error comes from flying with a wrong
// heading, so this checks that yaw uncertainty feeds the position covariance.
TEST(DriftOdometryModel, CovarianceTracksPositionErrorFromYawDrift)
{
  DriftOdometryParams params = noiseless();
  params.yaw_rate_noise = 0.01;
  params.yaw_rate_bias_noise = 0.001;
  expectCovarianceMatchesSpread(params, 100.0);
}

// Strong noise on every source, so any axis that is meant to drift does.
DriftOdometryParams strongNoise()
{
  DriftOdometryParams params;
  params.linear_velocity_noise = 0.1;
  params.linear_bias_noise = 0.01;
  params.yaw_rate_noise = 0.01;
  params.yaw_rate_bias_noise = 0.001;
  params.scale_error = 0.05;
  return params;
}

/// Fly the test circle for a minute and return the last sample and truth.
DriftOdometrySample flyCircle(DriftOdometryModel & model, Pose & truth)
{
  DriftOdometrySample sample;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(0.01 * i);
    sample = model.update(0.01 * i, truth, Twist{});
  }
  return sample;
}

double yawError(const DriftOdometrySample & sample, const Pose & truth)
{
  return wrap(
    as2_gazebo_assets::yawFromQuaternion(sample.pose.orientation) -
    as2_gazebo_assets::yawFromQuaternion(truth.orientation));
}

double horizontalError(const DriftOdometrySample & sample, const Pose & truth)
{
  return std::hypot(
    sample.pose.position.x - truth.position.x,
    sample.pose.position.y - truth.position.y);
}

constexpr double kExactVariance = 1e-6;

TEST(DriftOdometryModel, HorizontalDriftOffKeepsXYExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  DriftOdometryModel model(params, 11);
  Pose truth;
  const DriftOdometrySample sample = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(sample.pose.position.x, truth.position.x);
  EXPECT_DOUBLE_EQ(sample.pose.position.y, truth.position.y);
  EXPECT_DOUBLE_EQ(sample.pose_covariance[0 * 6 + 0], kExactVariance);
  EXPECT_DOUBLE_EQ(sample.pose_covariance[1 * 6 + 1], kExactVariance);
  // Yaw still drifts, but no longer leaks into the position covariance.
  EXPECT_DOUBLE_EQ(sample.pose_covariance[0 * 6 + 5], 0.0);
  EXPECT_GT(std::abs(sample.pose.position.z - truth.position.z), 1e-3);
  EXPECT_GT(std::abs(yawError(sample, truth)), 1e-4);
}

TEST(DriftOdometryModel, VerticalDriftOffKeepsZExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_vertical = false;
  DriftOdometryModel model(params, 12);
  Pose truth;
  const DriftOdometrySample sample = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(sample.pose.position.z, truth.position.z);
  EXPECT_DOUBLE_EQ(sample.pose_covariance[2 * 6 + 2], kExactVariance);
  EXPECT_GT(horizontalError(sample, truth), 1e-3);
  EXPECT_GT(std::abs(yawError(sample, truth)), 1e-4);
}

TEST(DriftOdometryModel, YawDriftOffKeepsYawExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_yaw = false;
  DriftOdometryModel model(params, 13);
  Pose truth;
  const DriftOdometrySample sample = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
  EXPECT_NEAR(yawError(sample, truth), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(sample.pose_covariance[5 * 6 + 5], kExactVariance);
  EXPECT_GT(horizontalError(sample, truth), 1e-3);
  EXPECT_GT(std::abs(sample.pose.position.z - truth.position.z), 1e-3);
}

TEST(DriftOdometryModel, AllDriftOffMatchesGroundTruth)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 14);
  Pose truth;
  const DriftOdometrySample sample = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(sample.pose.position.x, truth.position.x);
  EXPECT_DOUBLE_EQ(sample.pose.position.y, truth.position.y);
  EXPECT_DOUBLE_EQ(sample.pose.position.z, truth.position.z);
  EXPECT_NEAR(yawError(sample, truth), 0.0, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(sample.pose_covariance[i * 6 + i], kExactVariance);
  }
}

// The covariance must stay consistent when some axes are exact.
TEST(DriftOdometryModel, CovarianceMatchesSpreadWithOnlyHorizontalDrift)
{
  DriftOdometryParams params = allNoiseSources();
  params.drift_vertical = false;
  params.drift_yaw = false;
  expectCovarianceMatchesSpread(params, 100.0);
}

}  // namespace
