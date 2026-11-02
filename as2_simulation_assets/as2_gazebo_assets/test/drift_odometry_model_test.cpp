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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

#include "model.hpp"

namespace
{

using as2_gazebo_assets::DriftOdometryModel;
using as2_gazebo_assets::DriftStart;
using as2_gazebo_assets::DriftOdometryParams;
using as2_gazebo_assets::Odometry;
using as2_gazebo_assets::Pose;
using as2_gazebo_assets::Twist;

// Smallest variance the model reports on any axis.
constexpr double kExactVariance = 1e-6;

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Pose from a position and z-y-x Euler angles (roll, pitch, yaw).
Pose makePose(double x, double y, double z, double roll, double pitch, double yaw)
{
  Pose pose;
  pose.position = Eigen::Vector3d(x, y, z);
  pose.orientation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  return pose;
}

// z-y-x Euler angles of an orientation.
double roll(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.x() + q.y() * q.z()),
    1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
}

double pitch(const Eigen::Quaterniond & q)
{
  return std::asin(std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
}

double yaw(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// A drone flying a tilted circle while turning.
Pose circlePose(double t)
{
  return makePose(
    5.0 * std::cos(0.2 * t), 5.0 * std::sin(0.2 * t), 2.0 + 0.5 * std::sin(0.1 * t),
    0.1, -0.05, 0.2 * t + M_PI / 2.0);
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
  Odometry odom;
  Pose truth;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(i * dt);
    odom = model.update(i * dt, truth, Twist{});
  }
  EXPECT_NEAR(odom.pose.position.x(), truth.position.x(), 1e-9);
  EXPECT_NEAR(odom.pose.position.y(), truth.position.y(), 1e-9);
  EXPECT_NEAR(odom.pose.position.z(), truth.position.z(), 1e-9);
  EXPECT_NEAR(std::abs(odom.pose.orientation.w()), std::abs(truth.orientation.w()), 1e-9);
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
}

TEST(DriftOdometryModel, ScaleErrorStretchesDistance)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 1);
  Pose truth;
  Odometry odom;
  for (int i = 0; i <= 100; ++i) {
    truth.position.x() = 0.1 * i;
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_NEAR(odom.pose.position.x(), 10.0 * 1.05, 1e-9);
  EXPECT_NEAR(odom.pose.position.y(), 0.0, 1e-9);
}

TEST(DriftOdometryModel, DriftsInPositionAndYawButKeepsRollAndPitch)
{
  DriftOdometryParams params;
  params.yaw_rate_noise = 0.01;
  DriftOdometryModel model(params, 42);
  const double dt = 0.01;
  Odometry odom;
  Pose truth;
  for (int i = 0; i <= 12000; ++i) {
    truth = circlePose(i * dt);
    odom = model.update(i * dt, truth, Twist{});
  }
  const double position_error = std::hypot(
    odom.pose.position.x() - truth.position.x(),
    odom.pose.position.y() - truth.position.y());
  EXPECT_GT(position_error, 1e-3);
  EXPECT_GT(std::abs(model.yawError()), 1e-4);

  // The yaw error is a rotation about the world z axis, so tilt is unchanged.
  EXPECT_NEAR(roll(odom.pose.orientation), roll(truth.orientation), 1e-9);
  EXPECT_NEAR(pitch(odom.pose.orientation), pitch(truth.orientation), 1e-9);
  const double yaw_error = yaw(odom.pose.orientation) - yaw(truth.orientation);
  EXPECT_NEAR(wrap(yaw_error), wrap(model.yawError()), 1e-9);
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
  const Odometry odom = model.update(0.0, truth, Twist{});
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), truth.position.y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
}

// Run many independent flights and check that the reported covariance
// matches the spread of the actual errors.
void expectCovarianceMatchesSpread(const DriftOdometryParams & params, double rate_hz)
{
  const int runs = 400;
  const double duration = 10.0;
  const int steps = static_cast<int>(duration * rate_hz);
  double sum_x2 = 0.0;
  double sum_z2 = 0.0;
  double sum_yaw2 = 0.0;
  Odometry odom;
  for (int run = 0; run < runs; ++run) {
    DriftOdometryModel model(params, 1000 + run);
    Pose truth;
    for (int i = 0; i <= steps; ++i) {
      const double t = i / rate_hz;
      truth = circlePose(t);
      odom = model.update(t, truth, Twist{});
    }
    sum_x2 += std::pow(odom.pose.position.x() - truth.position.x(), 2);
    sum_z2 += std::pow(odom.pose.position.z() - truth.position.z(), 2);
    sum_yaw2 += std::pow(model.yawError(), 2);
  }
  // The covariance is deterministic for a given trajectory up to the noise
  // in the displacement, so the last run's value is representative.
  const double var_x = odom.pose_covariance(0, 0);
  const double var_z = odom.pose_covariance(2, 2);
  const double var_yaw = odom.pose_covariance(5, 5);
  EXPECT_NEAR(sum_x2 / runs / var_x, 1.0, 0.25);
  if (params.drift_vertical && var_z > kExactVariance) {
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

/// Fly the test circle for a minute and return the last odometry and truth.
Odometry flyCircle(DriftOdometryModel & model, Pose & truth)
{
  Odometry odom;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  return odom;
}

double yawError(const Odometry & odom, const Pose & truth)
{
  return wrap(yaw(odom.pose.orientation) - yaw(truth.orientation));
}

double horizontalError(const Odometry & odom, const Pose & truth)
{
  return std::hypot(
    odom.pose.position.x() - truth.position.x(),
    odom.pose.position.y() - truth.position.y());
}

TEST(DriftOdometryModel, HorizontalDriftOffKeepsXYExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  DriftOdometryModel model(params, 11);
  Pose truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), truth.position.y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
  EXPECT_DOUBLE_EQ(odom.pose_covariance(1, 1), kExactVariance);
  // Yaw still drifts, but no longer leaks into the position covariance.
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 5), 0.0);
  EXPECT_GT(std::abs(odom.pose.position.z() - truth.position.z()), 1e-3);
  EXPECT_GT(std::abs(yawError(odom, truth)), 1e-4);
}

TEST(DriftOdometryModel, VerticalDriftOffKeepsZExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_vertical = false;
  DriftOdometryModel model(params, 12);
  Pose truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.position.z(), truth.position.z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(2, 2), kExactVariance);
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(std::abs(yawError(odom, truth)), 1e-4);
}

TEST(DriftOdometryModel, YawDriftOffKeepsYawExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_yaw = false;
  DriftOdometryModel model(params, 13);
  Pose truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
  EXPECT_NEAR(yawError(odom, truth), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(odom.pose_covariance(5, 5), kExactVariance);
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(std::abs(odom.pose.position.z() - truth.position.z()), 1e-3);
}

TEST(DriftOdometryModel, AllDriftOffMatchesGroundTruth)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 14);
  Pose truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), truth.position.y());
  EXPECT_DOUBLE_EQ(odom.pose.position.z(), truth.position.z());
  EXPECT_NEAR(yawError(odom, truth), 0.0, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(odom.pose_covariance(i, i), kExactVariance);
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

DriftOdometryParams biasOnly()
{
  DriftOdometryParams params = noiseless();
  params.linear_bias_noise = 0.01;
  params.yaw_rate_bias_noise = 0.001;
  return params;
}

// Hovering in place with a tilt, so body and heading frames differ.
Pose hoverPose()
{
  return makePose(1.0, -2.0, 1.5, 0.2, -0.15, 0.8);
}

// While hovering, all drift comes from the biases, so integrating the
// reported velocities must reproduce the pose drift.
TEST(DriftOdometryModel, IntegratedVelocityMatchesPoseDrift)
{
  DriftOdometryParams params = biasOnly();
  params.drift_yaw = false;  // keep the heading exact so the integral is simple
  DriftOdometryModel model(params, 21);
  const Pose truth = hoverPose();
  const double dt = 0.01;
  Eigen::Vector3d integrated = Eigen::Vector3d::Zero();
  Odometry odom;
  for (int i = 0; i <= 6000; ++i) {
    odom = model.update(i * dt, truth, Twist{});
    if (i > 0) {
      integrated += truth.orientation * odom.twist.linear * dt;
    }
  }
  const Eigen::Vector3d drift = odom.pose.position - truth.position;
  EXPECT_GT(std::abs(drift.x()), 0.05);
  EXPECT_NEAR(integrated.x(), drift.x(), 1e-9);
  EXPECT_NEAR(integrated.y(), drift.y(), 1e-9);
  EXPECT_NEAR(integrated.z(), drift.z(), 1e-9);
}

TEST(DriftOdometryModel, IntegratedYawRateMatchesYawDrift)
{
  DriftOdometryParams params = biasOnly();
  params.linear_bias_noise = 0.0;
  DriftOdometryModel model(params, 22);
  const Pose truth = hoverPose();
  const double dt = 0.01;
  double integrated = 0.0;
  for (int i = 0; i <= 6000; ++i) {
    const Odometry odom = model.update(i * dt, truth, Twist{});
    if (i > 0) {
      // Yaw rate is the world z component of the angular velocity.
      integrated += (truth.orientation * odom.twist.angular).z() * dt;
    }
  }
  EXPECT_GT(std::abs(model.yawError()), 0.01);
  EXPECT_NEAR(integrated, model.yawError(), 1e-9);
}

TEST(DriftOdometryModel, ScaleErrorStretchesVelocity)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 23);
  Twist twist;
  twist.linear = Eigen::Vector3d(1.0, -0.5, 0.2);
  twist.angular = Eigen::Vector3d(0.1, 0.0, 0.3);
  const Odometry odom = model.update(0.0, hoverPose(), twist);
  EXPECT_NEAR(odom.twist.linear.x(), 1.05, 1e-12);
  EXPECT_NEAR(odom.twist.linear.y(), -0.525, 1e-12);
  EXPECT_NEAR(odom.twist.linear.z(), 0.21, 1e-12);
  EXPECT_NEAR(odom.twist.angular.z(), 0.3, 1e-12);
}

TEST(DriftOdometryModel, VelocityFollowsTruthOnAxesWithoutDrift)
{
  DriftOdometryParams params = biasOnly();
  params.scale_error = 0.05;
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 24);
  Twist twist;
  twist.linear = Eigen::Vector3d(1.0, -0.5, 0.2);
  twist.angular = Eigen::Vector3d(0.1, 0.0, 0.3);
  Odometry odom;
  for (int i = 0; i <= 1000; ++i) {
    odom = model.update(0.01 * i, hoverPose(), twist);
  }
  EXPECT_NEAR(odom.twist.linear.x(), 1.0, 1e-12);
  EXPECT_NEAR(odom.twist.linear.y(), -0.5, 1e-12);
  EXPECT_NEAR(odom.twist.linear.z(), 0.2, 1e-12);
  EXPECT_NEAR(odom.twist.angular.z(), 0.3, 1e-12);
}

// The reported twist covariance must match the spread of the twist error.
TEST(DriftOdometryModel, TwistCovarianceMatchesSpread)
{
  DriftOdometryParams params = allNoiseSources();
  params.twist_linear_noise = 0.02;
  params.twist_angular_noise = 0.005;
  const int runs = 400;
  const Pose truth = hoverPose();
  double sum_vx2 = 0.0;
  double sum_vz2 = 0.0;
  double sum_wz2 = 0.0;
  Odometry odom;
  for (int run = 0; run < runs; ++run) {
    DriftOdometryModel model(params, 3000 + run);
    for (int i = 0; i <= 1000; ++i) {
      odom = model.update(0.01 * i, truth, Twist{});
    }
    sum_vx2 += odom.twist.linear.x() * odom.twist.linear.x();
    sum_vz2 += odom.twist.linear.z() * odom.twist.linear.z();
    sum_wz2 += odom.twist.angular.z() * odom.twist.angular.z();
  }
  EXPECT_NEAR(sum_vx2 / runs / odom.twist_covariance(0, 0), 1.0, 0.25);
  EXPECT_NEAR(sum_vz2 / runs / odom.twist_covariance(2, 2), 1.0, 0.25);
  EXPECT_NEAR(sum_wz2 / runs / odom.twist_covariance(5, 5), 1.0, 0.25);
}

/// Rest on the ground for 10 s, then climb to 2 m and fly the circle.
Pose parkThenFly(double t)
{
  if (t < 10.0) {
    Pose pose = circlePose(0.0);
    pose.position.z() = 0.05;
    return pose;
  }
  Pose pose = circlePose(t - 10.0);
  pose.position.z() = std::min(0.05 + (t - 10.0), 2.0);
  return pose;
}

TEST(DriftOdometryModel, DriftStartsAtTakeoff)
{
  DriftOdometryParams params = strongNoise();
  params.drift_start = DriftStart::kTakeoff;
  DriftOdometryModel model(params, 31);
  Odometry odom;
  Pose truth;
  for (int i = 0; i <= 1000; ++i) {  // parked
    truth = parkThenFly(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.z(), truth.position.z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);

  for (int i = 1001; i <= 7000; ++i) {  // takeoff and flight
    truth = parkThenFly(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_TRUE(model.driftStarted());
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(odom.pose_covariance(0, 0), kExactVariance);
}

TEST(DriftOdometryModel, DriftStartsWithSimulationByDefault)
{
  DriftOdometryModel model(strongNoise(), 32);
  Odometry odom;
  Pose truth;
  for (int i = 0; i <= 1000; ++i) {  // parked
    truth = parkThenFly(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_TRUE(model.driftStarted());
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
}

TEST(DriftOdometryModel, ResetWaitsForTakeoffAgain)
{
  DriftOdometryParams params = strongNoise();
  params.drift_start = DriftStart::kTakeoff;
  DriftOdometryModel model(params, 33);
  for (int i = 0; i <= 3000; ++i) {
    model.update(0.01 * i, parkThenFly(0.01 * i), Twist{});
  }
  ASSERT_TRUE(model.driftStarted());
  const Pose truth = parkThenFly(0.0);
  const Odometry odom = model.update(0.0, truth, Twist{});
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
}

}  // namespace
