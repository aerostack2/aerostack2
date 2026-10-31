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

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "model.hpp"

namespace
{

using as2_gazebo_assets::DriftOdometryModel;
using as2_gazebo_assets::DriftStart;
using as2_gazebo_assets::DriftOdometryParams;
using as2_gazebo_assets::Odometry;
using as2_gazebo_assets::Twist;
using gz::math::Pose3d;
using gz::math::Vector3d;

// Smallest variance the model reports on any axis.
constexpr double kExactVariance = 1e-6;

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// A drone flying a tilted circle while turning.
Pose3d circlePose(double t)
{
  return Pose3d(
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
  Pose3d truth;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(i * dt);
    odom = model.update(i * dt, truth, Twist{});
  }
  EXPECT_NEAR(odom.pose.Pos().X(), truth.Pos().X(), 1e-9);
  EXPECT_NEAR(odom.pose.Pos().Y(), truth.Pos().Y(), 1e-9);
  EXPECT_NEAR(odom.pose.Pos().Z(), truth.Pos().Z(), 1e-9);
  EXPECT_NEAR(std::abs(odom.pose.Rot().W()), std::abs(truth.Rot().W()), 1e-9);
  EXPECT_DOUBLE_EQ(odom.pose_covariance[0], kExactVariance);
}

TEST(DriftOdometryModel, ScaleErrorStretchesDistance)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 1);
  Pose3d truth;
  Odometry odom;
  for (int i = 0; i <= 100; ++i) {
    truth.Pos().X() = 0.1 * i;
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_NEAR(odom.pose.Pos().X(), 10.0 * 1.05, 1e-9);
  EXPECT_NEAR(odom.pose.Pos().Y(), 0.0, 1e-9);
}

TEST(DriftOdometryModel, DriftsInPositionAndYawButKeepsRollAndPitch)
{
  DriftOdometryParams params;
  params.yaw_rate_noise = 0.01;
  DriftOdometryModel model(params, 42);
  const double dt = 0.01;
  Odometry odom;
  Pose3d truth;
  for (int i = 0; i <= 12000; ++i) {
    truth = circlePose(i * dt);
    odom = model.update(i * dt, truth, Twist{});
  }
  const double position_error = std::hypot(
    odom.pose.Pos().X() - truth.Pos().X(),
    odom.pose.Pos().Y() - truth.Pos().Y());
  EXPECT_GT(position_error, 1e-3);
  EXPECT_GT(std::abs(model.yawError()), 1e-4);

  // The yaw error is a rotation about the world z axis, so tilt is unchanged.
  EXPECT_NEAR(odom.pose.Rot().Roll(), truth.Rot().Roll(), 1e-9);
  EXPECT_NEAR(odom.pose.Rot().Pitch(), truth.Rot().Pitch(), 1e-9);
  const double yaw_error = odom.pose.Rot().Yaw() - truth.Rot().Yaw();
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
  const Pose3d truth = circlePose(0.0);
  const Odometry odom = model.update(0.0, truth, Twist{});
  EXPECT_DOUBLE_EQ(odom.pose.Pos().X(), truth.Pos().X());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().Y(), truth.Pos().Y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance[0], kExactVariance);
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
    Pose3d truth;
    for (int i = 0; i <= steps; ++i) {
      const double t = i / rate_hz;
      truth = circlePose(t);
      odom = model.update(t, truth, Twist{});
    }
    sum_x2 += std::pow(odom.pose.Pos().X() - truth.Pos().X(), 2);
    sum_z2 += std::pow(odom.pose.Pos().Z() - truth.Pos().Z(), 2);
    sum_yaw2 += std::pow(model.yawError(), 2);
  }
  // The covariance is deterministic for a given trajectory up to the noise
  // in the displacement, so the last run's value is representative.
  const double var_x = odom.pose_covariance[0];
  const double var_z = odom.pose_covariance[2 * 6 + 2];
  const double var_yaw = odom.pose_covariance[5 * 6 + 5];
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
Odometry flyCircle(DriftOdometryModel & model, Pose3d & truth)
{
  Odometry odom;
  for (int i = 0; i <= 6000; ++i) {
    truth = circlePose(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  return odom;
}

double yawError(const Odometry & odom, const Pose3d & truth)
{
  return wrap(odom.pose.Rot().Yaw() - truth.Rot().Yaw());
}

double horizontalError(const Odometry & odom, const Pose3d & truth)
{
  return std::hypot(
    odom.pose.Pos().X() - truth.Pos().X(),
    odom.pose.Pos().Y() - truth.Pos().Y());
}

TEST(DriftOdometryModel, HorizontalDriftOffKeepsXYExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  DriftOdometryModel model(params, 11);
  Pose3d truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.Pos().X(), truth.Pos().X());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().Y(), truth.Pos().Y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance[0 * 6 + 0], kExactVariance);
  EXPECT_DOUBLE_EQ(odom.pose_covariance[1 * 6 + 1], kExactVariance);
  // Yaw still drifts, but no longer leaks into the position covariance.
  EXPECT_DOUBLE_EQ(odom.pose_covariance[0 * 6 + 5], 0.0);
  EXPECT_GT(std::abs(odom.pose.Pos().Z() - truth.Pos().Z()), 1e-3);
  EXPECT_GT(std::abs(yawError(odom, truth)), 1e-4);
}

TEST(DriftOdometryModel, VerticalDriftOffKeepsZExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_vertical = false;
  DriftOdometryModel model(params, 12);
  Pose3d truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.Pos().Z(), truth.Pos().Z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance[2 * 6 + 2], kExactVariance);
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(std::abs(yawError(odom, truth)), 1e-4);
}

TEST(DriftOdometryModel, YawDriftOffKeepsYawExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_yaw = false;
  DriftOdometryModel model(params, 13);
  Pose3d truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
  EXPECT_NEAR(yawError(odom, truth), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(odom.pose_covariance[5 * 6 + 5], kExactVariance);
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(std::abs(odom.pose.Pos().Z() - truth.Pos().Z()), 1e-3);
}

TEST(DriftOdometryModel, AllDriftOffMatchesGroundTruth)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 14);
  Pose3d truth;
  const Odometry odom = flyCircle(model, truth);

  EXPECT_DOUBLE_EQ(odom.pose.Pos().X(), truth.Pos().X());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().Y(), truth.Pos().Y());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().Z(), truth.Pos().Z());
  EXPECT_NEAR(yawError(odom, truth), 0.0, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(odom.pose_covariance[i * 6 + i], kExactVariance);
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
Pose3d hoverPose()
{
  return Pose3d(1.0, -2.0, 1.5, 0.2, -0.15, 0.8);
}

// While hovering, all drift comes from the biases, so integrating the
// reported velocities must reproduce the pose drift.
TEST(DriftOdometryModel, IntegratedVelocityMatchesPoseDrift)
{
  DriftOdometryParams params = biasOnly();
  params.drift_yaw = false;  // keep the heading exact so the integral is simple
  DriftOdometryModel model(params, 21);
  const Pose3d truth = hoverPose();
  const double dt = 0.01;
  Vector3d integrated;
  Odometry odom;
  for (int i = 0; i <= 6000; ++i) {
    odom = model.update(i * dt, truth, Twist{});
    if (i > 0) {
      integrated += truth.Rot().RotateVector(odom.twist.linear) * dt;
    }
  }
  const double drift_x = odom.pose.Pos().X() - truth.Pos().X();
  const double drift_z = odom.pose.Pos().Z() - truth.Pos().Z();
  EXPECT_GT(std::abs(drift_x), 0.05);
  EXPECT_NEAR(integrated.X(), drift_x, 1e-9);
  EXPECT_NEAR(integrated.Y(), odom.pose.Pos().Y() - truth.Pos().Y(), 1e-9);
  EXPECT_NEAR(integrated.Z(), drift_z, 1e-9);
}

TEST(DriftOdometryModel, IntegratedYawRateMatchesYawDrift)
{
  DriftOdometryParams params = biasOnly();
  params.linear_bias_noise = 0.0;
  DriftOdometryModel model(params, 22);
  const Pose3d truth = hoverPose();
  const double dt = 0.01;
  double integrated = 0.0;
  for (int i = 0; i <= 6000; ++i) {
    const Odometry odom = model.update(i * dt, truth, Twist{});
    if (i > 0) {
      // Yaw rate is the world z component of the angular velocity.
      integrated += truth.Rot().RotateVector(odom.twist.angular).Z() * dt;
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
  twist.linear = {1.0, -0.5, 0.2};
  twist.angular = {0.1, 0.0, 0.3};
  const Odometry odom = model.update(0.0, hoverPose(), twist);
  EXPECT_NEAR(odom.twist.linear.X(), 1.05, 1e-12);
  EXPECT_NEAR(odom.twist.linear.Y(), -0.525, 1e-12);
  EXPECT_NEAR(odom.twist.linear.Z(), 0.21, 1e-12);
  EXPECT_NEAR(odom.twist.angular.Z(), 0.3, 1e-12);
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
  twist.linear = {1.0, -0.5, 0.2};
  twist.angular = {0.1, 0.0, 0.3};
  Odometry odom;
  for (int i = 0; i <= 1000; ++i) {
    odom = model.update(0.01 * i, hoverPose(), twist);
  }
  EXPECT_NEAR(odom.twist.linear.X(), 1.0, 1e-12);
  EXPECT_NEAR(odom.twist.linear.Y(), -0.5, 1e-12);
  EXPECT_NEAR(odom.twist.linear.Z(), 0.2, 1e-12);
  EXPECT_NEAR(odom.twist.angular.Z(), 0.3, 1e-12);
}

// The reported twist covariance must match the spread of the twist error.
TEST(DriftOdometryModel, TwistCovarianceMatchesSpread)
{
  DriftOdometryParams params = allNoiseSources();
  params.twist_linear_noise = 0.02;
  params.twist_angular_noise = 0.005;
  const int runs = 400;
  const Pose3d truth = hoverPose();
  double sum_vx2 = 0.0;
  double sum_vz2 = 0.0;
  double sum_wz2 = 0.0;
  Odometry odom;
  for (int run = 0; run < runs; ++run) {
    DriftOdometryModel model(params, 3000 + run);
    for (int i = 0; i <= 1000; ++i) {
      odom = model.update(0.01 * i, truth, Twist{});
    }
    sum_vx2 += odom.twist.linear.X() * odom.twist.linear.X();
    sum_vz2 += odom.twist.linear.Z() * odom.twist.linear.Z();
    sum_wz2 += odom.twist.angular.Z() * odom.twist.angular.Z();
  }
  EXPECT_NEAR(sum_vx2 / runs / odom.twist_covariance[0 * 6 + 0], 1.0, 0.25);
  EXPECT_NEAR(sum_vz2 / runs / odom.twist_covariance[2 * 6 + 2], 1.0, 0.25);
  EXPECT_NEAR(sum_wz2 / runs / odom.twist_covariance[5 * 6 + 5], 1.0, 0.25);
}

/// Rest on the ground for 10 s, then climb to 2 m and fly the circle.
Pose3d parkThenFly(double t)
{
  if (t < 10.0) {
    Pose3d pose = circlePose(0.0);
    pose.Pos().Z() = 0.05;
    return pose;
  }
  Pose3d pose = circlePose(t - 10.0);
  pose.Pos().Z() = std::min(0.05 + (t - 10.0), 2.0);
  return pose;
}

TEST(DriftOdometryModel, DriftStartsAtTakeoff)
{
  DriftOdometryParams params = strongNoise();
  params.drift_start = DriftStart::kTakeoff;
  DriftOdometryModel model(params, 31);
  Odometry odom;
  Pose3d truth;
  for (int i = 0; i <= 1000; ++i) {  // parked
    truth = parkThenFly(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().X(), truth.Pos().X());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().Z(), truth.Pos().Z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance[0], kExactVariance);

  for (int i = 1001; i <= 7000; ++i) {  // takeoff and flight
    truth = parkThenFly(0.01 * i);
    odom = model.update(0.01 * i, truth, Twist{});
  }
  EXPECT_TRUE(model.driftStarted());
  EXPECT_GT(horizontalError(odom, truth), 1e-3);
  EXPECT_GT(odom.pose_covariance[0], kExactVariance);
}

TEST(DriftOdometryModel, DriftStartsWithSimulationByDefault)
{
  DriftOdometryModel model(strongNoise(), 32);
  Odometry odom;
  Pose3d truth;
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
  const Pose3d truth = parkThenFly(0.0);
  const Odometry odom = model.update(0.0, truth, Twist{});
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.Pos().X(), truth.Pos().X());
}

}  // namespace
