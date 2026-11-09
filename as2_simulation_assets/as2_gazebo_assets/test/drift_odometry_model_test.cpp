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
#include <random>

#include "model.hpp"

// Each test feeds the model a few hand-picked samples. The first update
// starts from ground truth, and every later one is a single drift step, so
// the expected values follow from model.hpp by hand.

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

// Step length. With unit noise, sigma^2 dt = 0.1 is well above kExactVariance.
constexpr double kDt = 0.1;

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

double yawError(const Odometry & odom, const Pose & truth)
{
  return wrap(yaw(odom.pose.orientation) - yaw(truth.orientation));
}

// Level and facing +x, so the heading, body and world frames coincide.
const Pose kLevel = makePose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

// Tilted and turned, so the body, heading and world frames all differ.
const Pose kTilted = makePose(1.0, -2.0, 1.5, 0.2, -0.15, 0.8);

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

// Unit noise on every source, so any axis that is meant to drift does.
DriftOdometryParams strongNoise()
{
  DriftOdometryParams params;
  params.linear_velocity_noise = 1.0;
  params.linear_bias_noise = 1.0;
  params.yaw_rate_noise = 1.0;
  params.yaw_rate_bias_noise = 1.0;
  params.scale_error = 0.05;
  return params;
}

TEST(DriftOdometryModel, MatchesGroundTruthWithoutNoise)
{
  DriftOdometryModel model(noiseless(), 1);
  model.update(0.0, kTilted, Twist{});
  const Pose truth = makePose(3.0, 1.0, 2.0, 0.1, -0.05, 1.2);
  const Odometry odom = model.update(kDt, truth, Twist{});
  EXPECT_NEAR(odom.pose.position.x(), truth.position.x(), 1e-12);
  EXPECT_NEAR(odom.pose.position.y(), truth.position.y(), 1e-12);
  EXPECT_NEAR(odom.pose.position.z(), truth.position.z(), 1e-12);
  EXPECT_TRUE(odom.pose.orientation.isApprox(truth.orientation, 1e-12));
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
}

// Facing +y, so the step goes through the heading frame and back.
TEST(DriftOdometryModel, ScaleErrorStretchesStep)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 1);
  model.update(0.0, makePose(0.0, 0.0, 1.0, 0.0, 0.0, M_PI / 2.0), Twist{});
  const Odometry odom = model.update(kDt, makePose(0.0, 10.0, 1.0, 0.0, 0.0, M_PI / 2.0), Twist{});
  EXPECT_NEAR(odom.pose.position.x(), 0.0, 1e-12);
  EXPECT_NEAR(odom.pose.position.y(), 10.5, 1e-12);
  EXPECT_NEAR(odom.pose.position.z(), 1.0, 1e-12);
}

// The yaw error is a rotation about the world z axis, so tilt is unchanged.
TEST(DriftOdometryModel, YawDriftKeepsRollAndPitch)
{
  DriftOdometryParams params = noiseless();
  params.yaw_rate_noise = 1.0;
  DriftOdometryModel model(params, 42);
  model.update(0.0, kTilted, Twist{});
  const Odometry odom = model.update(kDt, kTilted, Twist{});
  ASSERT_NE(model.yawError(), 0.0);
  EXPECT_NEAR(roll(odom.pose.orientation), roll(kTilted.orientation), 1e-12);
  EXPECT_NEAR(pitch(odom.pose.orientation), pitch(kTilted.orientation), 1e-12);
  EXPECT_NEAR(yawError(odom, kTilted), wrap(model.yawError()), 1e-12);
}

TEST(DriftOdometryModel, RestartsFromGroundTruthWhenTimeGoesBack)
{
  DriftOdometryModel model(strongNoise(), 7);
  model.update(1.0, kLevel, Twist{});
  const Odometry drifted = model.update(1.0 + kDt, kLevel, Twist{});
  ASSERT_NE(drifted.pose.position.x(), kLevel.position.x());

  const Odometry odom = model.update(0.0, kTilted, Twist{});
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), kTilted.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), kTilted.position.y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
}

// The model draws N(0, 1) samples in a fixed order: per step, 3 for the
// displacement, 3 for the velocity bias, 1 for yaw and 1 for the yaw rate
// bias, then 6 for the twist on every update. Replaying the same generator
// shows each step adds sigma sqrt(dt) n, so its variance is sigma^2 dt.
TEST(DriftOdometryModel, StepNoiseIsSigmaSqrtDt)
{
  DriftOdometryParams params = noiseless();
  params.linear_velocity_noise = 0.3;
  params.yaw_rate_noise = 0.2;
  DriftOdometryModel model(params, 5);
  model.update(0.0, kLevel, Twist{});
  const Odometry odom = model.update(kDt, kLevel, Twist{});

  std::mt19937_64 rng(5);
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int i = 0; i < 6; ++i) {
    normal(rng);  // twist noise on the first update
  }
  Eigen::Vector3d n;
  n.x() = normal(rng);
  n.y() = normal(rng);
  n.z() = normal(rng);
  for (int i = 0; i < 3; ++i) {
    normal(rng);  // velocity bias
  }
  const double n_yaw = normal(rng);

  const Eigen::Vector3d expected = kLevel.position + 0.3 * std::sqrt(kDt) * n;
  EXPECT_TRUE(odom.pose.position.isApprox(expected, 1e-12));
  EXPECT_NEAR(model.yawError(), 0.2 * std::sqrt(kDt) * n_yaw, 1e-12);
}

// From zero covariance, one step adds Q = sigma^2 dt on each noisy axis.
TEST(DriftOdometryModel, OneStepAddsSigmaSquaredDt)
{
  DriftOdometryParams params = noiseless();
  params.linear_velocity_noise = 1.0;
  params.yaw_rate_noise = 1.0;
  for (const double dt : {0.01, 0.04}) {
    DriftOdometryModel model(params, 1);
    model.update(0.0, kLevel, Twist{});
    const Odometry odom = model.update(dt, kLevel, Twist{});
    EXPECT_NEAR(odom.pose_covariance(0, 0), dt, 1e-12);
    EXPECT_NEAR(odom.pose_covariance(1, 1), dt, 1e-12);
    EXPECT_NEAR(odom.pose_covariance(2, 2), dt, 1e-12);
    EXPECT_NEAR(odom.pose_covariance(5, 5), dt, 1e-12);
    EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 1), 0.0);
  }
}

// A bias drawn in one step moves the pose in the next one, so its variance
// sigma^2 dt reaches the pose as dt^2 sigma^2 dt.
TEST(DriftOdometryModel, BiasVarianceReachesPoseNextStep)
{
  DriftOdometryParams params = noiseless();
  params.linear_bias_noise = 1.0;
  params.yaw_rate_bias_noise = 1.0;
  DriftOdometryModel model(params, 1);
  model.update(0.0, kLevel, Twist{});
  const Odometry first = model.update(kDt, kLevel, Twist{});
  EXPECT_DOUBLE_EQ(first.pose_covariance(0, 0), kExactVariance);
  EXPECT_DOUBLE_EQ(first.pose_covariance(5, 5), kExactVariance);

  const Odometry odom = model.update(2.0 * kDt, kLevel, Twist{});
  const double expected = kDt * kDt * kDt;
  EXPECT_NEAR(odom.pose_covariance(0, 0), expected, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(1, 1), expected, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(2, 2), expected, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(5, 5), expected, 1e-12);
}

// A yaw error e turns a 1 m step along x into (cos e, sin e), so the yaw
// variance from the first step moves sideways along (-sin e, cos e).
TEST(DriftOdometryModel, YawVarianceTurnsIntoSidewaysVariance)
{
  DriftOdometryParams params = noiseless();
  params.yaw_rate_noise = 1.0;
  DriftOdometryModel model(params, 3);
  model.update(0.0, kLevel, Twist{});
  model.update(kDt, kLevel, Twist{});
  const double e = model.yawError();
  const Odometry odom = model.update(2.0 * kDt, makePose(1.0, 0.0, 1.0, 0.0, 0.0, 0.0), Twist{});

  const double s = std::sin(e);
  const double c = std::cos(e);
  EXPECT_NEAR(odom.pose_covariance(1, 1), c * c * kDt, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(0, 1), -s * c * kDt, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(1, 5), c * kDt, 1e-12);
  EXPECT_NEAR(odom.pose_covariance(5, 5), 2.0 * kDt, 1e-12);
}

TEST(DriftOdometryModel, HorizontalDriftOffKeepsXYExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  DriftOdometryModel model(params, 11);
  model.update(0.0, kLevel, Twist{});
  model.update(kDt, makePose(1.0, 0.0, 1.0, 0.0, 0.0, 0.0), Twist{});
  const Pose truth = makePose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  const Odometry odom = model.update(2.0 * kDt, truth, Twist{});

  EXPECT_DOUBLE_EQ(odom.pose.position.x(), truth.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), truth.position.y());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);
  EXPECT_DOUBLE_EQ(odom.pose_covariance(1, 1), kExactVariance);
  // Yaw still drifts, but no longer leaks into the position covariance.
  EXPECT_DOUBLE_EQ(odom.pose_covariance(1, 5), 0.0);
  EXPECT_NE(odom.pose.position.z(), truth.position.z());
  EXPECT_NE(model.yawError(), 0.0);
}

TEST(DriftOdometryModel, VerticalDriftOffKeepsZExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_vertical = false;
  DriftOdometryModel model(params, 12);
  model.update(0.0, kLevel, Twist{});
  const Odometry odom = model.update(kDt, kLevel, Twist{});

  EXPECT_DOUBLE_EQ(odom.pose.position.z(), kLevel.position.z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(2, 2), kExactVariance);
  EXPECT_NE(odom.pose.position.x(), kLevel.position.x());
  EXPECT_NE(model.yawError(), 0.0);
}

TEST(DriftOdometryModel, YawDriftOffKeepsYawExact)
{
  DriftOdometryParams params = strongNoise();
  params.drift_yaw = false;
  DriftOdometryModel model(params, 13);
  model.update(0.0, kTilted, Twist{});
  const Odometry odom = model.update(kDt, kTilted, Twist{});

  EXPECT_DOUBLE_EQ(model.yawError(), 0.0);
  EXPECT_NEAR(yawError(odom, kTilted), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(odom.pose_covariance(5, 5), kExactVariance);
  EXPECT_NE(odom.pose.position.x(), kTilted.position.x());
  EXPECT_NE(odom.pose.position.z(), kTilted.position.z());
}

TEST(DriftOdometryModel, AllDriftOffMatchesGroundTruth)
{
  DriftOdometryParams params = strongNoise();
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 14);
  model.update(0.0, kTilted, Twist{});
  const Odometry odom = model.update(kDt, kTilted, Twist{});

  EXPECT_DOUBLE_EQ(odom.pose.position.x(), kTilted.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.y(), kTilted.position.y());
  EXPECT_DOUBLE_EQ(odom.pose.position.z(), kTilted.position.z());
  EXPECT_NEAR(yawError(odom, kTilted), 0.0, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(odom.pose_covariance(i, i), kExactVariance);
  }
}

// The reported velocity carries the bias used in the last step, so it
// matches how far the pose moved in that step.
TEST(DriftOdometryModel, VelocityMatchesPoseStep)
{
  DriftOdometryParams params = noiseless();
  params.linear_bias_noise = 1.0;
  DriftOdometryModel model(params, 21);
  model.update(0.0, kTilted, Twist{});
  const Odometry first = model.update(kDt, kTilted, Twist{});  // draws the bias
  const Odometry second = model.update(2.0 * kDt, kTilted, Twist{});  // moves with it

  const Eigen::Vector3d moved = (second.pose.position - first.pose.position) / kDt;
  const Eigen::Vector3d velocity = kTilted.orientation * second.twist.linear;  // world frame
  ASSERT_GT(moved.norm(), 0.0);
  EXPECT_TRUE(velocity.isApprox(moved, 1e-9));
}

TEST(DriftOdometryModel, YawRateMatchesYawStep)
{
  DriftOdometryParams params = noiseless();
  params.yaw_rate_bias_noise = 1.0;
  DriftOdometryModel model(params, 22);
  model.update(0.0, kTilted, Twist{});
  model.update(kDt, kTilted, Twist{});  // draws the bias
  const double first = model.yawError();
  const Odometry odom = model.update(2.0 * kDt, kTilted, Twist{});  // turns with it

  // Yaw rate is the world z component of the angular velocity.
  const double yaw_rate = (kTilted.orientation * odom.twist.angular).z();
  ASSERT_NE(model.yawError(), first);
  EXPECT_NEAR(yaw_rate, (model.yawError() - first) / kDt, 1e-9);
}

TEST(DriftOdometryModel, ScaleErrorStretchesVelocity)
{
  DriftOdometryParams params = noiseless();
  params.scale_error = 0.05;
  DriftOdometryModel model(params, 23);
  Twist twist;
  twist.linear = Eigen::Vector3d(1.0, -0.5, 0.2);
  twist.angular = Eigen::Vector3d(0.1, 0.0, 0.3);
  const Odometry odom = model.update(0.0, kTilted, twist);
  EXPECT_NEAR(odom.twist.linear.x(), 1.05, 1e-12);
  EXPECT_NEAR(odom.twist.linear.y(), -0.525, 1e-12);
  EXPECT_NEAR(odom.twist.linear.z(), 0.21, 1e-12);
  EXPECT_NEAR(odom.twist.angular.z(), 0.3, 1e-12);
}

TEST(DriftOdometryModel, VelocityFollowsTruthOnAxesWithoutDrift)
{
  DriftOdometryParams params = noiseless();
  params.linear_bias_noise = 1.0;
  params.yaw_rate_bias_noise = 1.0;
  params.scale_error = 0.05;
  params.drift_horizontal = false;
  params.drift_vertical = false;
  params.drift_yaw = false;
  DriftOdometryModel model(params, 24);
  Twist twist;
  twist.linear = Eigen::Vector3d(1.0, -0.5, 0.2);
  twist.angular = Eigen::Vector3d(0.1, 0.0, 0.3);
  model.update(0.0, kTilted, twist);
  model.update(kDt, kTilted, twist);
  const Odometry odom = model.update(2.0 * kDt, kTilted, twist);
  EXPECT_NEAR(odom.twist.linear.x(), 1.0, 1e-12);
  EXPECT_NEAR(odom.twist.linear.y(), -0.5, 1e-12);
  EXPECT_NEAR(odom.twist.linear.z(), 0.2, 1e-12);
  EXPECT_NEAR(odom.twist.angular.z(), 0.3, 1e-12);
}

// Twist covariance: bias variance (sigma_b^2 dt after one step) plus the
// per-sample white noise. Level, so the yaw rate bias is only on z.
TEST(DriftOdometryModel, TwistCovarianceIsBiasPlusWhiteNoise)
{
  DriftOdometryParams params = noiseless();
  params.linear_bias_noise = 1.0;
  params.yaw_rate_bias_noise = 1.0;
  params.twist_linear_noise = 0.5;
  params.twist_angular_noise = 0.5;
  DriftOdometryModel model(params, 25);
  model.update(0.0, kLevel, Twist{});
  const Odometry odom = model.update(kDt, kLevel, Twist{});
  EXPECT_NEAR(odom.twist_covariance(0, 0), kDt + 0.25, 1e-12);
  EXPECT_NEAR(odom.twist_covariance(1, 1), kDt + 0.25, 1e-12);
  EXPECT_NEAR(odom.twist_covariance(2, 2), kDt + 0.25, 1e-12);
  EXPECT_NEAR(odom.twist_covariance(3, 3), 0.25, 1e-12);
  EXPECT_NEAR(odom.twist_covariance(4, 4), 0.25, 1e-12);
  EXPECT_NEAR(odom.twist_covariance(5, 5), kDt + 0.25, 1e-12);
}

// Resting at z = 0.05 with takeoff_height = 0.1.
TEST(DriftOdometryModel, DriftStartsAtTakeoff)
{
  DriftOdometryParams params = strongNoise();
  params.drift_start = DriftStart::kTakeoff;
  DriftOdometryModel model(params, 31);
  model.update(0.0, makePose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0), Twist{});
  const Pose low = makePose(0.0, 0.0, 0.10, 0.0, 0.0, 0.0);  // 0.05 above rest
  Odometry odom = model.update(kDt, low, Twist{});
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), low.position.x());
  EXPECT_DOUBLE_EQ(odom.pose.position.z(), low.position.z());
  EXPECT_DOUBLE_EQ(odom.pose_covariance(0, 0), kExactVariance);

  const Pose high = makePose(0.0, 0.0, 0.20, 0.0, 0.0, 0.0);  // 0.15 above rest
  odom = model.update(2.0 * kDt, high, Twist{});
  EXPECT_TRUE(model.driftStarted());
  EXPECT_NE(odom.pose.position.x(), high.position.x());
  EXPECT_GT(odom.pose_covariance(0, 0), kExactVariance);
}

TEST(DriftOdometryModel, DriftStartsWithSimulationByDefault)
{
  DriftOdometryModel model(strongNoise(), 32);
  const Pose rest = makePose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0);
  model.update(0.0, rest, Twist{});
  EXPECT_TRUE(model.driftStarted());
  const Odometry odom = model.update(kDt, rest, Twist{});
  EXPECT_NE(odom.pose.position.x(), rest.position.x());
}

TEST(DriftOdometryModel, ResetWaitsForTakeoffAgain)
{
  DriftOdometryParams params = strongNoise();
  params.drift_start = DriftStart::kTakeoff;
  DriftOdometryModel model(params, 33);
  const Pose rest = makePose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0);
  model.update(0.0, rest, Twist{});
  model.update(kDt, makePose(0.0, 0.0, 0.20, 0.0, 0.0, 0.0), Twist{});
  ASSERT_TRUE(model.driftStarted());

  const Odometry odom = model.update(0.0, rest, Twist{});  // time went back
  EXPECT_FALSE(model.driftStarted());
  EXPECT_DOUBLE_EQ(odom.pose.position.x(), rest.position.x());
}

}  // namespace
