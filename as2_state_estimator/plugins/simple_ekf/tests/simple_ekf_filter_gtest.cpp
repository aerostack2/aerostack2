// Copyright 2024 Universidad Politécnica de Madrid
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
* @file simple_ekf_filter_gtest.cpp
*
* Unit tests for simple_ekf_core::Filter, driven directly, without ROS: the pre-flight
* correction, the repeated-position check and the guard against IMU stamp jumps, with their
* defaults and configured values.
*
* @authors Rodrigo Da Silva Gómez
*/

#include <gtest/gtest.h>

#include <cmath>

#include <ekf/ekf_datatype.hpp>

#include <simple_ekf_core/filter.hpp>

namespace simple_ekf_core
{
namespace
{

constexpr Nanoseconds kStart = 1000000000;
constexpr Nanoseconds kTick = 10000000;
constexpr Nanoseconds kSecond = 1000000000;

// Loose initial covariance, so that a single correction visibly moves the state
Config looseConfig()
{
  Config config;
  config.initial_position_covariance = 1.0;
  config.initial_velocity_covariance = 1.0;
  config.initial_orientation_covariance = 1.0;
  return config;
}

// Start estimation and feed one IMU reading of a vehicle at rest
void startAtRest(Filter & filter)
{
  filter.markEarthToMapSet();
  ImuSample imu;
  imu.stamp = kStart;
  imu.linear_acceleration = {0.0, 0.0, 9.81};
  filter.onImu(imu);
}

// An IMU reading of a vehicle accelerating gently, so that a prediction is visible
ImuSample imuAt(Nanoseconds stamp, double yaw_rate = 0.0)
{
  ImuSample imu;
  imu.stamp = stamp;
  imu.linear_acceleration = {0.5, 0.0, 9.81};
  imu.angular_velocity = {0.0, 0.0, yaw_rate};
  return imu;
}

// Start estimation and feed `count` readings at 100 Hz, the last one stamped kStart + count - 1
// ticks
void feedImuRun(Filter & filter, int count = 10)
{
  filter.markEarthToMapSet();
  for (int i = 0; i < count; ++i) {
    filter.onImu(imuAt(kStart + i * kTick));
  }
}

void expectSameState(const Filter & filter, const Filter & other)
{
  for (std::size_t i = 0; i < ekf::State::size; ++i) {
    EXPECT_DOUBLE_EQ(filter.state().data[i], other.state().data[i]) << "state " << i;
  }
}

// A map-frame pose measuring every component, with the variances a mocap source is given
PoseSample mapPose(Nanoseconds stamp, const Vector3 & position)
{
  PoseSample pose;
  pose.stamp = stamp;
  pose.frame = SourceFrame::MAP;
  pose.pose = Rigid(Quaternion(0.0, 0.0, 0.0, 1.0), position);
  for (int i = 0; i < 3; ++i) {
    pose.covariance[i * 7] = 1e-4;
    pose.covariance[(i + 3) * 7] = 1e-5;
  }
  return pose;
}

}  // namespace

// ---------------------------------------------------------------------------
// Pre-flight correction
// ---------------------------------------------------------------------------

// The defaults are the constants the correction had before it was configurable.
TEST(FilterPreflightTest, DefaultsHoldTheDroneAtTheMapOrigin)
{
  const Config config;
  EXPECT_EQ(config.preflight_variance, 1e-5);
  EXPECT_EQ(config.preflight_pose.getOrigin(), Vector3(0.0, 0.0, 0.0));
  EXPECT_EQ(config.preflight_pose.getRotation(), Quaternion(0.0, 0.0, 0.0, 1.0));
}

TEST(FilterPreflightTest, PullsTheStateTowardsTheConfiguredPose)
{
  Config config = looseConfig();
  Quaternion rotation;
  rotation.setRPY(0.0, 0.0, 0.5);
  config.preflight_pose = Rigid(rotation, Vector3(1.0, -2.0, 0.5));
  Filter filter(config);
  startAtRest(filter);

  for (int tick = 1; tick <= 10; ++tick) {
    EXPECT_TRUE(filter.onTick(kStart + tick * kTick));
  }

  EXPECT_NEAR(filter.state().data[ekf::State::X], 1.0, 1e-3);
  EXPECT_NEAR(filter.state().data[ekf::State::Y], -2.0, 1e-3);
  EXPECT_NEAR(filter.state().data[ekf::State::Z], 0.5, 1e-3);
  EXPECT_NEAR(filter.state().data[ekf::State::YAW], 0.5, 1e-3);
}

TEST(FilterPreflightTest, ALowerVariancePullsHarder)
{
  Config firm = looseConfig();
  firm.preflight_pose = Rigid(Quaternion(0.0, 0.0, 0.0, 1.0), Vector3(1.0, 0.0, 0.0));
  Config soft = firm;
  soft.preflight_variance = 1.0;

  Filter firm_filter(firm);
  Filter soft_filter(soft);
  startAtRest(firm_filter);
  startAtRest(soft_filter);
  firm_filter.onTick(kStart + kTick);
  soft_filter.onTick(kStart + kTick);

  // One correction of variance v on a state of variance 1 moves it 1 / (1 + v) of the way
  EXPECT_GT(firm_filter.state().data[ekf::State::X], 0.99);
  EXPECT_NEAR(soft_filter.state().data[ekf::State::X], 0.5, 0.01);
}

TEST(FilterPreflightTest, StopsForGoodOnceTheDroneHasBeenOffboard)
{
  Filter filter(looseConfig());
  startAtRest(filter);
  EXPECT_TRUE(filter.onTick(kStart + kTick));

  filter.setOffboard(true);
  EXPECT_FALSE(filter.onTick(kStart + 2 * kTick));
  filter.setOffboard(false);
  EXPECT_FALSE(filter.onTick(kStart + 3 * kTick));
}

TEST(FilterPreflightTest, HasBeenOffboardLatchesLikeTheCorrection)
{
  Filter filter(looseConfig());
  EXPECT_FALSE(filter.hasBeenOffboard());
  filter.setOffboard(true);
  EXPECT_TRUE(filter.hasBeenOffboard());
  filter.setOffboard(false);
  EXPECT_TRUE(filter.hasBeenOffboard());
}

TEST(FilterPreflightTest, ANonPositiveVarianceFallsBackToTheDefault)
{
  Config config;
  config.preflight_variance = 0.0;
  const Filter filter(config);
  EXPECT_EQ(filter.config().preflight_variance, 1e-5);
}

// ---------------------------------------------------------------------------
// Repeated positions
// ---------------------------------------------------------------------------

TEST(FilterRepeatedPositionTest, DefaultThresholdIsAMicrometre)
{
  EXPECT_EQ(SourceConfig().repeated_position_threshold, 1e-6);

  Filter filter{Config()};
  SourceConfig source;
  source.name = "mocap";
  source.reject_repeated_positions = true;
  const SourceId id = filter.addSource(source);

  EXPECT_FALSE(filter.isRepeatedPosition(id, Vector3(0.0, 0.0, 0.0), kStart));
  EXPECT_TRUE(filter.isRepeatedPosition(id, Vector3(5e-7, 0.0, 0.0), kStart));
  EXPECT_FALSE(filter.isRepeatedPosition(id, Vector3(2e-6, 0.0, 0.0), kStart));
}

TEST(FilterRepeatedPositionTest, ThresholdIsConfigurable)
{
  Filter filter{Config()};
  SourceConfig source;
  source.name = "mocap";
  source.reject_repeated_positions = true;
  source.repeated_position_threshold = 0.01;
  const SourceId id = filter.addSource(source);

  EXPECT_FALSE(filter.isRepeatedPosition(id, Vector3(0.0, 0.0, 0.0), kStart));
  // Within a centimetre of the last position accepted, which a rejection does not replace
  EXPECT_TRUE(filter.isRepeatedPosition(id, Vector3(0.005, 0.0, 0.0), kStart));
  EXPECT_TRUE(filter.isRepeatedPosition(id, Vector3(0.0, 0.009, 0.0), kStart));
  EXPECT_FALSE(filter.isRepeatedPosition(id, Vector3(0.02, 0.0, 0.0), kStart));
}

TEST(FilterRepeatedPositionTest, EachSourceHasItsOwnThreshold)
{
  Filter filter{Config()};
  SourceConfig fine;
  fine.name = "fine";
  fine.reject_repeated_positions = true;
  SourceConfig coarse = fine;
  coarse.name = "coarse";
  coarse.repeated_position_threshold = 0.1;
  const SourceId fine_id = filter.addSource(fine);
  const SourceId coarse_id = filter.addSource(coarse);

  EXPECT_FALSE(filter.isRepeatedPosition(fine_id, Vector3(0.0, 0.0, 0.0), kStart));
  EXPECT_FALSE(filter.isRepeatedPosition(coarse_id, Vector3(0.0, 0.0, 0.0), kStart));
  EXPECT_FALSE(filter.isRepeatedPosition(fine_id, Vector3(0.05, 0.0, 0.0), kStart));
  EXPECT_TRUE(filter.isRepeatedPosition(coarse_id, Vector3(0.05, 0.0, 0.0), kStart));
}

// ---------------------------------------------------------------------------
// Whether a pose was fused, and the pose it was fused as
// ---------------------------------------------------------------------------

TEST(FilterPoseFusionTest, AFusedPoseIsReportedAndKeptInTheMapFrame)
{
  Filter filter(looseConfig());
  startAtRest(filter);
  SourceConfig source;
  source.name = "mocap";
  const SourceId id = filter.addSource(source);

  EXPECT_TRUE(filter.onPose(id, mapPose(kStart + kTick, Vector3(1.0, -0.5, 2.0)), kStart + kTick));
  const PoseSample & fused = filter.lastFusedPoseInMap();
  EXPECT_EQ(fused.frame, SourceFrame::MAP);
  EXPECT_EQ(fused.stamp, kStart + kTick);
  EXPECT_NEAR(fused.pose.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(fused.pose.getOrigin().y(), -0.5, 1e-9);
  EXPECT_NEAR(fused.pose.getOrigin().z(), 2.0, 1e-9);
}

TEST(FilterPoseFusionTest, AGatedPoseIsNotReportedAsFused)
{
  Filter filter(looseConfig());
  startAtRest(filter);
  SourceConfig source;
  source.name = "mocap";
  source.innovation_gate = 3.0;
  const SourceId id = filter.addSource(source);

  ASSERT_TRUE(filter.onPose(id, mapPose(kStart + kTick, Vector3(0.1, 0.0, 0.0)), kStart + kTick));
  // A hundred metres away from a state that just converged is far outside three sigma
  EXPECT_FALSE(
    filter.onPose(id, mapPose(kStart + 2 * kTick, Vector3(100.0, 0.0, 0.0)), kStart + 2 * kTick));
  EXPECT_NEAR(filter.lastFusedPoseInMap().pose.getOrigin().x(), 0.1, 1e-9);
}

TEST(FilterPoseFusionTest, APoseTooOldToReplayIsNotReportedAsFused)
{
  Config config = looseConfig();
  config.max_update_latency_ms = 100.0;
  Filter filter(config);
  startAtRest(filter);
  SourceConfig source;
  source.name = "mocap";
  const SourceId id = filter.addSource(source);

  // Stamped at the start, received a second later
  EXPECT_FALSE(filter.onPose(id, mapPose(kStart, Vector3(1.0, 0.0, 0.0)), kStart + 100 * kTick));
}

// ---------------------------------------------------------------------------
// IMU stamp jumps
// ---------------------------------------------------------------------------

// The default is the threshold the guard was introduced with.
TEST(FilterImuStampTest, TheDefaultThresholdIsTwoHundredMilliseconds)
{
  EXPECT_EQ(Config().max_imu_dt_ms, 200.0);
}

// The clock is corrected forward mid-flight: the sample that carries the step is not predicted
// with, and the next one is measured against it, so only that one sample is lost.
TEST(FilterImuStampTest, AForwardJumpIsSkippedAndTheNextSampleContinuesFromIt)
{
  Filter jumped(looseConfig());
  Filter steady(looseConfig());
  feedImuRun(jumped);
  feedImuRun(steady);

  const ekf::State before = jumped.state();
  const Nanoseconds jump = kStart + 9 * kTick + 5 * kSecond;
  jumped.onImu(imuAt(jump));
  for (std::size_t i = 0; i < ekf::State::size; ++i) {
    EXPECT_DOUBLE_EQ(jumped.state().data[i], before.data[i])
      << "the jumped sample must not predict, state " << i;
  }

  jumped.onImu(imuAt(jump + kTick));
  steady.onImu(imuAt(kStart + 10 * kTick));
  expectSameState(jumped, steady);
}

// The same backward, which is what an NTP correction looks like from the other side.
TEST(FilterImuStampTest, ABackwardJumpIsSkippedAndTheNextSampleContinuesFromIt)
{
  Filter jumped(looseConfig());
  Filter steady(looseConfig());
  feedImuRun(jumped);
  feedImuRun(steady);

  const ekf::State before = jumped.state();
  const Nanoseconds jump = kStart + 9 * kTick - 2 * kSecond;
  jumped.onImu(imuAt(jump));
  for (std::size_t i = 0; i < ekf::State::size; ++i) {
    EXPECT_DOUBLE_EQ(jumped.state().data[i], before.data[i])
      << "the jumped sample must not predict, state " << i;
  }

  jumped.onImu(imuAt(jump + kTick));
  steady.onImu(imuAt(kStart + 10 * kTick));
  expectSameState(jumped, steady);
}

// A gap the IMU itself can have, rather than a clock step, is still predicted over.
TEST(FilterImuStampTest, AGapUnderTheThresholdIsPredictedWith)
{
  Filter filter(looseConfig());
  feedImuRun(filter);
  const double x_before = filter.state().data[ekf::State::X];

  filter.onImu(imuAt(kStart + 9 * kTick + fromSeconds(0.15)));

  EXPECT_NE(filter.state().data[ekf::State::X], x_before);
}

TEST(FilterImuStampTest, TheThresholdIsConfigurable)
{
  Config config = looseConfig();
  config.max_imu_dt_ms = 50.0;
  Filter filter(config);
  feedImuRun(filter);
  const double x_before = filter.state().data[ekf::State::X];

  filter.onImu(imuAt(kStart + 9 * kTick + fromSeconds(0.1)));

  EXPECT_EQ(filter.state().data[ekf::State::X], x_before)
    << "a 100 ms gap is a jump when the threshold is 50 ms";
}

// Only the stamp of a jumped sample is wrong: the reading itself is the newest the filter has,
// and it is what the published angular velocity reports.
TEST(FilterImuStampTest, TheSkippedReadingStillFeedsThePublishedAngularVelocity)
{
  Filter filter(looseConfig());
  feedImuRun(filter);

  filter.onImu(imuAt(kStart + 9 * kTick + 5 * kSecond, 0.3));

  EXPECT_NEAR(filter.outputs().twist_in_base.angular.z(), 0.3, 1e-3);
}

// The history the buffer keeps is on the old clock, so a late pose arriving after a jump would
// rewind into it. Past the jump, the filter behaves as one that never saw the old clock.
TEST(FilterImuStampTest, AfterAJumpALatePoseIsReplayedAgainstTheNewHistoryOnly)
{
  Filter jumped(looseConfig());
  Filter steady(looseConfig());
  feedImuRun(jumped);
  feedImuRun(steady);
  SourceConfig source;
  source.name = "mocap";
  const SourceId jumped_source = jumped.addSource(source);
  const SourceId steady_source = steady.addSource(source);

  const Nanoseconds jump = kStart + 9 * kTick - 2 * kSecond;
  jumped.onImu(imuAt(jump));
  jumped.onImu(imuAt(jump + kTick));
  steady.onImu(imuAt(kStart + 10 * kTick));

  // Stamped before the last reading of each, so both rewind one prediction
  jumped.onPose(jumped_source, mapPose(jump + kTick / 2, Vector3(1.0, 2.0, 3.0)), jump + kTick);
  steady.onPose(
    steady_source, mapPose(kStart + 10 * kTick - kTick / 2, Vector3(1.0, 2.0, 3.0)),
    kStart + 10 * kTick);

  expectSameState(jumped, steady);
}

}  // namespace simple_ekf_core
