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
* correction and the repeated-position check, with their defaults and configured values.
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

}  // namespace simple_ekf_core
