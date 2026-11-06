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
* @file correction_gtest.cpp
*
* Unit tests for the transform helpers the external_localization plugin uses to move map to
* odom toward a localization correction.
*/

#include <gtest/gtest.h>

#include <cmath>

#include "external_localization/external_localization.hpp"

namespace
{

tf2::Transform pose(double x, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, 0.0, 0.0));
}

double yawOf(const tf2::Transform & t)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3(t.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

TEST(CorrectionTest, Interpolate_Midpoint)
{
  const auto mid = external_localization::interpolate(pose(0.0, 0.0), pose(2.0, 1.0), 0.5);
  EXPECT_NEAR(mid.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(yawOf(mid), 0.5, 1e-9);
}

TEST(CorrectionTest, Interpolate_TakesShortWayAcrossPi)
{
  // 3 rad to -3 rad is 0.28 rad through pi, not 6 rad through zero
  const auto mid = external_localization::interpolate(pose(0.0, 3.0), pose(0.0, -3.0), 0.5);
  EXPECT_NEAR(std::abs(yawOf(mid)), M_PI, 1e-9);
}

TEST(CorrectionTest, StepToward_WithinLimits_ReachesTarget)
{
  const auto out = external_localization::stepToward(pose(0.0, 0.0), pose(0.1, 0.1), 1.0, 1.0);
  EXPECT_NEAR(out.getOrigin().x(), 0.1, 1e-9);
  EXPECT_NEAR(yawOf(out), 0.1, 1e-9);
}

TEST(CorrectionTest, StepToward_LimitsTranslation)
{
  const auto out = external_localization::stepToward(pose(0.0, 0.0), pose(10.0, 0.0), 0.5, 1.0);
  EXPECT_NEAR(out.getOrigin().x(), 0.5, 1e-9);
}

TEST(CorrectionTest, StepToward_LimitsRotation)
{
  const auto out = external_localization::stepToward(pose(0.0, 0.0), pose(0.0, 1.0), 1.0, 0.1);
  EXPECT_NEAR(yawOf(out), 0.1, 1e-9);
}

TEST(CorrectionTest, StepToward_TighterLimitWins)
{
  // Translation allows 50% of the way, rotation only 10%, so both move 10%
  const auto out = external_localization::stepToward(pose(0.0, 0.0), pose(2.0, 1.0), 1.0, 0.1);
  EXPECT_NEAR(out.getOrigin().x(), 0.2, 1e-9);
  EXPECT_NEAR(yawOf(out), 0.1, 1e-9);
}
