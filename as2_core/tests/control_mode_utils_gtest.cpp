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

/*!*******************************************************************************************
 *  \file       control_mode_utils_gtest.cpp
 *  \brief      Test file for the control mode matching and command frame utilities
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_core/utils/control_mode_utils.hpp"

#include <vector>
#include "gtest/gtest.h"

namespace
{

using as2_msgs::msg::ControlMode;

ControlMode makeMode(uint8_t control_mode, uint8_t yaw_mode)
{
  ControlMode mode;
  mode.control_mode = control_mode;
  mode.yaw_mode = yaw_mode;
  return mode;
}

TEST(ControlModeCodec, EncodeLeavesReservedBitsToZero) {
  const uint8_t encoded =
    as2::control_mode::convertAS2ControlModeToUint8t(
    makeMode(ControlMode::SPEED, ControlMode::YAW_SPEED));

  EXPECT_EQ(encoded, 0b01000100);
  EXPECT_EQ(encoded & MATCH_RESERVED_BITS, 0);
}

TEST(ControlModeCodec, DecodeIgnoresReservedBits) {
  // The reserved bits used to encode the reference frame of the mode
  const ControlMode with_bits = as2::control_mode::convertUint8tToAS2ControlMode(0b01000101);
  const ControlMode without_bits = as2::control_mode::convertUint8tToAS2ControlMode(0b01000100);

  EXPECT_EQ(with_bits.control_mode, without_bits.control_mode);
  EXPECT_EQ(with_bits.yaw_mode, without_bits.yaw_mode);
  EXPECT_EQ(with_bits.control_mode, ControlMode::SPEED);
  EXPECT_EQ(with_bits.yaw_mode, ControlMode::YAW_SPEED);
}

TEST(FindBestMatchWithMask, ExactMatchWinsOverListOrder) {
  const std::vector<uint8_t> modes = {0b01000100, 0b01000000};
  uint8_t match = 0;

  ASSERT_TRUE(
    as2::control_mode::findBestMatchWithMask(0b01000000, modes, MATCH_CONTROL_MODE, match));
  EXPECT_EQ(match, 0b01000000);
}

TEST(FindBestMatchWithMask, ListOrderExpressesPreference) {
  const std::vector<uint8_t> modes = {0b01000100, 0b01000000};
  uint8_t match = 0;

  // Neither candidate is an exact match under the full byte, so the first wins
  ASSERT_TRUE(
    as2::control_mode::findBestMatchWithMask(0b01001000, modes, MATCH_CONTROL_MODE, match));
  EXPECT_EQ(match, 0b01000100);
}

TEST(FindBestMatchWithMask, LeavesOutputUntouchedWhenNothingMatches) {
  const std::vector<uint8_t> modes = {0b01000100};
  uint8_t match = 0xFF;

  EXPECT_FALSE(
    as2::control_mode::findBestMatchWithMask(0b01100000, modes, MATCH_CONTROL_MODE, match));
  EXPECT_EQ(match, 0xFF);
}

TEST(ResolveControlMode, MatchesControlModeAndYawMode) {
  const std::vector<uint8_t> available = {0b01000000, 0b01000100};
  ControlMode resolved;

  ASSERT_TRUE(
    as2::control_mode::resolveControlMode(
      makeMode(ControlMode::SPEED, ControlMode::YAW_SPEED), available, resolved));
  EXPECT_EQ(resolved.control_mode, ControlMode::SPEED);
  EXPECT_EQ(resolved.yaw_mode, ControlMode::YAW_SPEED);
}

TEST(ResolveControlMode, HoverIgnoresYawMode) {
  const std::vector<uint8_t> available = {0b00010000};
  ControlMode resolved;

  ASSERT_TRUE(
    as2::control_mode::resolveControlMode(
      makeMode(ControlMode::HOVER, ControlMode::YAW_SPEED), available, resolved));
  EXPECT_EQ(resolved.control_mode, ControlMode::HOVER);
}

TEST(ResolveControlMode, FailsWhenTheModeIsNotAvailable) {
  const std::vector<uint8_t> available = {0b01000100};
  ControlMode resolved = makeMode(ControlMode::HOVER, ControlMode::NONE);

  EXPECT_FALSE(
    as2::control_mode::resolveControlMode(
      makeMode(ControlMode::TRAJECTORY, ControlMode::YAW_ANGLE), available, resolved));
  EXPECT_EQ(resolved.control_mode, ControlMode::HOVER);
}

TEST(ResolveControlMode, YawModeMustMatch) {
  const std::vector<uint8_t> available = {0b01000100};
  ControlMode resolved;

  EXPECT_FALSE(
    as2::control_mode::resolveControlMode(
      makeMode(ControlMode::SPEED, ControlMode::YAW_ANGLE), available, resolved));
}

TEST(GetCommandFrameUsage, PoseAndTwistModes) {
  for (const uint8_t control_mode :
    {ControlMode::POSITION, ControlMode::SPEED_IN_A_PLANE, ControlMode::TRAJECTORY})
  {
    const auto usage =
      as2::control_mode::getCommandFrameUsage(makeMode(control_mode, ControlMode::YAW_ANGLE));
    EXPECT_TRUE(usage.pose) << "control mode " << static_cast<int>(control_mode);
    EXPECT_TRUE(usage.twist) << "control mode " << static_cast<int>(control_mode);
  }
}

TEST(GetCommandFrameUsage, AttitudeOnlyUsesThePose) {
  const auto usage = as2::control_mode::getCommandFrameUsage(
    makeMode(ControlMode::ATTITUDE, ControlMode::YAW_ANGLE));

  EXPECT_TRUE(usage.pose);
  EXPECT_FALSE(usage.twist);
}

TEST(GetCommandFrameUsage, SpeedUsesThePoseOnlyForTheYawAngle) {
  const auto with_angle = as2::control_mode::getCommandFrameUsage(
    makeMode(ControlMode::SPEED, ControlMode::YAW_ANGLE));
  EXPECT_TRUE(with_angle.pose);
  EXPECT_TRUE(with_angle.twist);

  const auto with_rate = as2::control_mode::getCommandFrameUsage(
    makeMode(ControlMode::SPEED, ControlMode::YAW_SPEED));
  EXPECT_FALSE(with_rate.pose);
  EXPECT_TRUE(with_rate.twist);
}

TEST(GetCommandFrameUsage, ModesWithoutConvertibleReferences) {
  for (const uint8_t control_mode :
    {ControlMode::UNSET, ControlMode::HOVER, ControlMode::BODY_RATES})
  {
    const auto usage =
      as2::control_mode::getCommandFrameUsage(makeMode(control_mode, ControlMode::YAW_SPEED));
    EXPECT_FALSE(usage.pose) << "control mode " << static_cast<int>(control_mode);
    EXPECT_FALSE(usage.twist) << "control mode " << static_cast<int>(control_mode);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
