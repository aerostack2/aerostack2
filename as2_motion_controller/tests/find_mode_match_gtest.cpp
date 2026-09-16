// Copyright 2023 Universidad Politécnica de Madrid
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
 *  \file       find_mode_match_gtest.cpp
 *  \brief      Tests for the control mode matching of the controller handler
 *  \authors    Miguel Fernández Cortizas
 ********************************************************************************************/

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_motion_controller/controller_handler.hpp"
#include "as2_msgs/msg/control_mode.hpp"

namespace
{

using controller_handler::mode_negotiation::checkSuitabilityInputMode;
using controller_handler::mode_negotiation::findModePairs;
using controller_handler::mode_negotiation::findOutputModes;

// Modes of a plugin that ingests speed, position and trajectory and only
// commands speed, as the pid_speed_controller does.
const std::vector<uint8_t> kControllerModesIn = {
  0b00000000, 0b00010000, 0b01000000, 0b01000001, 0b01000100,
  0b01000101, 0b01100001, 0b01100101, 0b01110001, 0b01110101};

const std::vector<uint8_t> kControllerModesOut = {0b00000000, 0b01000100, 0b01000101};

// The single output mode this platform ingests.
const std::vector<uint8_t> kPlatformModesIn = {0b01000100};

constexpr uint8_t kNoPreferredMode = 0b00000000;

std::vector<std::pair<uint8_t, uint8_t>> findModePairsInScenario(const uint8_t input_mode)
{
  return findModePairs(
    input_mode, kNoPreferredMode, kControllerModesIn, kControllerModesOut, kPlatformModesIn);
}

}  // namespace

TEST(FindModeMatchTest, ResolvesRequestsIgnoringTheirFrame)
{
  // Requests reach the controller with an undefined frame: the mode of the
  // plugin, with its own frame, is the one that must be settled.
  const std::vector<std::pair<uint8_t, uint8_t>> requests_and_modes_in = {
    {0b01000011, 0b01000000},   // SPEED yaw ANGLE -> first SPEED yaw ANGLE of the plugin
    {0b01000111, 0b01000100},   // SPEED yaw SPEED
    {0b01100011, 0b01100001},   // POSITION yaw ANGLE
    {0b01100111, 0b01100101},   // POSITION yaw SPEED
    {0b01110111, 0b01110101},   // TRAJECTORY yaw SPEED
  };

  for (const auto & [request, expected_mode_in] : requests_and_modes_in) {
    const auto mode_pairs = findModePairsInScenario(request);
    ASSERT_EQ(mode_pairs.size(), 1u)
      << "request " << as2::control_mode::controlModeToString(request);
    EXPECT_EQ(mode_pairs.front().first, expected_mode_in)
      << "request " << as2::control_mode::controlModeToString(request);
    // The only output mode the platform of this scenario supports
    EXPECT_EQ(mode_pairs.front().second, 0b01000100);
  }
}

TEST(FindModeMatchTest, RejectsUnsetInputMode)
{
  // UNSET is below any output mode, so it cannot feed the platform
  EXPECT_TRUE(findModePairsInScenario(0b00000000).empty());
}

TEST(FindModeMatchTest, RejectsInputModeBelowOutputMode)
{
  // BODY_RATES cannot be the input of a SPEED output: the controller would have to
  // integrate, not to differentiate
  uint8_t input_mode = 0b00100001;         // BODY_RATES, yaw ANGLE
  const uint8_t output_mode = 0b01000100;  // SPEED, yaw SPEED
  EXPECT_FALSE(checkSuitabilityInputMode(input_mode, output_mode, kControllerModesIn));
}

TEST(FindModeMatchTest, AcceptsSameLevelOutputModeWithoutYaw)
{
  // A yaw NONE output mode has bit 3 set. Masking the output with 7 bits leaked
  // that bit into the level comparison and rejected a same level input mode.
  uint8_t input_mode = 0b01000001;         // SPEED, yaw ANGLE
  const uint8_t output_mode = 0b01001000;  // SPEED, yaw NONE
  EXPECT_TRUE(checkSuitabilityInputMode(input_mode, output_mode, kControllerModesIn));
}

TEST(FindModeMatchTest, CollectsEveryCommonOutputModeOnce)
{
  // Two plugin output modes resolve to the same platform mode, which must be
  // offered once: retrying an identical pair cannot change the plugin answer.
  const std::vector<uint8_t> platform_modes_in = {0b01000100, 0b01100100};
  const std::vector<uint8_t> controller_modes_out = {0b01000100, 0b01000101, 0b01100100};

  const auto output_modes =
    findOutputModes(kNoPreferredMode, controller_modes_out, platform_modes_in);
  EXPECT_EQ(output_modes, std::vector<uint8_t>({0b01000100, 0b01100100}));
}

TEST(FindModeMatchTest, SkipsUnsetAndHoverOutputModes)
{
  // Neither can drive a platform, whatever the platform declares.
  const std::vector<uint8_t> platform_modes_in = {0b00000000, 0b00010000, 0b01000100};
  const std::vector<uint8_t> controller_modes_out = {0b00000000, 0b00010000, 0b01000100};

  const auto output_modes =
    findOutputModes(kNoPreferredMode, controller_modes_out, platform_modes_in);
  EXPECT_EQ(output_modes, std::vector<uint8_t>({0b01000100}));
}

TEST(FindModeMatchTest, PreferredOutputModeGoesFirstWithoutHidingTheRest)
{
  // The plugin can still refuse the preferred pair, so the others must survive.
  const std::vector<uint8_t> platform_modes_in = {0b01000100, 0b01100100};
  const std::vector<uint8_t> controller_modes_out = {0b01000100, 0b01100100};

  const auto output_modes =
    findOutputModes(0b01100100, controller_modes_out, platform_modes_in);
  EXPECT_EQ(output_modes, std::vector<uint8_t>({0b01100100, 0b01000100}));
}

TEST(FindModeMatchTest, PairsEveryCompatibleOutputModeWithItsInputMode)
{
  // One pair per output mode, in the order they are to be tried.
  const std::vector<uint8_t> platform_modes_in = {0b01000100, 0b01100100};
  const std::vector<uint8_t> controller_modes_out = {0b01000100, 0b01100100};

  const auto mode_pairs = findModePairs(
    0b01110111, kNoPreferredMode, kControllerModesIn, controller_modes_out, platform_modes_in);

  const std::vector<std::pair<uint8_t, uint8_t>> expected = {
    {0b01110101, 0b01000100},
    {0b01110101, 0b01100100},
  };
  EXPECT_EQ(mode_pairs, expected);
}

TEST(FindModeMatchTest, ReturnsNoPairWhenThePlatformSharesNoOutputMode)
{
  // An ATTITUDE-only platform cannot be fed by a speed-only plugin.
  const std::vector<uint8_t> platform_modes_in = {0b00110000};

  const auto mode_pairs = findModePairs(
    0b01100011, kNoPreferredMode, kControllerModesIn, kControllerModesOut, platform_modes_in);
  EXPECT_TRUE(mode_pairs.empty());
}
