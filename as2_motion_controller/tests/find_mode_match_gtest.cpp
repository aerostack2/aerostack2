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
#include <iostream>
#include <stdexcept>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"

#define MATCH_ALL 0b11111111
#define MATCH_MODE 0b11110000
#define MATCH_MODE_AND_YAW 0b11111100

#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

std::vector<uint8_t> controller_available_modes_in_ = {
  0b00000000, 0b00010000, 0b01000000, 0b01000001, 0b01000100,
  0b01000101, 0b01100001, 0b01100101, 0b01110001, 0b01110101};

// With undefined frame
std::vector<uint8_t> controller_request_modes_in_ = {0b00000000, 0b00010011, 0b01000011, 0b01000111,
  0b01100011, 0b01100111, 0b01110111};

std::vector<uint8_t> controller_available_modes_out_ = {0b00000000, 0b01000100, 0b01000101};

std::vector<uint8_t> platform_available_modes_in_ = {0b01000100};

std::uint8_t preferred_output_mode_ = 0;

bool checkSuitabilityInputMode(uint8_t & input_mode, const uint8_t output_mode)
{
  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  // Try to match control mode and yaw mode, the reference frame is settled by
  // the plugin, not by the requester
  for (auto & mode : controller_available_modes_in_) {
    if ((input_mode & MATCH_MODE) == HOVER_MODE_MASK && (input_mode & MATCH_MODE) == mode) {
      mode_found = true;
      return true;
    } else if (as2::control_mode::compareModes(mode, input_mode, MATCH_MODE_AND_YAW)) {
      input_mode = mode;
      mode_found = true;
      break;
    }
  }

  // check if the input mode is compatible with the output mode
  if ((input_mode & MATCH_MODE) < (output_mode & MATCH_MODE)) {
    return false;
  }

  return mode_found;
}

bool findSuitableOutputControlModeForPlatformInputMode(
  uint8_t & output_mode,
  const uint8_t input_mode)
{
  //  check if the preferred mode is available
  uint8_t match = UNSET_MODE_MASK;
  if (preferred_output_mode_) {
    if (as2::control_mode::findBestMatchWithMask(
        preferred_output_mode_, platform_available_modes_in_, MATCH_MODE_AND_YAW, match))
    {
      output_mode = match;
      return true;
    }
  }

  // if the preferred mode is not available, search for the first common mode

  for (auto & mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    if (as2::control_mode::findBestMatchWithMask(
        mode_out, platform_available_modes_in_, MATCH_MODE_AND_YAW, match))
    {
      output_mode = match;
      return true;
    }
  }

  // no common mode exists
  return false;
}

bool findSuitableControlModes(uint8_t & input_mode, uint8_t & output_mode)
{
  // check if the input mode is available. Get the best output mode
  bool success = findSuitableOutputControlModeForPlatformInputMode(output_mode, input_mode);
  if (!success) {
    std::cout << "No suitable output mode found" << std::endl;
    return false;
  }

  // Get the best input mode for the output mode
  success = checkSuitabilityInputMode(input_mode, output_mode);
  if (!success) {
    std::cout << "Input control mode is not suitable for this controller" << std::endl;
    return false;
  }
  return success;
}

TEST(FindModeMatchTest, ResolvesRequestsIgnoringTheirFrame)
{
  // Requests reach the controller with an undefined frame: the mode of the
  // plugin, with its own frame, is the one that must be settled.
  const std::vector<std::pair<uint8_t, uint8_t>> requests_and_modes_in = {
    {0b00010011, 0b00010011},   // HOVER, kept as requested
    {0b01000011, 0b01000000},   // SPEED yaw ANGLE -> first SPEED yaw ANGLE of the plugin
    {0b01000111, 0b01000100},   // SPEED yaw SPEED
    {0b01100011, 0b01100001},   // POSITION yaw ANGLE
    {0b01100111, 0b01100101},   // POSITION yaw SPEED
    {0b01110111, 0b01110101},   // TRAJECTORY yaw SPEED
  };

  for (const auto & [request, expected_mode_in] : requests_and_modes_in) {
    uint8_t input_mode = request;
    uint8_t output_mode = 0;
    EXPECT_TRUE(findSuitableControlModes(input_mode, output_mode))
      << "request " << as2::control_mode::controlModeToString(request);
    EXPECT_EQ(input_mode, expected_mode_in)
      << "request " << as2::control_mode::controlModeToString(request);
    // The only output mode the platform of this scenario supports
    EXPECT_EQ(output_mode, 0b01000100);
  }
}

TEST(FindModeMatchTest, RejectsUnsetInputMode)
{
  // UNSET is below any output mode, so it cannot feed the platform
  uint8_t input_mode = 0b00000000;
  uint8_t output_mode = 0;
  EXPECT_FALSE(findSuitableControlModes(input_mode, output_mode));
}

TEST(FindModeMatchTest, RejectsInputModeBelowOutputMode)
{
  // BODY_RATES cannot be the input of a SPEED output: the controller would have to
  // integrate, not to differentiate
  uint8_t input_mode = 0b00100001;         // BODY_RATES, yaw ANGLE
  const uint8_t output_mode = 0b01000100;  // SPEED, yaw SPEED
  EXPECT_FALSE(checkSuitabilityInputMode(input_mode, output_mode));
}

TEST(FindModeMatchTest, AcceptsSameLevelOutputModeWithoutYaw)
{
  // A yaw NONE output mode has bit 3 set. Masking the output with 7 bits leaked
  // that bit into the level comparison and rejected a same level input mode.
  uint8_t input_mode = 0b01000001;         // SPEED, yaw ANGLE
  const uint8_t output_mode = 0b01001000;  // SPEED, yaw NONE
  EXPECT_TRUE(checkSuitabilityInputMode(input_mode, output_mode));
}
