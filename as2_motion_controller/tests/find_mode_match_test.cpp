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
 *  \file       find_mode_match_test.cpp
 *  \brief      Find mode match test
 *  \authors    Miguel Fernández Cortizas
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"

#define MATCH_ALL 0b11111111
#define MATCH_MODE_AND_FRAME 0b11110011
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

std::uint8_t prefered_output_mode_ = 0;

bool checkMatchWithMask(const uint8_t mode1, const uint8_t mode2, const uint8_t mask)
{
  return (mode1 & mask) == (mode2 & mask);
}

uint8_t findBestMatchWithMask(
  const uint8_t mode,
  const std::vector<uint8_t> & mode_list,
  const uint8_t mask)
{
  uint8_t best_match = 0;
  for (const auto & candidate : mode_list) {
    if (checkMatchWithMask(mode, candidate, mask)) {
      best_match = candidate;
      if (candidate == mode) {
        return candidate;
      }
    }
  }
  return best_match;
}

bool checkSuitabilityInputMode(uint8_t & input_mode, const uint8_t output_mode)
{
  // check if input_conversion is in the list of available modes
  bool mode_found = false;
  // Try to math control mode, yaw mode and reference frame
  for (auto & mode : controller_available_modes_in_) {
    if ((input_mode & MATCH_MODE) == HOVER_MODE_MASK && (input_mode & MATCH_MODE) == mode) {
      mode_found = true;
      return true;
    } else if (mode == input_mode) {
      input_mode = mode;
      mode_found = true;
      break;
    }
  }

  // If not match, try to match only control mode and yaw mode
  if (!mode_found) {
    for (auto & mode : controller_available_modes_in_) {
      if (checkMatchWithMask(mode, input_mode, MATCH_MODE_AND_YAW)) {
        input_mode = mode;
        mode_found = true;
        break;
      }
    }
  }

  // check if the input mode is compatible with the output mode
  if ((input_mode & MATCH_MODE) < (output_mode & 0b1111000)) {
    return false;
  }

  return mode_found;
}

bool findSuitableOutputControlModeForPlatformInputMode(
  uint8_t & output_mode,
  const uint8_t input_mode)
{
  //  check if the prefered mode is available
  if (prefered_output_mode_) {
    auto match = findBestMatchWithMask(
      prefered_output_mode_, platform_available_modes_in_,
      MATCH_MODE_AND_YAW);
    if (match) {
      output_mode = match;
      return true;
    }
  }

  // if the prefered mode is not available, search for the first common mode

  uint8_t common_mode = 0;
  bool same_yaw = false;

  for (auto & mode_out : controller_available_modes_out_) {
    // skip unset modes and hover
    if ((mode_out & MATCH_MODE) == UNSET_MODE_MASK || (mode_out & MATCH_MODE) == HOVER_MODE_MASK) {
      continue;
    }
    common_mode = findBestMatchWithMask(mode_out, platform_available_modes_in_, MATCH_MODE_AND_YAW);
    if (common_mode) {
      break;
    }
  }

  // check if the common mode exist
  if (common_mode == 0) {
    return false;
  }
  output_mode = common_mode;
  return true;
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

void test()
{
  // uint8_t input_mode  = 0b00000000; // UNSET
  // uint8_t input_mode  = 0b00010000; // HOVER
  // uint8_t input_mode  = 0b01000000; // SPEED with yaw ANGLE in the LOCAL_FLU_FRAME
  // uint8_t input_mode  = 0b01000001; // SPEED with yaw ANGLE in the GLOBAL_ENU_FRAME
  // uint8_t input_mode  = 0b01000100; // SPEED with yaw SPEED in the LOCAL_FLU_FRAME
  // uint8_t input_mode  = 0b01000101; // SPEED with yaw SPEED in the GLOBAL_ENU_FRAME
  // uint8_t input_mode  = 0b01100001; // POSITION with yaw ANGLE in the GLOBAL_ENU_FRAME
  // uint8_t input_mode  = 0b01100101; // POSITION with yaw SPEED in the GLOBAL_ENU_FRAME
  // uint8_t input_mode  = 0b01110001; // TRAJECTORY with yaw ANGLE in the GLOBAL_ENU_FRAME
  // uint8_t input_mode  = 0b01110101; // TRAJECTORY with yaw SPEED in the GLOBAL_ENU_FRAME

  for (auto & input_mode : controller_available_modes_in_) {
    std::cout << "control_mode request: " << as2::control_mode::controlModeToString(input_mode)
              << std::endl;

    uint8_t output_mode = 0;

    if (findSuitableControlModes(input_mode, output_mode)) {
      auto control_mode_in = as2::control_mode::convertUint8tToAS2ControlMode(input_mode);
      auto control_mode_out = as2::control_mode::convertUint8tToAS2ControlMode(output_mode);
      std::cout << "control_mode in:      "
                << as2::control_mode::controlModeToString(control_mode_in) << std::endl;
      // std::cout << "control_mode out:     "
      //           << as2::control_mode::controlModeToString(control_mode_out) << std::endl;
    } else {
      std::cout << "No suitable control mode found" << std::endl;
    }

    std::cout << "----------------------------------------" << std::endl;
  }
}

int main(int argc, char * argv[])
{
  test();
  return 0;
}
