/*!*******************************************************************************************
 *  \file       control_mode_utils.hpp
 *  \brief      Utility functions for handling control modes over the aerostack2 framework.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef __CONTROL_MODE_UTILS_HPP__
#define __CONTROL_MODE_UTILS_HPP__

#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "as2_msgs/msg/control_mode.hpp"
#include "rclcpp/logging.hpp"

namespace as2 {
namespace control_mode {

// # ------------- mode codification (4 bits) ----------------------
// #
// # unset             = 0 = 0b00000000
// # hover             = 1 = 0b00010000
// # acro              = 2 = 0b00100000
// # attitude          = 3 = 0b00110000
// # speed             = 4 = 0b01000000
// # speed_in_a_plane  = 5 = 0b01010000
// # position          = 6 = 0b01100000
// # trajectory        = 7 = 0b01110000
// #
// #-------------- yaw codification --------------------------------
// #
// # angle             = 0 = 0b00000000
// # speed             = 1 = 0b00000100
// # none              = 2 = 0b00001000
// #
// # frame codification
// #
// # local_frame_flu   = 0 = 0b00000000
// # global_frame_enu  = 1 = 0b00000001
// # global_frame_lla  = 2 = 0b00000010
// # undefined_frame   = 3 = 0b00000011
// #
// #-----------------------------------------------------------------

#define MATCH_ALL 0b11111111
#define MATCH_CONTROL_MODE 0b11110000
#define MATCH_YAW_MODE 0b00001100
#define MATCH_REFERENCE_FRAME 0b00000011
#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode &mode);
as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t);

std::string controlModeToString(const uint8_t control_mode_uint8t);
std::string controlModeToString(const as2_msgs::msg::ControlMode &mode);

constexpr uint8_t convertToUint8t(const as2_msgs::msg::ControlMode &mode) {
  return (mode.control_mode << 4) | (mode.yaw_mode << 2) | mode.reference_frame;
}

constexpr uint8_t convertToUint8t(uint8_t control_mode_uint8t,
                                  uint8_t yaw_mode_uint8t,
                                  uint8_t reference_frame_uint8t) {
  return (control_mode_uint8t << 4) | (yaw_mode_uint8t << 2) | reference_frame_uint8t;
}

inline bool compareModes(const uint8_t mode1, const uint8_t mode2, const uint8_t mask = MATCH_ALL) {
  return (mode1 & mask) == (mode2 & mask);
}

inline bool compareModes(const as2_msgs::msg::ControlMode &mode1,
                         const as2_msgs::msg::ControlMode &mode2,
                         const uint8_t mask = MATCH_ALL) {
  return compareModes(convertAS2ControlModeToUint8t(mode1), convertAS2ControlModeToUint8t(mode2),
                      mask);
}

inline bool isUnsetMode(const uint8_t control_mode_uint8t) {
  return compareModes(control_mode_uint8t, UNSET_MODE_MASK, MATCH_CONTROL_MODE);
}

inline bool isUnsetMode(const as2_msgs::msg::ControlMode &mode) {
  return mode.control_mode == as2_msgs::msg::ControlMode::UNSET;
}

inline bool isHoverMode(const uint8_t control_mode_uint8t) {
  return compareModes(control_mode_uint8t, HOVER_MODE_MASK, MATCH_CONTROL_MODE);
}

inline bool isHoverMode(const as2_msgs::msg::ControlMode &mode) {
  return mode.control_mode == as2_msgs::msg::ControlMode::HOVER;
}

void printControlMode(const as2_msgs::msg::ControlMode &mode);
void printControlMode(uint8_t control_mode_uint8t);

}  // namespace control_mode
}  // namespace as2

#endif  // __CONTROL_MODE_UTILS_HPP__
