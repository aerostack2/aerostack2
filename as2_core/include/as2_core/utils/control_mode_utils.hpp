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
 *  \file       control_mode_utils.hpp
 *  \brief      Utility functions for handling control modes over the aerostack2 framework.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_CORE__UTILS__CONTROL_MODE_UTILS_HPP_
#define AS2_CORE__UTILS__CONTROL_MODE_UTILS_HPP_

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

namespace as2
{
namespace control_mode
{

// # ------------- mode codification (4 bits) ----------------------
// #
// # unset             = 0 = 0b00000000
// # hover             = 1 = 0b00010000
// # body_rates        = 2 = 0b00100000
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

/**
 * @brief Encode a control mode into its uint8_t representation.
 *
 * @param mode Control mode to encode.
 * @return Encoded control mode.
 */
uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode & mode);

/**
 * @brief Decode a control mode from its uint8_t representation.
 *
 * @param control_mode_uint8t Encoded control mode.
 * @return Decoded control mode.
 */
as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t);

/**
 * @brief Get a human readable name of an encoded control mode.
 *
 * @param control_mode_uint8t Encoded control mode.
 * @return Control mode, yaw mode and reference frame, as text.
 */
std::string controlModeToString(const uint8_t control_mode_uint8t);

/**
 * @brief Get a human readable name of a control mode.
 *
 * @param mode Control mode.
 * @return Control mode, yaw mode and reference frame, as text.
 */
std::string controlModeToString(const as2_msgs::msg::ControlMode & mode);

/**
 * @brief Pack a control mode into a uint8_t, without validating its fields.
 *
 * @param mode Control mode to pack.
 * @return Packed control mode.
 */
constexpr uint8_t convertToUint8t(const as2_msgs::msg::ControlMode & mode)
{
  return (mode.control_mode << 4) | (mode.yaw_mode << 2) | mode.reference_frame;
}

/**
 * @brief Pack the fields of a control mode into a uint8_t.
 *
 * @param control_mode_uint8t Control mode field.
 * @param yaw_mode_uint8t Yaw mode field.
 * @param reference_frame_uint8t Reference frame field.
 * @return Packed control mode.
 */
constexpr uint8_t convertToUint8t(
  uint8_t control_mode_uint8t, uint8_t yaw_mode_uint8t, uint8_t reference_frame_uint8t)
{
  return (control_mode_uint8t << 4) | (yaw_mode_uint8t << 2) | reference_frame_uint8t;
}

/**
 * @brief Compare two encoded control modes, under a mask.
 *
 * @param mode1 First encoded control mode.
 * @param mode2 Second encoded control mode.
 * @param mask Bits taken into account. Defaults to every bit.
 * @return true if both modes are equal in the masked bits.
 */
inline bool compareModes(const uint8_t mode1, const uint8_t mode2, const uint8_t mask = MATCH_ALL)
{
  return (mode1 & mask) == (mode2 & mask);
}

/**
 * @brief Compare two control modes, under a mask.
 *
 * @param mode1 First control mode.
 * @param mode2 Second control mode.
 * @param mask Bits taken into account. Defaults to every bit.
 * @return true if both modes are equal in the masked bits.
 */
inline bool compareModes(
  const as2_msgs::msg::ControlMode & mode1, const as2_msgs::msg::ControlMode & mode2,
  const uint8_t mask = MATCH_ALL)
{
  return compareModes(
    convertAS2ControlModeToUint8t(mode1), convertAS2ControlModeToUint8t(mode2), mask);
}

/**
 * @brief Get whether an encoded control mode is UNSET.
 *
 * @param control_mode_uint8t Encoded control mode.
 * @return true if the control mode is UNSET, whatever its other fields are.
 */
inline bool isUnsetMode(const uint8_t control_mode_uint8t)
{
  return compareModes(control_mode_uint8t, UNSET_MODE_MASK, MATCH_CONTROL_MODE);
}

/**
 * @brief Get whether a control mode is UNSET.
 *
 * @param mode Control mode.
 * @return true if the control mode is UNSET, whatever its other fields are.
 */
inline bool isUnsetMode(const as2_msgs::msg::ControlMode & mode)
{
  return mode.control_mode == as2_msgs::msg::ControlMode::UNSET;
}

/**
 * @brief Get whether an encoded control mode is HOVER.
 *
 * @param control_mode_uint8t Encoded control mode.
 * @return true if the control mode is HOVER, whatever its other fields are.
 */
inline bool isHoverMode(const uint8_t control_mode_uint8t)
{
  return compareModes(control_mode_uint8t, HOVER_MODE_MASK, MATCH_CONTROL_MODE);
}

/**
 * @brief Get whether a control mode is HOVER.
 *
 * @param mode Control mode.
 * @return true if the control mode is HOVER, whatever its other fields are.
 */
inline bool isHoverMode(const as2_msgs::msg::ControlMode & mode)
{
  return mode.control_mode == as2_msgs::msg::ControlMode::HOVER;
}

/**
 * @brief Log a control mode with the node logger, at INFO level.
 *
 * @param mode Control mode to log.
 */
void printControlMode(const as2_msgs::msg::ControlMode & mode);

/**
 * @brief Log an encoded control mode with the node logger, at INFO level.
 *
 * @param control_mode_uint8t Encoded control mode to log.
 */
void printControlMode(uint8_t control_mode_uint8t);

}  // namespace control_mode
}  // namespace as2

#endif  // AS2_CORE__UTILS__CONTROL_MODE_UTILS_HPP_
