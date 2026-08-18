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
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
// # bits [1:0] are reserved and ignored
// #
// #-----------------------------------------------------------------

#define MATCH_ALL 0b11111111
#define MATCH_CONTROL_MODE 0b11110000
#define MATCH_YAW_MODE 0b00001100
#define MATCH_RESERVED_BITS 0b00000011
#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

/**
 * @brief Encode a control mode into its uint8_t representation.
 *
 * @param mode Control mode to encode. Its reserved bits are left to zero.
 * @return Encoded control mode.
 */
uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode & mode);

/**
 * @brief Decode a control mode from its uint8_t representation.
 *
 * @param control_mode_uint8t Encoded control mode. Bits [1:0] are ignored.
 * @return Decoded control mode.
 */
as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t);

/**
 * @brief Get a human readable name of an encoded control mode.
 *
 * @param control_mode_uint8t Encoded control mode.
 * @return Control mode and yaw mode, as text.
 */
std::string controlModeToString(const uint8_t control_mode_uint8t);

/**
 * @brief Get a human readable name of a control mode.
 *
 * @param mode Control mode.
 * @return Control mode and yaw mode, as text.
 */
std::string controlModeToString(const as2_msgs::msg::ControlMode & mode);

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
 * @brief Find, in a list of control modes, the best match for a requested mode.
 *
 * A candidate matches when it is equal to the requested mode under @p mask. An
 * exact match (every bit, mask included) always wins; otherwise the first
 * matching candidate in list order wins, so the list order expresses preference.
 *
 * @param mode        Requested control mode, as uint8_t.
 * @param mode_list   Candidate control modes, as uint8_t, in preference order.
 * @param mask        Bits that must be equal for a candidate to match.
 * @param best_match  Output. Matching candidate, untouched when none matches.
 * @return true if a matching candidate was found.
 */
bool findBestMatchWithMask(
  const uint8_t mode, const std::vector<uint8_t> & mode_list, const uint8_t mask,
  uint8_t & best_match);

/**
 * @brief Resolve a requested control mode against a list of available modes.
 *
 * Control mode and yaw mode must match; the reserved bits [1:0] are ignored.
 * HOVER matches on the control mode alone. The resolved mode is the available
 * one, and not the requested one.
 *
 * @param request         Requested control mode.
 * @param available_modes Available control modes, as uint8_t, in preference order.
 * @param resolved        Output. Available control mode matching the request.
 * @return true if an available control mode matches the request.
 */
bool resolveControlMode(
  const as2_msgs::msg::ControlMode & request, const std::vector<uint8_t> & available_modes,
  as2_msgs::msg::ControlMode & resolved);

/**
 * @brief Command messages a control mode drives, and therefore the frames it needs.
 */
struct CommandFrameUsage
{
  bool pose;
  bool twist;
};

/**
 * @brief Which command messages a control mode uses.
 *
 * Single source of truth for the frames a mode needs: it tells both which
 * frames must be declared for the mode and which commands are converted.
 *
 * @param mode Control mode, typically one already resolved.
 */
CommandFrameUsage getCommandFrameUsage(const as2_msgs::msg::ControlMode & mode);

/**
 * @brief Convert, in place, the command messages a control mode uses to their frames.
 *
 * Conversion is driven by the message header: a command already expressed in
 * its target frame is left untouched. Only the commands the mode uses are
 * converted, see getCommandFrameUsage(). BODY_RATES commands are not, since
 * their rates are body magnitudes by definition, and neither is thrust.
 *
 * @param tf_handler      TF handler used for the lookups.
 * @param pose_frame_id   Target frame of the pose command.
 * @param twist_frame_id  Target frame of the twist command.
 * @param mode            Active control mode.
 * @param pose            Pose command, modified in place when converted.
 * @param twist           Twist command, modified in place when converted.
 * @param trajectory      Trajectory command, modified in place when converted.
 *                        It carries a single header, so the caller must give
 *                        the same frame for the pose and for the twist.
 * @param timeout         TF lookup timeout. Zero uses the latest cached transform.
 * @return true if every command the mode uses is expressed in its target frame.
 *         A command without a frame id in its header cannot be placed and fails.
 */
bool convertCommandsToFrame(
  as2::tf::TfHandler & tf_handler, const std::string & pose_frame_id,
  const std::string & twist_frame_id, const as2_msgs::msg::ControlMode & mode,
  geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & twist,
  as2_msgs::msg::TrajectorySetpoints & trajectory,
  const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero());

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
