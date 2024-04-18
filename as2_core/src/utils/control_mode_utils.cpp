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
 *  \file       control_mode_utils.cpp
 *  \brief      Utility functions for handling control modes over the aerostack2 framework
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_core/utils/control_mode_utils.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace as2
{
namespace control_mode
{

uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode & mode)
{
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

  uint8_t control_mode_uint8t = 0;
  switch (mode.control_mode) {
    case as2_msgs::msg::ControlMode::ACRO:
      control_mode_uint8t = 0b00100000;
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      control_mode_uint8t = 0b00110000;
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      control_mode_uint8t = 0b01000000;
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      control_mode_uint8t = 0b01010000;
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      control_mode_uint8t = 0b01100000;
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      control_mode_uint8t = 0b01110000;
      break;
    case as2_msgs::msg::ControlMode::UNSET:
      control_mode_uint8t = 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::HOVER:
      control_mode_uint8t = 0b00010000;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "control_mode not recognized");

      break;
  }

  switch (mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      control_mode_uint8t |= 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      control_mode_uint8t |= 0b00000100;
      break;
    case as2_msgs::msg::ControlMode::NONE:
      control_mode_uint8t |= 0b00001000;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "Yaw mode not recognized");

      break;
  }

  switch (mode.reference_frame) {
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      control_mode_uint8t |= 0b00000000;
      break;
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      control_mode_uint8t |= 0b00000001;
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      control_mode_uint8t |= 0b00000010;
      break;
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
      control_mode_uint8t |= 0b00000011;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "Reference frame not recognized");
      break;
  }
  return control_mode_uint8t;
}

as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t)
{
  as2_msgs::msg::ControlMode mode;
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

  if ((control_mode_uint8t & 0b11110000) == 0b00000000) {
    mode.control_mode = as2_msgs::msg::ControlMode::UNSET;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00010000) {
    mode.control_mode = as2_msgs::msg::ControlMode::HOVER;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00100000) {
    mode.control_mode = as2_msgs::msg::ControlMode::ACRO;
  } else if ((control_mode_uint8t & 0b11110000) == 0b00110000) {
    mode.control_mode = as2_msgs::msg::ControlMode::ATTITUDE;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01000000) {
    mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01010000) {
    mode.control_mode = as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01100000) {
    mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
  } else if ((control_mode_uint8t & 0b11110000) == 0b01110000) {
    mode.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "Control mode not recognized");
  }

  if ((control_mode_uint8t & 0b00001100) == 0b00000100) {
    mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  } else if ((control_mode_uint8t & 0b00000110) == 0b00000000) {
    mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  } else if ((control_mode_uint8t & 0b00000110) == 0b00001000) {
    mode.yaw_mode = as2_msgs::msg::ControlMode::NONE;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "Yaw mode not recognized");
  }

  if ((control_mode_uint8t & 0b00000011) == 0b00000001) {
    mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  } else if ((control_mode_uint8t & 0b00000011) == 0b00000010) {
    mode.reference_frame = as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML;
  } else if ((control_mode_uint8t & 0b00000011) == 0b00000000) {
    mode.reference_frame = as2_msgs::msg::ControlMode::BODY_FLU_FRAME;
  } else if ((control_mode_uint8t & 0b00000011) == 0b00000011) {
    mode.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("as2_mode"), "Reference frame not recognized");
  }

  return mode;
}

std::string controlModeToString(const as2_msgs::msg::ControlMode & mode)
{
  std::stringstream ss;
  switch (mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET: {
        ss << "UNSET ";
        return ss.str();
      } break;
    case as2_msgs::msg::ControlMode::HOVER: {
        ss << "HOVER ";
      } break;
    case as2_msgs::msg::ControlMode::ACRO:
      ss << "ACRO ";
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      ss << "ATTITUDE ";
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      ss << "SPEED ";
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      ss << "SPEED_IN_A_PLANE ";
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      ss << "POSITION ";
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      ss << "TRAJECTORY ";
      break;
    default:
      ss << "Control mode not recognized" << std::endl;
      break;
  }

  // ss << "\t\tYaw mode: ";
  switch (mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      ss << "YAW_SPEED ";
      break;
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      ss << "YAW_ANGLE ";
      break;
    case as2_msgs::msg::ControlMode::NONE:
      ss << "YAW_NONE ";
      break;
    default:
      ss << "Yaw mode not recognized" << std::endl;
      break;
  }

  // ss << "\t\tReference frame: ";
  switch (mode.reference_frame) {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      ss << "LOCAL_ENU_FRAME ";
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      ss << "GLOBAL_LAT_LONG_ASML ";
      break;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      ss << "BODY_FLU_FRAME ";
      break;
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
      ss << "UNDEFINED_FRAME ";
      break;
    default:
      ss << "Reference frame not recognized" << std::endl;
      break;
  }

  return ss.str();
}

std::string controlModeToString(const uint8_t control_mode_uint8t)
{
  as2_msgs::msg::ControlMode mode = convertUint8tToAS2ControlMode(control_mode_uint8t);
  return controlModeToString(mode);
}

void printControlMode(const as2_msgs::msg::ControlMode & mode)
{
  RCLCPP_INFO(
    rclcpp::get_logger("as2_mode"), "Control mode: %s", controlModeToString(mode).c_str());
}

void printControlMode(uint8_t control_mode_uint8t)
{
  printControlMode(convertUint8tToAS2ControlMode(control_mode_uint8t));
}

}  // namespace control_mode
}  // namespace as2
