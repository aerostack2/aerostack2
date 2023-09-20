/*!*******************************************************************************************
 *  \file       hover_motion.hpp
 *  \brief      This file contains the definition of the HoverMotion class.
 *  \authors    Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
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

#ifndef HOVER_MOTION_COMMANDS_HPP
#define HOVER_MOTION_COMMANDS_HPP

#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <thread>

#include "as2_core/node.hpp"
#include "basic_motion_references.hpp"

namespace as2 {
namespace motionReferenceHandlers {
/**
 * @brief The HoverMotion class is a motion reference handler that allows the
 *       robot to hover at the current position.
 */
class HoverMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler {
public:
  /**
   * @brief HoverMotion Constructor.
   * @param node as2::Node pointer.
   */
  HoverMotion(as2::Node *node_ptr, const std::string &ns = "");
  ~HoverMotion(){};

public:
  /**
   * @brief Send hover motion command.
   * @returns true if the motion reference was sent successfully.
   */
  bool sendHover();
};

}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // HOVER_MOTION_COMMANDS_HPP
