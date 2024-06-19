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


/*!*******************************************************************************
 *  \file       hover_motion.hpp
 *  \brief      This file contains the definition of the HoverMotion class.
 *  \authors    Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 ********************************************************************************/

#ifndef AS2_MOTION_REFERENCE_HANDLERS__HOVER_MOTION_HPP_
#define AS2_MOTION_REFERENCE_HANDLERS__HOVER_MOTION_HPP_

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_motion_reference_handlers/basic_motion_references.hpp"

namespace as2
{
namespace motionReferenceHandlers
{
/**
 * @brief The HoverMotion class is a motion reference handler that allows the
 *       robot to hover at the current position.
 */
class HoverMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler
{
public:
  /**
     * @brief HoverMotion Constructor.
     * @param node as2::Node pointer.
     */
  explicit HoverMotion(as2::Node * node_ptr, const std::string & ns = "");
  ~HoverMotion()
  {
  }

public:
  /**
     * @brief Send hover motion command.
     * @returns true if the motion reference was sent successfully.
     */
  bool sendHover();
};

}   // namespace motionReferenceHandlers
}  // namespace as2

#endif  // AS2_MOTION_REFERENCE_HANDLERS__HOVER_MOTION_HPP_
