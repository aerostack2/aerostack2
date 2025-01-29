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
 *  \file       acro_motion.hpp
 *  \brief      This file contains the definition of the ACROMotion class.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_MOTION_REFERENCE_HANDLERS__ACRO_MOTION_HPP_
#define AS2_MOTION_REFERENCE_HANDLERS__ACRO_MOTION_HPP_

#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "as2_motion_reference_handlers/basic_motion_references.hpp"

namespace as2
{
namespace motionReferenceHandlers
{

/**
 * @brief The ACROMotion class is a motion reference handler that moves the
 *       robot to a given acro.
 */
class ACROMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler
{
public:
  /**
     * @brief ACROMotion Constructor.
     * @param node as2::Node pointer.
     */
  explicit ACROMotion(as2::Node * node_ptr, const std::string & ns = "");

  /**
     * @brief ACROMotion Destructor.
     */
  ~ACROMotion() {}

  /**
     * @brief sendACRO sends a acro to the robot.
     *
     * Using the time stamp and frame id from the thrust message.
     * Frame id should be base_link.
     *
     * @param thrust as2_msgs::msg::Thrust to be sent.
     * @param angular_rates geometry_msgs::msg::Vector3 to be sent.
     * @return true if the command was sent successfully, false otherwise.
     */
  bool sendACRO(
    const as2_msgs::msg::Thrust & thrust,
    const geometry_msgs::msg::Vector3 & angular_rates);
};

}    // namespace motionReferenceHandlers
}  // namespace as2

#endif  // AS2_MOTION_REFERENCE_HANDLERS__ACRO_MOTION_HPP_
