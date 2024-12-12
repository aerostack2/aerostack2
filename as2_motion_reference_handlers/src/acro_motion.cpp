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
 *  \file       acro_motion.cpp
 *  \brief      This file contains the implementation of the ACROMotion class.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_motion_reference_handlers/acro_motion.hpp"

namespace as2
{
namespace motionReferenceHandlers
{
ACROMotion::ACROMotion(as2::Node * node_ptr, const std::string & ns)
: BasicMotionReferenceHandler(node_ptr, ns)
{
  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::NONE;
  desired_control_mode_.control_mode = as2_msgs::msg::ControlMode::ACRO;
  desired_control_mode_.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;
}

bool ACROMotion::sendACRO(
  const as2_msgs::msg::Thrust & thrust,
  const geometry_msgs::msg::Vector3 & angular_rates)
{
  command_thrust_msg_ = thrust;
  command_twist_msg_.header.stamp = thrust.header.stamp;
  command_twist_msg_.header.frame_id = thrust.header.frame_id;
  command_twist_msg_.twist.angular = angular_rates;
  return this->sendThrustCommand() && this->sendTwistCommand();
}
}    // namespace motionReferenceHandlers
}  // namespace as2
