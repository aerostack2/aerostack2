/*!*******************************************************************************************
 *  \file       actions.hpp
 *  \brief      This file contains the definition of the actions that can be performed in
 *aerostack2. \authors    Miguel Fernández Cortizas Pedro Arias Pérez David Pérez Saura Rafael Pérez
 *Seguí
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

#ifndef __AS2_NAMES_ACTIONS_HPP__
#define __AS2_NAMES_ACTIONS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names {
namespace actions {
namespace behaviors {
const std::string takeoff             = "TakeoffBehavior";
const std::string gotowaypoint        = "GoToBehavior";
const std::string followreference     = "FollowReferenceBehavior";
const std::string followpath          = "FollowPathBehavior";
const std::string land                = "LandBehavior";
const std::string trajectorygenerator = "TrajectoryGeneratorBehavior";
}  // namespace behaviors
}  // namespace actions
}  // namespace as2_names

#endif  // __AS2_NAMES_ACTIONS_HPP__
