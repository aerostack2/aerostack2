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
 *  \file       services.hpp
 *  \brief      This file contains the definitions of the services used in aerostack2.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_CORE__NAMES__SERVICES_HPP_
#define AS2_CORE__NAMES__SERVICES_HPP_

#include <rclcpp/rclcpp.hpp>

namespace as2_names
{
namespace services
{
namespace platform
{
const char set_arming_state[] = "set_arming_state";
const char set_offboard_mode[] = "set_offboard_mode";
const char set_platform_control_mode[] = "set_platform_control_mode";
const char takeoff[] = "platform_takeoff";
const char land[] = "platform_land";
const char set_platform_state_machine_event[] = "platform/state_machine_event";
const char list_control_modes[] = "platform/list_control_modes";
}  // namespace platform
namespace controller
{
const char set_control_mode[] = "controller/set_control_mode";
const char list_control_modes[] = "controller/list_control_modes";
}  // namespace controller
namespace motion_reference
{
const char send_traj_wayp[] = "traj_gen/send_traj_wayp";
const char add_traj_wayp[] = "traj_gen/add_traj_wayp";
const char set_traj_speed[] = "traj_gen/set_traj_speed";
}  // namespace motion_reference
namespace gps
{
const char get_origin[] = "get_origin";
const char set_origin[] = "set_origin";
const char path_to_geopath[] = "";
const char geopath_to_path[] = "";
}  // namespace gps
namespace behavior
{
const char package_pickup[] = "behavior/package_pickup";
const char package_unpick[] = "behavior/package_unpick";
const char dynamic_land[] = "behavior/dynamic_land";
const char dynamic_follower[] = "behavior/dynamic_follower";
}  // namespace behavior
const char set_speed[] = "";  // TODO(parias)
}  // namespace services
}  // namespace as2_names

#endif  // AS2_CORE__NAMES__SERVICES_HPP_
