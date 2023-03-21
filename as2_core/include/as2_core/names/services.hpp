/*!*******************************************************************************************
 *  \file       services.hpp
 *  \brief      This file contains the definitions of the services used in aerostack2.
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

#ifndef __AS2_NAMES_SERVICES_HPP__
#define __AS2_NAMES_SERVICES_HPP__

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names {
namespace services {
namespace platform {
const std::string set_arming_state = "set_arming_state";
// const std::string set_arming_state = "platform/set_arming_state";
const std::string set_offboard_mode = "set_offboard_mode";
// const std::string set_offboard_mode = "platform/set_offboard_mode";
const std::string set_platform_control_mode = "set_platform_control_mode";
// const std::string set_control_mode = "platform/set_control_mode";
const std::string takeoff = "platform_takeoff";
// const std::string takeoff = "platform/takeoff";
const std::string land = "platform_land";
// const std::string land = "platform/land";
const std::string set_platform_state_machine_event = "platform/state_machine_event";
// const std::string set_state_machine_event = "platform/state_machine_event";
const std::string list_control_modes = "platform/list_control_modes";
}  // namespace platform
namespace controller {
const std::string set_control_mode   = "controller/set_control_mode";
const std::string list_control_modes = "controller/list_control_modes";
}  // namespace controller
namespace motion_reference {
const std::string send_traj_wayp = "traj_gen/send_traj_wayp";
const std::string add_traj_wayp  = "traj_gen/add_traj_wayp";
const std::string set_traj_speed = "traj_gen/set_traj_speed";
}  // namespace motion_reference
namespace gps {
const std::string get_origin      = "get_origin";
const std::string set_origin      = "set_origin";
const std::string path_to_geopath = "";
const std::string geopath_to_path = "";
}  // namespace gps
namespace behavior {
const std::string package_pickup   = "behavior/package_pickup";
const std::string package_unpick   = "behavior/package_unpick";
const std::string dynamic_land     = "behavior/dynamic_land";
const std::string dynamic_follower = "behavior/dynamic_follower";
}  // namespace behavior
const std::string set_speed = "";  // TODO
}  // namespace services
}  // namespace as2_names

#endif  // __AS2_NAMES_SERVICES_HPP__
