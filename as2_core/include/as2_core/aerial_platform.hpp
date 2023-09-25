/*!*******************************************************************************************
 *  \file       aerial_platform.hpp
 *  \brief      Aerostack2 Aerial Platformm class header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#ifndef AEROSTACK2_AERIAL_PLATFORM_HPP_
#define AEROSTACK2_AERIAL_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/platform_state_machine.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "aerial_platform_details.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "utils/control_mode_utils.hpp"
#include "utils/yaml_utils.hpp"

namespace as2 {

/**
 * @brief Base class for all Aerial platforms. It provides the basic functionality for the platform.
 *  It is responsible for handling the platform state machine and the platform status.
 *  It also handles the command subscriptions and the basic platform services.
 */
class AerialPlatform : public as2::RosNode, public AerialPlatformDetails {

public:
  /**
   * @brief Construct a new Aerial Platform object, with default parameters.
   *
   */
  AerialPlatform();
  /**
   * @brief Construct a new Aerial Platform object, with default parameters.
   *
   */
  AerialPlatform(const std::string &ns);

  virtual ~AerialPlatform() {
  }

};  // class as2::AerialPlatform

}  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
