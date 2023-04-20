/*!*******************************************************************************************
 *  \file       platform_emulator.hpp
 *  \brief      Platform emulator class definition
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

#ifndef PLATFORM_EMULATOR_HPP
#define PLATFORM_EMULATOR_HPP

#include "as2_core/as2_basic_behavior.hpp"
#include "as2_core/names/services.hpp"

class PlatformEmulator : public as2::Node {
public:
  PlatformEmulator() : Node("platform_emulator") {
    set_arming_state_srv_ = this->create_service<std_srvs::srv::SetBool>(
        as2_names::services::platform::set_arming_state,
        std::bind(&PlatformEmulator::setArmingStateSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    set_offboard_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
        as2_names::services::platform::set_offboard_mode,
        std::bind(&PlatformEmulator::setOffboardModeSrvCall, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));
  };

  ~PlatformEmulator(){};

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offboard_mode_srv_;

  void setArmingStateSrvCall(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "ARMED");
    response->message = "Fake Arming: True";
    response->success = true;
  }

  void setOffboardModeSrvCall(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "OFFBOARD");
    response->message = "Fake Offboard: True";
    response->success = true;
  }
};

#endif // LAND_EMULATOR_HPP
