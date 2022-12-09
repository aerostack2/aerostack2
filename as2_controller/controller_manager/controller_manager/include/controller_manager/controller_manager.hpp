/*!*******************************************************************************************
 *  \file       controller_manager.hpp
 *  \brief      Controller manager class definition
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

#ifndef CONTROLLER_MANAGER_HPP
#define CONTROLLER_MANAGER_HPP

#include <as2_core/node.hpp>
#include <as2_core/utils/control_mode_utils.hpp>
#include <as2_core/utils/yaml_utils.hpp>
#include <as2_msgs/msg/controller_info.hpp>

#include <chrono>
#include <filesystem>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>

#include "controller_handler.hpp"

class ControllerManager : public as2::Node {
public:
  ControllerManager();
  ~ControllerManager();

private:
  void config_available_control_modes(const std::filesystem::path project_path);
  void mode_timer_callback();

public:
  double cmd_freq_;

private:
  double info_freq_;
  std::filesystem::path plugin_name_;
  std::filesystem::path available_modes_config_file_;

  std::shared_ptr<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>> loader_;
  std::shared_ptr<ControllerHandler> controller_handler_;
  std::shared_ptr<controller_plugin_base::ControllerBase> controller_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
};

#endif  // CONTROLLER_MANAGER_HPP
