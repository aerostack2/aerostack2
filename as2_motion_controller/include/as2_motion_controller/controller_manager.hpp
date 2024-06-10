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
 *  \file       controller_manager.hpp
 *  \brief      Controller manager class definition
 *  \authors    Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_
#define AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_

#include <memory>
#include <chrono>
#include <filesystem>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>

#include "as2_core/node.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/yaml_utils.hpp"
#include "as2_msgs/msg/controller_info.hpp"

#include "controller_handler.hpp"

namespace controller_manager
{

class ControllerManager : public as2::Node
{
public:
  explicit ControllerManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerManager();

public:
  double cmd_freq_;

private:
  double info_freq_;
  std::filesystem::path plugin_name_;
  std::filesystem::path available_modes_config_file_;

  std::shared_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>
  loader_;
  std::shared_ptr<controller_handler::ControllerHandler> controller_handler_;
  std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;

private:
  void configAvailableControlModes(const std::filesystem::path project_path);
  void modeTimerCallback();

  /**
   * @brief Modify the node options to allow undeclared parameters
   */
  static rclcpp::NodeOptions get_modified_options(const rclcpp::NodeOptions & options);
};  // class ControllerManager

}  // namespace controller_manager

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_
