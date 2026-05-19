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
 *  @file       controller_manager.hpp
 *  @brief      Controller manager class definition
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_
#define AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_

#include <memory>
#include <chrono>
#include <filesystem>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>

#include "as2_core/node.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/yaml_utils.hpp"
#include "as2_msgs/msg/controller_info.hpp"

#include "controller_handler.hpp"

namespace controller_manager
{

/**
 * @brief ROS 2 node that loads a controller plugin and runs the control loop.
 *
 * Loads the plugin selected by the `plugin_name` parameter through pluginlib,
 * owns the shared TfHandler, hosts the ControllerHandler that orchestrates
 * the control cycle, and publishes the active control modes on
 * `controller/info`.
 */
class ControllerManager : public as2::Node
{
public:
  /**
   * @brief Construct the manager and load the configured controller plugin.
   *
   * @param options ROS 2 node options forwarded to the as2::Node base.
   */
  explicit ControllerManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the manager, releasing the plugin loader and the handler.
   */
  ~ControllerManager();

public:
  /**
   * @brief Control loop frequency, in Hz.
   */
  double cmd_freq_;

private:
  double info_freq_;
  std::string plugin_name_;
  std::filesystem::path available_modes_config_file_;

  as2::tf::TfHandler tf_handler_;
  std::string base_link_frame_id_;

  std::shared_ptr<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>
  loader_;
  std::shared_ptr<controller_handler::ControllerHandler> controller_handler_;
  std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_;
  rclcpp::Publisher<as2_msgs::msg::ControllerInfo>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;

private:
  /**
   * @brief Read the available control modes file and configure the handler.
   *
   * @param project_path Path to the YAML file describing the modes the plugin can handle.
   */
  void configAvailableControlModes(const std::filesystem::path project_path);

  /**
   * @brief Periodic callback that publishes the active control modes on `controller/info`.
   */
  void modeTimerCallback();

  /**
   * @brief Modify the node options to allow undeclared parameters.
   *
   * @param options Original NodeOptions.
   * @return NodeOptions with undeclared parameters allowed.
   */
  static rclcpp::NodeOptions get_modified_options(const rclcpp::NodeOptions & options);
};  // class ControllerManager

}  // namespace controller_manager

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_MANAGER_HPP_
