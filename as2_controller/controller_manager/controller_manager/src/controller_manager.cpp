/*!*******************************************************************************************
 *  \file       controller_manager.cpp
 *  \brief      controller_manager main file
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

#include "controller_manager/controller_manager.hpp"

ControllerManager::ControllerManager()
    : as2::Node("controller_manager",
                rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)) {
  /* this->declare_parameter<double>("publish_cmd_freq", 100.0);
  this->declare_parameter<double>("publish_info_freq", 10.0); */
  try {
    this->get_parameter("plugin_name", plugin_name_);
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
                 e.what());
    this->~ControllerManager();
  }

  /* this->declare_parameter<std::filesystem::path>("plugin_config_file",
                                                 "");  // ONLY DECLARED, USED IN LAUNCH
  this->declare_parameter<std::filesystem::path>("plugin_available_modes_config_file", ""); */

  this->get_parameter("publish_cmd_freq", cmd_freq_);
  this->get_parameter("publish_info_freq", info_freq_);
  plugin_name_ += "::Plugin";
  // this->get_parameter("plugin_config_file", parameter_string_);
  this->get_parameter("plugin_available_modes_config_file", available_modes_config_file_);

  loader_ = std::make_shared<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>>(
      "controller_plugin_base", "controller_plugin_base::ControllerBase");
  try {
    controller_ = loader_->createSharedInstance(plugin_name_);
    controller_->initialize(this);
    controller_->reset();
    controller_->updateParams(this->list_parameters({}, 0).names);
    controller_handler_ = std::make_shared<ControllerHandler>(controller_, this);
    RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED [%s]", plugin_name_.c_str());
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                 ex.what());
    return;
  }

  // controller_handler_->initialize(this);
  if (available_modes_config_file_.empty()) {
    available_modes_config_file_ = loader_->getPluginManifestPath(plugin_name_);
  }
  RCLCPP_DEBUG(this->get_logger(), "MODES FILE LOADED: %s",
               available_modes_config_file_.parent_path().c_str());

  config_available_control_modes(available_modes_config_file_.parent_path());

  mode_pub_ = this->create_publisher<as2_msgs::msg::ControllerInfo>(
      as2_names::topics::controller::info, as2_names::topics::controller::qos_info);

  mode_timer_ = this->create_wall_timer(std::chrono::duration<double>(info_freq_),
                                        std::bind(&ControllerManager::mode_timer_callback, this));
};

ControllerManager::~ControllerManager(){};

void ControllerManager::config_available_control_modes(const std::filesystem::path project_path) {
  auto available_input_modes =
      as2::yaml::parse_uint_from_string(as2::yaml::find_tag_from_project_exports_path<std::string>(
          project_path, "input_control_modes"));
  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "AVAILABLE INPUT MODES: ");
  for (auto mode : available_input_modes) {
    RCLCPP_INFO(this->get_logger(), "\t - %s",
                as2::control_mode::controlModeToString(mode).c_str());
  }
  auto available_output_modes =
      as2::yaml::parse_uint_from_string(as2::yaml::find_tag_from_project_exports_path<std::string>(
          project_path, "output_control_modes"));
  RCLCPP_INFO(this->get_logger(), "AVAILABLE OUTPUT MODES: ");
  for (auto mode : available_output_modes) {
    RCLCPP_INFO(this->get_logger(), "\t -%s", as2::control_mode::controlModeToString(mode).c_str());
  }

  RCLCPP_INFO(this->get_logger(), "==========================================================");

  controller_handler_->setInputControlModesAvailables(available_input_modes);
  controller_handler_->setOutputControlModesAvailables(available_output_modes);
};

void ControllerManager::mode_timer_callback() {
  as2_msgs::msg::ControllerInfo msg;
  msg.header.stamp         = this->now();
  controller_handler_->getMode(msg.input_control_mode, msg.output_control_mode);
  mode_pub_->publish(msg);
};
