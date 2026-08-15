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
 *  @file       controller_manager.cpp
 *  @brief      Controller manager class implementation
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#include "as2_motion_controller/controller_manager.hpp"

namespace controller_manager
{

ControllerManager::ControllerManager(const rclcpp::NodeOptions & options)
: as2::Node("controller_manager", get_modified_options(options)),
  tf_handler_(this)
{
  plugin_name_ = this->getParameter<std::string>("plugin_name");
  cmd_freq_ = this->getParameter<double>("cmd_freq");
  info_freq_ = this->getParameter<double>("info_freq");
  available_modes_config_file_ =
    this->getParameter<std::string>("plugin_available_modes_config_file");

  if (cmd_freq_ <= 0.0f) {
    RCLCPP_ERROR(this->get_logger(), "Param cmd_freq must be greater than 0.0");
    assert(cmd_freq_ > 0.0f);
    return;
  }

  if (info_freq_ <= 0.0f) {
    RCLCPP_ERROR(this->get_logger(), "Param info_freq must be greater than 0.0");
    assert(info_freq_ > 0.0f);
    return;
  }

  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_motion_controller_plugin_base::ControllerBase>>(
    "as2_motion_controller", "as2_motion_controller_plugin_base::ControllerBase");
  const std::string pluginlib_class_id = plugin_name_ + "::Plugin";
  try {
    controller_ = loader_->createSharedInstance(pluginlib_class_id);

    // Configure the base before initialize() so that ownInitialize() observes
    // a fully-configured ControllerBase (TF handler, frame ids, parameter
    // namespace).
    controller_->setTfHandler(&tf_handler_);
    controller_->setPluginParamNamespace(plugin_name_);

    controller_->initialize(this);
    controller_->reset();

    // Dispatch the initial parameter bulk through the plugin. The base owns
    // the pending-essentials set (populated at the end of initialize()) and
    // flips the latch + fires onAllParametersRead exactly once when the set
    // first empties.
    auto parameters = this->list_parameters({}, 0);
    std::vector<rclcpp::Parameter> params;
    params.reserve(parameters.names.size());
    for (const auto & param : parameters.names) {
      params.emplace_back(this->get_parameter(param));
    }
    controller_->dispatchParameters(params);

    controller_handler_ =
      std::make_shared<controller_handler::ControllerHandler>(
      controller_, this, &tf_handler_);
    RCLCPP_INFO(
      this->get_logger(), "PLUGIN LOADED [%s])",
      pluginlib_class_id.c_str());
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
      ex.what());
    return;
  }

  // Preserve the legacy directory search only when no exact file was configured.
  const bool read_exact_modes_file = !available_modes_config_file_.empty();
  if (!read_exact_modes_file) {
    // Get the path of the package
    available_modes_config_file_ = loader_->getPluginManifestPath(pluginlib_class_id);

    // Try search if file available_modes.yaml exists in package_folder/config/
    available_modes_config_file_ =
      available_modes_config_file_.parent_path() / "config" / "available_modes.yaml";

    if (!std::filesystem::exists(available_modes_config_file_)) {
      // Try search if file available_modes.yaml exists in
      // package_folder/plugins/<plugin_name>/config/
      available_modes_config_file_ = loader_->getPluginManifestPath(pluginlib_class_id);
      available_modes_config_file_ = available_modes_config_file_.parent_path() / "plugins" /
        plugin_name_ / "config" / "available_modes.yaml";

      if (!std::filesystem::exists(available_modes_config_file_)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Default modes file available_modes.yaml not found in plugin config folder: %s",
          available_modes_config_file_.c_str());
        return;
      }
    }
  }

  const auto modes_config_source = read_exact_modes_file ?
    available_modes_config_file_ : available_modes_config_file_.parent_path();

  RCLCPP_DEBUG(
    this->get_logger(), "MODES CONFIG LOADED: %s",
    modes_config_source.c_str());

  configAvailableControlModes(modes_config_source, read_exact_modes_file);

  mode_pub_ = this->create_publisher<as2_msgs::msg::ControllerInfo>(
    as2_names::topics::controller::info, as2_names::topics::controller::qos_info);

  mode_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0f / info_freq_),
    std::bind(&ControllerManager::modeTimerCallback, this));
}

ControllerManager::~ControllerManager() {}

void ControllerManager::configAvailableControlModes(
  const std::filesystem::path & config_path, bool read_exact_file)
{
  const auto find_modes = [&config_path, read_exact_file](const std::string & tag) {
      if (read_exact_file) {
        return as2::yaml::find_tag_in_yaml_file<std::string>(config_path, tag);
      }
      return as2::yaml::find_tag_from_project_exports_path<std::string>(config_path, tag);
    };

  auto available_input_modes =
    as2::yaml::parse_uint_from_string(
    find_modes("input_control_modes"));
  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "AVAILABLE INPUT MODES: ");
  for (auto mode : available_input_modes) {
    RCLCPP_INFO(
      this->get_logger(), "\t - %s",
      as2::control_mode::controlModeToString(mode).c_str());
  }
  auto available_output_modes =
    as2::yaml::parse_uint_from_string(
    find_modes("output_control_modes"));
  RCLCPP_INFO(this->get_logger(), "AVAILABLE OUTPUT MODES: ");
  for (auto mode : available_output_modes) {
    RCLCPP_INFO(this->get_logger(), "\t -%s", as2::control_mode::controlModeToString(mode).c_str());
  }

  RCLCPP_INFO(this->get_logger(), "==========================================================");

  controller_handler_->setInputControlModesAvailables(available_input_modes);
  controller_handler_->setOutputControlModesAvailables(available_output_modes);
}

void ControllerManager::modeTimerCallback()
{
  as2_msgs::msg::ControllerInfo msg;
  msg.header.stamp = this->now();
  controller_handler_->getMode(msg.input_control_mode, msg.output_control_mode);
  mode_pub_->publish(msg);
}

rclcpp::NodeOptions ControllerManager::get_modified_options(const rclcpp::NodeOptions & options)
{
  rclcpp::NodeOptions modified_options = options;
  modified_options.allow_undeclared_parameters(true);
  modified_options.automatically_declare_parameters_from_overrides(true);
  return modified_options;
}

}  // namespace controller_manager
