// Copyright 2024 Universidad Politécnica de Madrid
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

/*!******************************************************************************
 *  \file       as2_map_server.cpp
 *  \brief      Aerostack2 Map Server node.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include "as2_map_server/map_server.hpp"

namespace as2_map_server
{

MapServer::MapServer()
: as2::Node("as2_map_server")
{
  try {
    this->declare_parameter("plugin_name", "mapping_2d");
    this->get_parameter("plugin_name", plugin_name_);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~MapServer();
  }
  plugin_name_ += "::Plugin";
  RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", plugin_name_.c_str());
  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_map_server_plugin_base::MapServerBase>>(
    "as2_map_server", "as2_map_server_plugin_base::MapServerBase");
  try {
    plugin_ptr_ = loader_->createSharedInstance(plugin_name_);
    plugin_ptr_->setup(this);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
    this->~MapServer();
  }
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & _state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
}

CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State & _state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
}

CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & _state)
{
  // Clean other resources here.
  return CallbackReturn::SUCCESS;
}

}  // namespace as2_map_server
