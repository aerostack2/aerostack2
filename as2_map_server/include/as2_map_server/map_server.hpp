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
 *  \file       as2_map_server.hpp
 *  \brief      Aerostack2 Map Server node.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef AS2_MAP_SERVER__MAP_SERVER_HPP_
#define AS2_MAP_SERVER__MAP_SERVER_HPP_

#include <filesystem>
#include <memory>
#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"

#include "plugin_base.hpp"


namespace as2_map_server
{

class MapServer : public as2::Node
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  MapServer();
  virtual ~MapServer() = default;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  std::filesystem::path plugin_name_;
  std::shared_ptr<pluginlib::ClassLoader<as2_map_server_plugin_base::MapServerBase>>
  loader_;
  std::shared_ptr<as2_map_server_plugin_base::MapServerBase> plugin_ptr_;
};  // class MapServer

}  // namespace as2_map_server

#endif  // AS2_MAP_SERVER__MAP_SERVER_HPP_
