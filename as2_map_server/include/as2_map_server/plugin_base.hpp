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
 *  \file       plugin_base.hpp
 *  \brief      Plugin base class for Map Server node.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include <as2_core/names/topics.hpp>
#include <as2_core/node.hpp>
// TODO(parias): ADD AS CMake deps and package.xml
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#ifndef AS2_MAP_SERVER__PLUGIN_BASE_HPP_
#define AS2_MAP_SERVER__PLUGIN_BASE_HPP_

namespace as2_map_server_plugin_base
{
class MapServerBase
{
protected:
  as2::Node * node_ptr_;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

public:
  MapServerBase() {}
  void setup(
    as2::Node * node)
  {
    node_ptr_ = node;

    twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos);
    pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos);

    // node_ptr_->declare_parameter<std::string>("base_frame", "base_link");
    // node_ptr_->get_parameter("base_frame", base_frame_id_);

    on_setup();
  }

  virtual void on_setup() = 0;
};
}  // namespace as2_map_server_plugin_base

#endif  // AS2_MAP_SERVER__PLUGIN_BASE_HPP_
