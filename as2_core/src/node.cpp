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
 *  \file       node.cpp
 *  \brief      Aerostack2 node implementation file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "as2_core/node.hpp"

std::string as2::Node::generate_global_name(const std::string & name)
{
  if (name.find("/") == 0) {
    return name.substr(1);
  } else {
    return name;
  }
}

std::string as2::Node::generate_local_name(const std::string & name)
{
  if (name.find("/") == 0) {
    return this->get_name() + name;
  } else {
    return std::string(this->get_name()) + "/" + name;
  }
}

namespace as2
{
namespace tf
{
std::string generateTfName(const std::string & _namespace, const std::string & _frame_name);
}  // namespace tf

void Node::warnIfNotStandardFrame(
  const std::string & param_name, const std::string & value, const std::string & standard)
{
  // An empty name is the documented way to ask for the bare namespace, e.g. Gazebo
  if (value == standard || value.empty()) {
    return;
  }
  RCLCPP_WARN(
    this->get_logger(),
    "[%s] is '%s', not the REP-105 name '%s'.", param_name.c_str(), value.c_str(),
    standard.c_str());
}

void Node::initializeCanonicalFrames()
{
  // A leading '/' marks a frame as global: generateTfName() strips it and does
  // not namespace it. earth defaults to it, since every robot shares that frame
  const std::string ns = this->get_namespace();

  const std::string earth = getParameter<std::string>("earth_frame_id", "/earth");
  const std::string map = getParameter<std::string>("map_frame_id", "map");
  const std::string odom = getParameter<std::string>("odom_frame_id", "odom");
  const std::string base = getParameter<std::string>("base_frame_id", "base_link");

  warnIfNotStandardFrame("earth_frame_id", earth, "/earth");
  warnIfNotStandardFrame("map_frame_id", map, "map");
  warnIfNotStandardFrame("odom_frame_id", odom, "odom");
  warnIfNotStandardFrame("base_frame_id", base, "base_link");

  earth_frame_id_ = tf::generateTfName(ns, earth);
  map_frame_id_ = tf::generateTfName(ns, map);
  odom_frame_id_ = tf::generateTfName(ns, odom);
  base_frame_id_ = tf::generateTfName(ns, base);

  // An empty name resolves to the namespace, so two frames left empty collapse into the same
  // id and yield a self transform. Fail here rather than at the first lookup
  const std::vector<std::string> frames =
  {earth_frame_id_, map_frame_id_, odom_frame_id_, base_frame_id_};
  if (std::set<std::string>(frames.begin(), frames.end()).size() != frames.size()) {
    throw std::runtime_error(
            "Canonical TF frames must be distinct, got earth='" + earth_frame_id_ +
            "' map='" + map_frame_id_ + "' odom='" + odom_frame_id_ +
            "' base='" + base_frame_id_ + "'");
  }

  // The namespaced result is what the rest of the stack actually uses, and it is not
  // recoverable from the parameter values alone
  RCLCPP_INFO(this->get_logger(), "Frame names:");
  RCLCPP_INFO(this->get_logger(), "\tEarth frame: %s", earth_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "\tMap frame: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "\tOdom frame: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "\tBase frame: %s", base_frame_id_.c_str());
}

}  // namespace as2
