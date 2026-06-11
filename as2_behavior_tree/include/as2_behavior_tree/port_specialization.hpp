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

/**
 * @file port_specialization.hpp
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

#ifndef AS2_BEHAVIOR_TREE__PORT_SPECIALIZATION_HPP_
#define AS2_BEHAVIOR_TREE__PORT_SPECIALIZATION_HPP_

#include <vector>
#include <chrono>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "as2_msgs/msg/pose_with_id.hpp"

// Template specialization to converts a string to Position2D.
namespace BT
{
template<>
inline geometry_msgs::msg::Pose convertFromString(BT::StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    geometry_msgs::msg::Pose output;
    output.position.x = convertFromString<double>(parts[0]);
    output.position.y = convertFromString<double>(parts[1]);
    output.position.z = convertFromString<double>(parts[2]);
    return output;
  }
}

template<>
inline geometry_msgs::msg::Point convertFromString(BT::StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    geometry_msgs::msg::Point output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.z = convertFromString<double>(parts[2]);
    return output;
  }
}

template<>
inline geometry_msgs::msg::PointStamped convertFromString(BT::StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    geometry_msgs::msg::PointStamped output;
    output.header.frame_id = "earth";
    output.point.x = convertFromString<double>(parts[0]);
    output.point.y = convertFromString<double>(parts[1]);
    output.point.z = convertFromString<double>(parts[2]);
    return output;
  }
}

template<>
inline std::vector<as2_msgs::msg::PoseWithID>
convertFromString(BT::StringView str)
{
  std::vector<as2_msgs::msg::PoseWithID> output;

  // Split poses by '|'
  auto points = splitString(str, '|');

  if (points.empty()) {
    throw RuntimeError("Empty input for poses");
  }

  output.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i)
  {
    auto parts = splitString(points[i], ';');

    if (parts.size() != 3) {
      throw RuntimeError("Invalid pose format: expected x;y;z");
    }

    as2_msgs::msg::PoseWithID pose;
    pose.id = std::to_string(i);

    pose.pose.position.x = convertFromString<double>(parts[0]);
    pose.pose.position.y = convertFromString<double>(parts[1]);
    pose.pose.position.z = convertFromString<double>(parts[2]);

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    output.emplace_back(std::move(pose));
  }

  return output;
}

template<>
inline std::chrono::milliseconds convertFromString(BT::StringView str)
{
    int ms = convertFromString<int>(str);
    return std::chrono::milliseconds(ms);
}

}  // end namespace BT

#endif  // AS2_BEHAVIOR_TREE__PORT_SPECIALIZATION_HPP_
