// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       coordinate_item.cpp
 *  \brief      AuctionItemPlugin for a 3D target coordinate implementation.
 *              Cost = Euclidean distance from the robot's current XY position to the target.
 *              Z coordinate is stored but has zero weight in the cost evaluation.
 *              Expected AuctionItem features: [x, y, z]
 *              Required state_component: as2_names::topics::self_localization::pose
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "coordinate_item/coordinate_item.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <memory>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <as2_core/names/topics.hpp>

namespace coordinate_item
{

std::shared_ptr<as2_auction_behavior::AuctionItemPluginBase> Plugin::create(
  const as2_msgs::msg::AuctionItem & item_msg) const
{
  auto instance = std::make_shared<Plugin>();
  instance->item_ = item_msg;
  instance->name_ = item_msg.name;
  if (item_msg.features.size() >= 3) {
    instance->x_ = item_msg.features[0];
    instance->y_ = item_msg.features[1];
    instance->z_ = item_msg.features[2];
  } else if (item_msg.features.size() >= 2) {
    instance->x_ = item_msg.features[0];
    instance->y_ = item_msg.features[1];
    RCLCPP_WARN(
      rclcpp::get_logger("coordinate_item"),
      "Item '%s' has %zu features, expected at least 3 (x, y, z). Defaulting z to 0.",
      item_msg.name.c_str(), item_msg.features.size());
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("coordinate_item"),
      "Item '%s' has %zu features, expected at least 3 (x, y, z). Defaulting to origin.",
      item_msg.name.c_str(), item_msg.features.size());
  }
  return instance;
}

float Plugin::evaluate(const StateInterface & state_interface) const
{
  try {
    const auto pose = state_interface.get_value<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose);
    const double dx = pose.pose.position.x - x_;
    const double dy = pose.pose.position.y - y_;
    return static_cast<float>(std::sqrt(dx * dx + dy * dy));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("coordinate_item"),
      "Failed to get pose for item '%s': %s. Is 'pose' in state_component?",
      name_.c_str(), e.what());
    return std::numeric_limits<float>::max();
  }
}

std::string Plugin::to_string() const
{
  return "CoordinateItem(name='" + name_ + "', x=" + std::to_string(x_) + ", y=" +
         std::to_string(y_) + ", z=" + std::to_string(z_) + ")";
}

std::string Plugin::get_name() const
{
  return name_;
}

as2_msgs::msg::AuctionItem Plugin::get_item() const
{
  return item_;
}

}  // namespace coordinate_item

PLUGINLIB_EXPORT_CLASS(coordinate_item::Plugin, as2_auction_behavior::AuctionItemPluginBase)
