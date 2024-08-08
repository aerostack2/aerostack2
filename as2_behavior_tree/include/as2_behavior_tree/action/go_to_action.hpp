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
 * @file go_to_action.hpp
 *
 * Go to action implementation as behavior tree node
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

#ifndef AS2_BEHAVIOR_TREE__ACTION__GO_TO_ACTION_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__GO_TO_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"

#include "as2_behavior_tree/bt_action_node.hpp"
#include "as2_behavior_tree/port_specialization.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace as2_behavior_tree
{
class GoToAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>
{
public:
  GoToAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void on_tick();

  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::GoToWaypoint::Feedback> feedback);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<double>("max_speed"), BT::InputPort<double>("yaw_angle"),
        BT::InputPort<geometry_msgs::msg::PointStamped>("pose"),
        BT::InputPort<int>("yaw_mode")});
  }
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__GO_TO_ACTION_HPP_
