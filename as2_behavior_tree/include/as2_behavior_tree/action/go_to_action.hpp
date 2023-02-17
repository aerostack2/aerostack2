/*!*******************************************************************************************
 *  \file       go_to_action.hpp
 *  \brief      Go to action implementation as behaviour tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
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

#ifndef GO_TO_ACTION_HPP
#define GO_TO_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"

#include "as2_behavior_tree/bt_action_node.hpp"
#include "as2_behavior_tree/port_specialization.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace as2_behaviour_tree {
class GoToAction
    : public nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint> {
public:
  GoToAction(const std::string &xml_tag_name,
             const BT::NodeConfiguration &conf);

  void on_tick();

  void on_wait_for_result(
      std::shared_ptr<const as2_msgs::action::GoToWaypoint::Feedback> feedback);

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("max_speed"), BT::InputPort<double>("yaw_angle"),
         BT::InputPort<geometry_msgs::msg::PointStamped>("pose"),
         BT::InputPort<int>("yaw_mode")});
  }
};

} // namespace as2_behaviour_tree

#endif // GO_TO_ACTION_HPP