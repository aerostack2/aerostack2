/*!*******************************************************************************************
 *  \file       is_target_detected_condition.hpp
 *  \brief      behavior tree node to check if target is detected and close
 *              enough
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

#ifndef FOLLOW_PATH_HPP
#define FOLLOW_PATH_HPP

#include <string>

#include "as2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

namespace as2_behavior_tree {
class FollowPathAction
    : public nav2_behavior_tree::BtActionNode<as2_msgs::action::FollowPath> {
public:
  FollowPathAction(const std::string &xml_tag_name,
                   const BT::NodeConfiguration &conf)
      : nav2_behavior_tree::BtActionNode<as2_msgs::action::FollowPath>(
            xml_tag_name, as2_names::actions::behaviors::followpath, conf) {}

  void on_tick() {
    getInput("path", path_);
    getInput("speed", max_speed_);
    getInput("yaw_mode", yaw_mode_);
    goal_.path = path_; // TODO: improve with port_specialization
    goal_.max_speed = max_speed_;
    goal_.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;
  }

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<std::vector<as2_msgs::msg::PoseWithID>>("path"),
         BT::InputPort<double>("speed"), BT::OutputPort<int>("yaw_mode")});
  }

  void on_wait_for_result(
      std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback) {}

private:
  std::vector<as2_msgs::msg::PoseWithID> path_;
  double max_speed_;
  int yaw_mode_;
};

} // namespace as2_behavior_tree

#endif // FOLLOW_PATH_HPP