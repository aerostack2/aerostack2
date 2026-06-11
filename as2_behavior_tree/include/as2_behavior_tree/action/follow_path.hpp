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
 * @file follow_path.hpp
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

#ifndef AS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_HPP_

#include <string>
#include <memory>
#include <vector>

#include "as2_behavior_tree/bt_action_node.hpp"

#include "as2_core/names/actions.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

namespace as2_behavior_tree
{
class FollowPathAction
  : public as2_behavior_tree::BtActionNode<as2_msgs::action::FollowPath>
{
public:
  FollowPathAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : as2_behavior_tree::BtActionNode<as2_msgs::action::FollowPath>(
      xml_tag_name, as2_names::actions::behaviors::followpath, conf) {}

  void on_tick()
  {
    getInput("frame_id", frame_id_);
    getInput("path", path_);
    getInput("speed", max_speed_);
    getInput("max_speed_x", max_speed_x_);
    getInput("max_speed_y", max_speed_y_);
    getInput("max_speed_z", max_speed_z_);
    getInput("yaw_mode", yaw_mode_);
    getInput("yaw_angle", yaw_angle_);
    goal_.header.frame_id = frame_id_;
    goal_.path = path_;  // TODO(pariaspe): improve with port_specialization
    goal_.max_speed = max_speed_;
    goal_.max_speed_x = max_speed_x_;
    goal_.max_speed_y = max_speed_y_;
    goal_.max_speed_z = max_speed_z_;
    goal_.yaw.mode = yaw_mode_;
    goal_.yaw.angle = yaw_angle_;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("frame_id"),
      BT::InputPort<std::vector<as2_msgs::msg::PoseWithID>>("path"),
      BT::InputPort<double>("speed", "every direction will use this if the specific ones are set to zero"), 
      BT::InputPort<double>("max_speed_x", "0.0", "if > 0, overwite max_speed param for this direction"),
      BT::InputPort<double>("max_speed_y", "0.0", "if > 0, overwite max_speed param for this direction"),
      BT::InputPort<double>("max_speed_z", "0.0", "if > 0, overwite max_speed param for this direction"),
      BT::InputPort<int>("yaw_mode"),
      BT::InputPort<double>("yaw_angle", 0.0, "for yaw_mode FIXED_YAW (2)"),
    });
  }

  void on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback) {}

private:
  std::string frame_id_;
  std::vector<as2_msgs::msg::PoseWithID> path_;
  double max_speed_;
  double max_speed_x_, max_speed_y_, max_speed_z_;
  int yaw_mode_;
  double yaw_angle_;
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_HPP_
