/*!*******************************************************************************************
 *  \file       is_target_detected_condition.hpp
 *  \brief      Behaviour tree node to check if target is detected and close
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

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_service_node.hpp"

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"

// TODO: not working
namespace as2_behaviour_tree {
class FollowPath : public nav2_behavior_tree::BtServiceNode<
                       as2_msgs::srv::SendTrajectoryWaypoints> {
public:
  FollowPath(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
      : nav2_behavior_tree::BtServiceNode<
            as2_msgs::srv::SendTrajectoryWaypoints>(xml_tag_name, conf) {
    getInput("path_topic_name", path_topic_name_);
    getInput("speed", max_speed_);
    getInput("yaw_mode", yaw_mode_);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    path_sub_ =
        node_->create_subscription<as2_msgs::msg::TrajectoryWaypointsWithID>(
            path_topic_name_, rclcpp::QoS(10),
            std::bind(&FollowPath::pathCallback, this, std::placeholders::_1),
            sub_option);
  }

  FollowPath() = delete;

  void on_tick() {
    callback_group_executor_.spin_some();
    this->request_->waypoints = this->last_path_;
  }

  // static BT::PortsList providedPorts()
  // {
  //     return {BT::InputPort<std::string>("path_topic_name"),
  //             BT::InputPort<double>("speed"),
  //             BT::OutputPort<int>("yaw_mode")};
  // }

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<std::string>("path_topic_name"),
                               BT::InputPort<double>("speed"),
                               BT::OutputPort<int>("yaw_mode")});
  }

  BT::NodeStatus on_completion(
      std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response>
          response) {
    return response->success ? BT::NodeStatus::SUCCESS
                             : BT::NodeStatus::FAILURE;
  }

private:
  void pathCallback(as2_msgs::msg::TrajectoryWaypointsWithID::SharedPtr msg) {
    this->last_path_ = *(msg);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<as2_msgs::msg::TrajectoryWaypointsWithID>::SharedPtr
      path_sub_;

  std::string path_topic_name_;
  double max_speed_;
  int yaw_mode_;

  as2_msgs::msg::TrajectoryWaypointsWithID last_path_ =
      as2_msgs::msg::TrajectoryWaypointsWithID();
};

} // namespace as2_behaviour_tree

#endif // FOLLOW_PATH_HPP