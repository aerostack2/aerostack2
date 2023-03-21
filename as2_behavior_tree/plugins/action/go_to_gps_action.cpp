/*!*******************************************************************************************
 *  \file       go_to_gps_action.cpp
 *  \brief      Go to gps action implementation as behavior tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Javier Melero Deza
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

#include "as2_behavior_tree/action/go_to_gps_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_behavior_tree {
GoToGpsAction::GoToGpsAction(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf)
    : nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>(
          xml_tag_name, as2_names::actions::behaviors::gotowaypoint, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  client =
      node_->create_client<as2_msgs::srv::GeopathToPath>("geopath_to_path");
}

void GoToGpsAction::on_tick() {

  getInput("altitude", geopose.pose.position.altitude);
  getInput("latitude", geopose.pose.position.latitude);
  getInput("longitude", geopose.pose.position.longitude);

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "interrupted while waiting for the service. exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(),
                "service: %s not available, waiting again...",
                service_name_.c_str());
  }

  auto request = std::make_shared<as2_msgs::srv::GeopathToPath::Request>();

  request->geo_path.poses.push_back(geopose);

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result,
                                         std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(),
                "failed to receive response from service '%s'",
                service_name_.c_str());
    return;
  }

  goal_.target_pose.point.x =
      (*(result.get()->path.poses.begin())).pose.position.x;
  goal_.target_pose.point.y =
      (*(result.get()->path.poses.begin())).pose.position.y;
  goal_.target_pose.point.z =
      (*(result.get()->path.poses.begin())).pose.position.z;

  getInput("max_speed", goal_.max_speed);
  getInput("yaw_angle", goal_.yaw.angle);
  getInput("yaw_mode",
           goal_.yaw.mode); // TODO --> runtime warning, called
                            // BT::convertFromString() for type [unsigned char]
}

void GoToGpsAction::on_wait_for_result(
    std::shared_ptr<const as2_msgs::action::GoToWaypoint::Feedback> feedback) {}

} // namespace as2_behavior_tree
