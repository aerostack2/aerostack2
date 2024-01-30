/*!*******************************************************************************************
 *  \file       speed_in_a_plane_motion.cpp
 *  \brief      This file contains the implementation of the SpeedInAPlaneMotion class.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
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

#include "as2_motion_reference_handlers/speed_in_a_plane_motion.hpp"

namespace as2 {
namespace motionReferenceHandlers {

SpeedInAPlaneMotion::SpeedInAPlaneMotion(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ptr,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr,
    const std::string &ns)
    : BasicMotionReferenceHandler(node_base_ptr,
                                  node_graph_ptr,
                                  node_parameters_ptr,
                                  node_topics_ptr,
                                  node_services_ptr,
                                  node_clock_ptr,
                                  node_logging_ptr,
                                  ns) {
  desired_control_mode_.yaw_mode        = as2_msgs::msg::ControlMode::NONE;
  desired_control_mode_.control_mode    = as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE;
  desired_control_mode_.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;
}

SpeedInAPlaneMotion::SpeedInAPlaneMotion(as2::Node *node_ptr, const std::string &ns)
    : SpeedInAPlaneMotion(node_ptr->get_node_base_interface(),
                          node_ptr->get_node_graph_interface(),
                          node_ptr->get_node_parameters_interface(),
                          node_ptr->get_node_topics_interface(),
                          node_ptr->get_node_services_interface(),
                          node_ptr->get_node_clock_interface(),
                          node_ptr->get_node_logging_interface(),
                          ns) {}

bool SpeedInAPlaneMotion::ownSendCommand() {
  bool send_pose  = sendPoseCommand();
  bool send_twist = sendTwistCommand();
  return send_pose && send_twist;
};

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawSpeed(const std::string &frame_id_speed,
                                                               const float vx,
                                                               const float vy,
                                                               const std::string &frame_id_pose,
                                                               const float hz,
                                                               const float yaw_speed) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id_pose;
  pose.pose.position.z = hz;

  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame_id_speed;
  twist.twist.linear.x  = vx;
  twist.twist.linear.y  = vy;
  twist.twist.angular.z = yaw_speed;
  return sendSpeedInAPlaneCommandWithYawSpeed(pose, twist);
};

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawSpeed(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
  if (pose.header.frame_id == "" || twist.header.frame_id == "") {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Frame id is empty");
    return false;
  }
  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  this->command_pose_msg_        = pose;
  this->command_twist_msg_       = twist;
  return this->ownSendCommand();
};

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(const std::string &frame_id_speed,
                                                               const float vx,
                                                               const float vy,
                                                               const std::string &frame_id_pose,
                                                               const float hz,
                                                               const float yaw_angle) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id  = frame_id_pose;
  pose.pose.position.z  = hz;
  pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle));

  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame_id_speed;
  twist.twist.linear.x  = vx;
  twist.twist.linear.y  = vy;
  return sendSpeedInAPlaneCommandWithYawAngle(pose, twist);
};

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(
    const std::string &frame_id_speed,
    const float vx,
    const float vy,
    const std::string &frame_id_pose,
    const float hz,
    const geometry_msgs::msg::Quaternion &q) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id  = frame_id_pose;
  pose.pose.position.z  = hz;
  pose.pose.orientation = q;

  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame_id_speed;
  twist.twist.linear.x  = vx;
  twist.twist.linear.y  = vy;
  return sendSpeedInAPlaneCommandWithYawAngle(pose, twist);
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
  if (pose.header.frame_id == "" || twist.header.frame_id == "") {
    RCLCPP_ERROR(node_logging_ptr_->get_logger(), "Frame id is empty");
    return false;
  }
  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  this->command_pose_msg_        = pose;
  this->command_twist_msg_       = twist;
  return this->ownSendCommand();
};

}  // namespace motionReferenceHandlers
}  // namespace as2
