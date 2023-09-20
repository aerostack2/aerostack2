/*!*******************************************************************************************
 *  \file       position_motion.cpp
 *  \brief      This file contains the implementation of the PositionMotion class.
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

#include "as2_motion_reference_handlers/position_motion.hpp"

namespace as2 {
namespace motionReferenceHandlers {
PositionMotion::PositionMotion(as2::Node *node_ptr, const std::string &ns)
    : BasicMotionReferenceHandler(node_ptr, ns) {
  desired_control_mode_.yaw_mode        = as2_msgs::msg::ControlMode::NONE;
  desired_control_mode_.control_mode    = as2_msgs::msg::ControlMode::POSITION;
  desired_control_mode_.reference_frame = as2_msgs::msg::ControlMode::UNDEFINED_FRAME;
};

bool PositionMotion::ownSendCommand() {
  bool send_pose  = sendPoseCommand();
  bool send_twist = sendTwistCommand();
  return send_pose && send_twist;
};

bool PositionMotion::sendPositionCommandWithYawAngle(const std::string &frame_id_pose,
                                                     const float &x,
                                                     const float &y,
                                                     const float &z,
                                                     const float &yaw_angle,
                                                     const std::string &frame_id_twist,
                                                     const float &vx = 0.0f,
                                                     const float &vy = 0.0f,
                                                     const float &vz = 0.0f) {
  return sendPositionCommandWithYawAngle(
      frame_id_pose, x, y, z, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)),
      frame_id_twist, vx, vy, vz);
};

bool PositionMotion::sendPositionCommandWithYawAngle(const std::string &frame_id_pose,
                                                     const float &x,
                                                     const float &y,
                                                     const float &z,
                                                     const geometry_msgs::msg::Quaternion &q,
                                                     const std::string &frame_id_twist,
                                                     const float &vx = 0.0f,
                                                     const float &vy = 0.0f,
                                                     const float &vz = 0.0f) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id  = frame_id_pose;
  pose_msg.pose.position.x  = x;
  pose_msg.pose.position.y  = y;
  pose_msg.pose.position.z  = z;
  pose_msg.pose.orientation = q;

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.frame_id = frame_id_twist;
  twist_msg.twist.linear.x  = vx;
  twist_msg.twist.linear.y  = vy;
  twist_msg.twist.linear.z  = vz;

  rclcpp::Time stamp     = node_ptr_->now();
  pose_msg.header.stamp  = stamp;
  twist_msg.header.stamp = stamp;

  return sendPositionCommandWithYawAngle(pose_msg, twist_msg);
};

bool PositionMotion::sendPositionCommandWithYawAngle(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
  if (pose.header.frame_id == "" || twist.header.frame_id == "") {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Frame id is empty");
    return false;
  }

  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  this->command_pose_msg_        = pose;
  this->command_twist_msg_       = twist;

  return this->ownSendCommand();
};

bool PositionMotion::sendPositionCommandWithYawSpeed(const std::string &frame_id_pose,
                                                     const float &x,
                                                     const float &y,
                                                     const float &z,
                                                     const float &yaw_speed,
                                                     const std::string &frame_id_twist,
                                                     const float &vx = 0.0f,
                                                     const float &vy = 0.0f,
                                                     const float &vz = 0.0f) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id_pose;
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.frame_id = frame_id_twist;
  twist_msg.twist.linear.x  = vx;
  twist_msg.twist.linear.y  = vy;
  twist_msg.twist.linear.z  = vz;
  twist_msg.twist.angular.z = yaw_speed;

  rclcpp::Time stamp     = node_ptr_->now();
  pose_msg.header.stamp  = stamp;
  twist_msg.header.stamp = stamp;

  return sendPositionCommandWithYawSpeed(pose_msg, twist_msg);
};

bool PositionMotion::sendPositionCommandWithYawSpeed(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
  if (pose.header.frame_id == "" || twist.header.frame_id == "") {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Frame id is empty");
    return false;
  }
  desired_control_mode_.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  this->command_pose_msg_        = pose;
  this->command_twist_msg_       = twist;

  return this->ownSendCommand();
};
}  // namespace motionReferenceHandlers
}  // namespace as2
