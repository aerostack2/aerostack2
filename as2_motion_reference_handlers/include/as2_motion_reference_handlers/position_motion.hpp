/*!*******************************************************************************************
 *  \file       position_motion.hpp
 *  \brief      This file contains the definition of the SpeedMotion class.
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

#ifndef POSITION_MOTION_COMMANDS_HPP
#define POSITION_MOTION_COMMANDS_HPP

#include <as2_core/utils/tf_utils.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <thread>

#include "as2_core/node.hpp"
#include "basic_motion_references.hpp"

namespace as2 {
namespace motionReferenceHandlers {

/**
 * @brief The PositionMotion class is a motion reference handler that moves the
 *       robot to a given position.
 */
class PositionMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler {
public:
  /**
   * @brief PositionMotion Constructor.
   * @param node as2::Node pointer.
   */
  PositionMotion(as2::Node *node_ptr, const std::string &ns = "");
  ~PositionMotion(){};

public:
  /**
   * @brief ownSendCommand sends the pose and twist messages.
   * @return true if commands was sent successfully, false otherwise.
   */
  bool ownSendCommand();

  /**
   * @brief sendPositionCommandWithYawAngle sends a position command to the
   *       robot.
   *       The yaw angle is given in radians.
   *       The linear velocity limitation is given in m/s.
   *       The position command and the velocity limitation are sent in the input frame id.
   * @param frame_id_pose frame id of the position command.
   * @param x x coordinate of the position command.
   * @param y y coordinate of the position command.
   * @param z z coordinate of the position command.
   * @param yaw_angle yaw angle of the position command.
   * @param frame_id_twist frame id of the velocity limitation.
   * @param vx linear velocity limitation in x direction.
   * @param vy linear velocity limitation in y direction.
   * @param vz linear velocity limitation in z direction.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendPositionCommandWithYawAngle(const std::string &frame_id_pose,
                                       const float &x,
                                       const float &y,
                                       const float &z,
                                       const float &yaw_angle,
                                       const std::string &frame_id_twist,
                                       const float &vx,
                                       const float &vy,
                                       const float &vz);

  /**
   * @brief sendPositionCommandWithYawAngle sends a position command to the
   *      robot.
   *      The yaw angle is given in a quaternion.
   *      The linear velocity is given in m/s.
   *      The position command and the velocity limitation are sent in the input frame id.
   * @param frame_id_pose frame id of the position command.
   * @param x x coordinate of the position command.
   * @param y y coordinate of the position command.
   * @param z z coordinate of the position command.
   * @param q quaternion of the position command. (with the desired yaw angle).
   * @param frame_id_twist frame id of the velocity limitation.
   * @param vx linear velocity limitation in x direction.
   * @param vy linear velocity limitation in y direction.
   * @param vz linear velocity limitation in z direction.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendPositionCommandWithYawAngle(const std::string &frame_id_pose,
                                       const float &x,
                                       const float &y,
                                       const float &z,
                                       const geometry_msgs::msg::Quaternion &q,
                                       const std::string &frame_id_twist,
                                       const float &vx,
                                       const float &vy,
                                       const float &vz);
  /**
   * @brief sendPositionCommandWithYawAngle sends a position command to the
   *     robot.
   *     The position command is sent in the frame id frame.
   * @param pose geometry_msgs::msg::PoseStamped with the desired position and yaw angle.
   * @param twist geometry_msgs::msg::TwistStamped with the desired linear velocity.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendPositionCommandWithYawAngle(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::TwistStamped &twist);

  /**
   * @brief sendPositionCommandWithYawSpeed sends a position command to the
   *     robot.
   *     The yaw speed is given in rad/s.
   *     The linear velocity is given in m/s.
   *     The position command and the velocity limitation are sent in the input frame id.
   * @param frame_id_pose frame id of the position command.
   * @param x x coordinate of the position command.
   * @param y y coordinate of the position command.
   * @param z z coordinate of the position command.
   * @param yaw_speed yaw speed of the position command.
   * @param frame_id_twist frame id of the velocity limitation.
   * @param vx linear velocity limitation in x direction.
   * @param vy linear velocity limitation in y direction.
   * @param vz linear velocity limitation in z direction.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendPositionCommandWithYawSpeed(const std::string &frame_id_pose,
                                       const float &x,
                                       const float &y,
                                       const float &z,
                                       const float &yaw_speed,
                                       const std::string &frame_id_twist,
                                       const float &vx,
                                       const float &vy,
                                       const float &vz);

  /**
   * @brief sendPositionCommandWithYawSpeed sends a position command to the
   *    robot.
   *    The position command is sent in the frame id frame.
   * @param x pose geometry_msgs::msg::PoseStamped with the desired position.
   * @param twist geometry_msgs::msg::TwistStamped with the desired linear velocity and yaw speed.
   * @return true if the command was sent successfully, false otherwise.
   */

  bool sendPositionCommandWithYawSpeed(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::TwistStamped &twist);
};

}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // POSITION_MOTION_COMMANDS_HPP
