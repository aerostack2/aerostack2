/*!*******************************************************************************************
 *  \file       speed_in_a_plane_motion.hpp
 *  \brief      This file contains the definition of the SpeedInAPlaneMotion class.
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

#ifndef SPEED_IN_A_PLANE_MOTION_COMMANDS_HPP
#define SPEED_IN_A_PLANE_MOTION_COMMANDS_HPP

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
 * @brief The SpeedInAPlaneMotion class is a motion reference handler that moves the robot
 *       at a given speed.
 */
class SpeedInAPlaneMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler {
public:
  /**
   * @brief PositionMotion Constructor.
   * @param node as2::Node pointer.
   */
  SpeedInAPlaneMotion(as2::Node *node_ptr, const std::string &ns = "");
  ~SpeedInAPlaneMotion(){};

public:
  /**
   * @brief ownSendCommand sends pose and twist messages.
   * @return true if commands was sent successfully, false otherwise.
   */
  bool ownSendCommand();

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed in a plane command to the
   *     robot with a yaw speed.
   *     The speed plane command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   *     The height command is sent in the frame id frame.
   *     The height is given in m.
   * @param frame_id_speed frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param frame_id_yaw frame id of the height and yaw angle command.
   * @param hz Height in the z axis.
   * @param yaw_speed Yaw speed in radians/s.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawSpeed(const std::string &frame_id_speed,
                                            const float vx,
                                            const float vy,
                                            const std::string &frame_id_pose,
                                            const float hz,
                                            const float yaw_speed);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed in a plane command to the
   *     robot with a yaw speed.
   *     The speed plane command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   *     The height command is sent in the frame id frame.
   *     The height is given in m.
   * @param pose PoseStamped message that represents the height.
   * @param twist TwistStamped message that represents the linear velocity and yaw speed.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawSpeed(const geometry_msgs::msg::PoseStamped &pose,
                                            const geometry_msgs::msg::TwistStamped &twist);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed in a plane command to the
   *     robot with a yaw angle.
   *     The speed plane command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   *     The height command is sent in the frame id frame.
   *     The height is given in m.
   * @param frame_id_speed frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param frame_id_yaw frame id of the height and yaw angle command.
   * @param hz Height in the z axis.
   * @param yaw_angle Yaw angle in radians.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawAngle(const std::string &frame_id_speed,
                                            const float vx,
                                            const float vy,
                                            const std::string &frame_id_pose,
                                            const float hz,
                                            const float yaw_angle);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed in a plane command to the
   *     robot with a yaw angle.
   *     The speed plane command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   *     The height command is sent in the frame id frame.
   *     The height is given in m.
   * @param frame_id_speed frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param frame_id_yaw frame id of the height and yaw angle command.
   * @param hz Height in the z axis.
   * @param q Quaternion that represents the yaw angle.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawAngle(const std::string &frame_id_speed,
                                            const float vx,
                                            const float vy,
                                            const std::string &frame_id_pose,
                                            const float hz,
                                            const geometry_msgs::msg::Quaternion &q);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed in a plane command to the
   *     robot with a yaw angle.
   *     The speed plane command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   *     The height command is sent in the frame id frame.
   *     The height is given in m.
   * @param pose PoseStamped message that represents the height and yaw angle.
   * @param twist TwistStamped message that represents the linear velocity.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawAngle(const geometry_msgs::msg::PoseStamped &pose,
                                            const geometry_msgs::msg::TwistStamped &twist);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed command to the
   *       robot.
   *       The speed command is sent in the input frame id.
   *       The linear velocity is given in m/s.
   * @param frame_id_speed frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param hz Height in the z axis.
   * @param frame_id_yaw frame id of the yaw angle command.
   * @param yaw_angle Yaw angle in radians.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawAngle(const std::string &frame_id_speed,
                                            const float &vx,
                                            const float &vy,
                                            const std::string &frame_id_pose,
                                            const float &hz,
                                            const float &yaw_angle);
  /**
   * @brief sendSpeedInAPlaneCommandWithYawAngle sends a speed command to the
   *      robot.
   *      The speed command is sent in the input frame id.
   *      The linear velocity is given in m/s.
   * @param frame_id_speed frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param hz Height in the z axis.
   * @param frame_id_yaw frame id of the yaw angle command.
   * @param q Quaternion that represents the yaw angle.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawAngle(const std::string &frame_id_speed,
                                            const float &vx,
                                            const float &vy,
                                            const std::string &frame_id_pose,
                                            const float &hz,
                                            const geometry_msgs::msg::Quaternion &q);

  /**
   * @brief sendSpeedInAPlaneCommandWithYawSpeed sends a speed command to the
   *      robot.
   *      The speed command is sent in the input frame id.
   *      The linear velocity is given in m/s.
   *      The yaw speed is given in rad/s.
   * @param frame_id frame id of the velocity command.
   * @param vx Linear velocity in the x axis.
   * @param vy Linear velocity in the y axis.
   * @param vz Linear velocity in the z axis.
   * @param yaw_speed Yaw speed in rad/s.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedInAPlaneCommandWithYawSpeed(const std::string &frame_id,
                                            const float &vx,
                                            const float &vy,
                                            const std::string &frame_id_pose,
                                            const float &hz,
                                            const float &yaw_speed);
};

}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // SPEED_IN_A_PLANE_MOTION_COMMANDS_HPP
