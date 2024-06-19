// Copyright 2023 Universidad Politécnica de Madrid
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


/*!*******************************************************************************************
 *  \file       speed_motion.hpp
 *  \brief      This file contains the definition of the SpeedMotion class.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_MOTION_REFERENCE_HANDLERS__SPEED_MOTION_HPP_
#define AS2_MOTION_REFERENCE_HANDLERS__SPEED_MOTION_HPP_

#include <string>

#include <as2_core/utils/tf_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"
#include "as2_motion_reference_handlers/basic_motion_references.hpp"

namespace as2
{
namespace motionReferenceHandlers
{

/**
 * @brief The SpeedMotion class is a motion reference handler that moves the robot
 *       at a given speed.
 */
class SpeedMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler
{
public:
  /**
     * @brief PositionMotion Constructor.
     * @param node as2::Node pointer.
     */
  explicit SpeedMotion(as2::Node * node_ptr, const std::string & ns = "");
  ~SpeedMotion()
  {
  }

public:
  /**
     * @brief ownSendCommand sends pose and twist messages.
     * @return true if commands was sent successfully, false otherwise.
     */
  bool ownSendCommand();

  /**
     * @brief sendSpeedCommandWithYawAngle sends a speed command to the
     *       robot.
     *       The speed command is sent in the input frame id.
     *       The linear velocity is given in m/s.
     * @param frame_id_speed frame id of the velocity command.
     * @param vx Linear velocity in the x axis.
     * @param vy Linear velocity in the y axis.
     * @param vz Linear velocity in the z axis.
     * @param frame_id_yaw frame id of the yaw angle command.
     * @param yaw_angle Yaw angle in radians.
     * @return true if the command was sent successfully, false otherwise.
     */
  bool sendSpeedCommandWithYawAngle(
    const std::string & frame_id_speed,
    const float & vx,
    const float & vy,
    const float & vz,
    const std::string & frame_id_yaw,
    const float & yaw_angle);
  /**
     * @brief sendSpeedCommandWithYawAngle sends a speed command to the
     *      robot.
     *      The speed command is sent in the input frame id.
     *      The linear velocity is given in m/s.
     * @param frame_id_speed frame id of the velocity command.
     * @param vx Linear velocity in the x axis.
     * @param vy Linear velocity in the y axis.
     * @param vz Linear velocity in the z axis.
     * @param frame_id_yaw frame id of the yaw angle command.
     * @param q Quaternion that represents the yaw angle.
     * @return true if the command was sent successfully, false otherwise.
     */
  bool sendSpeedCommandWithYawAngle(
    const std::string & frame_id_speed,
    const float & vx,
    const float & vy,
    const float & vz,
    const std::string & frame_id_yaw,
    const geometry_msgs::msg::Quaternion & q);
  /**
     * @brief sendSpeedCommandWithYawAngle sends a speed command to the
     *     robot.
     *     The speed command is sent in the frame id frame.
     *     The linear velocity is given in m/s.
     * @param pose PoseStamped message that represents the rotation .
     * @param twist TwistStamped message that represents the linear velocity.
     * @return true if the command was sent successfully, false otherwise.
     */
  bool sendSpeedCommandWithYawAngle(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TwistStamped & twist);

  /**
     * @brief sendSpeedCommandWithYawSpeed sends a speed command to the
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
  bool sendSpeedCommandWithYawSpeed(
    const std::string & frame_id,
    const float & vx,
    const float & vy,
    const float & vz,
    const float & yaw_speed);

  /**
     * @brief sendSpeedCommandWithYawSpeed sends a speed command to the
     *      robot.
     *      The speed command is sent in the frame id frame.
     *      The linear velocity is given in m/s.
     *      The yaw speed is given in rad/s.
     * @param twist TwistStamped message that represents the linear velocity and the angular yaw speed
     * @return true if the command was sent successfully, false otherwise.
     */
  bool sendSpeedCommandWithYawSpeed(const geometry_msgs::msg::TwistStamped & twist);
};

}    // namespace motionReferenceHandlers
}  // namespace as2

#endif  // AS2_MOTION_REFERENCE_HANDLERS__SPEED_MOTION_HPP_
