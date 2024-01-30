/*!*******************************************************************************************
 *  \file       speed_motion.hpp
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

#ifndef SPEED_MOTION_COMMANDS_HPP
#define SPEED_MOTION_COMMANDS_HPP

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
 * @brief The SpeedMotion class is a motion reference handler that moves the robot
 *       at a given speed.
 */
class SpeedMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler {
public:
  /**
   * @brief SpeedMotion Constructor.
   * @param node_base_ptr rclcpp::node_interfaces::NodeBaseInterface::SharedPtr pointer.
   * @param node_graph_ptr rclcpp::node_interfaces::NodeGraphInterface::SharedPtr pointer.
   * @param node_parameters_ptr rclcpp::node_interfaces::NodeParametersInterface::SharedPtr pointer.
   * @param node_topics_ptr rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr pointer.
   * @param node_services_ptr rclcpp::node_interfaces::NodeServicesInterface::SharedPtr pointer.
   * @param node_clock_ptr rclcpp::node_interfaces::NodeClockInterface::SharedPtr pointer.
   * @param node_logging_ptr rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr pointer.
   * @param ns namespace.
   */
  SpeedMotion(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
              rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
              rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ptr,
              rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr,
              rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr,
              rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr,
              rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr,
              const std::string &ns = "");

  /**
   * @brief PositionMotion Constructor.
   * @param node as2::Node pointer.
   */
  SpeedMotion(as2::Node *node_ptr, const std::string &ns = "");
  ~SpeedMotion(){};

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
  bool sendSpeedCommandWithYawAngle(const std::string &frame_id_speed,
                                    const float &vx,
                                    const float &vy,
                                    const float &vz,
                                    const std::string &frame_id_yaw,
                                    const float &yaw_angle);
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
  bool sendSpeedCommandWithYawAngle(const std::string &frame_id_speed,
                                    const float &vx,
                                    const float &vy,
                                    const float &vz,
                                    const std::string &frame_id_yaw,
                                    const geometry_msgs::msg::Quaternion &q);
  /**
   * @brief sendSpeedCommandWithYawAngle sends a speed command to the
   *     robot.
   *     The speed command is sent in the frame id frame.
   *     The linear velocity is given in m/s.
   * @param pose PoseStamped message that represents the rotation .
   * @param twist TwistStamped message that represents the linear velocity.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedCommandWithYawAngle(const geometry_msgs::msg::PoseStamped &pose,
                                    const geometry_msgs::msg::TwistStamped &twist);

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
  bool sendSpeedCommandWithYawSpeed(const std::string &frame_id,
                                    const float &vx,
                                    const float &vy,
                                    const float &vz,
                                    const float &yaw_speed);

  /**
   * @brief sendSpeedCommandWithYawSpeed sends a speed command to the
   *      robot.
   *      The speed command is sent in the frame id frame.
   *      The linear velocity is given in m/s.
   *      The yaw speed is given in rad/s.
   * @param twist TwistStamped message that represents the linear velocity and the angular yaw speed
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendSpeedCommandWithYawSpeed(const geometry_msgs::msg::TwistStamped &twist);
};

}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // SPEED_MOTION_COMMANDS_HPP
