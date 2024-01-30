/*!*******************************************************************************************
 *  \file       trajectory_motion.hpp
 *  \brief      this file contains the implementation of the TrajectoryMotion class
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

#ifndef TRAJECTORY_MOTION_COMMANDS_HPP
#define TRAJECTORY_MOTION_COMMANDS_HPP

#include <Eigen/Dense>
#include "as2_core/node.hpp"
#include "basic_motion_references.hpp"

namespace as2 {
namespace motionReferenceHandlers {
/**
 * @brief The TrajectoryMotion class is a motion reference handler that sends a puntual
 *       trajectory reference to the robot. The trajectory point is given by a position, a yaw
 *       angle, a velocity and a acceleration.
 */
class TrajectoryMotion : public as2::motionReferenceHandlers::BasicMotionReferenceHandler {
public:
  /**
   * @brief TrajectoryMotion Constructor.
   * @param node_base_ptr rclcpp::node_interfaces::NodeBaseInterface::SharedPtr pointer.
   * @param node_graph_ptr rclcpp::node_interfaces::NodeGraphInterface::SharedPtr pointer.
   * @param node_parameters_ptr rclcpp::node_interfaces::NodeParametersInterface::SharedPtr pointer.
   * @param node_topics_ptr rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr pointer.
   * @param node_services_ptr rclcpp::node_interfaces::NodeServicesInterface::SharedPtr pointer.
   * @param node_clock_ptr rclcpp::node_interfaces::NodeClockInterface::SharedPtr pointer.
   * @param node_logging_ptr rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr pointer.
   * @param ns namespace.
   */
  TrajectoryMotion(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ptr,
                   rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_ptr,
                   rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ptr,
                   rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_ptr,
                   rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_ptr,
                   rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_ptr,
                   rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ptr,
                   const std::string &ns = "");

  /**
   * @brief TrajectoryMotion constructor.
   * @param node as2::Node pointer.
   */
  TrajectoryMotion(as2::Node *node_ptr, const std::string &ns = "");
  ~TrajectoryMotion(){};

public:
  /**
   * @brief sendTrajectoryCommandWithYawAngle sends a trajectory command to the robot.
   * @param frame_id frame id of trayectory point.
   * @param x x coordinate of the trajectory point.
   * @param y y coordinate of the trajectory point.
   * @param z z coordinate of the trajectory point.
   * @param yaw_angle yaw angle of the trajectory point.
   * @param vx x velocity of the trajectory point.
   * @param vy y velocity of the trajectory point.
   * @param vz z velocity of the trajectory point.
   * @param ax x acceleration of the trajectory point.
   * @param ay y acceleration of the trajectory point.
   * @param az z acceleration of the trajectory point.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendTrajectoryCommandWithYawAngle(const std::string &frame_id,
                                         const double x,
                                         const double y,
                                         const double z,
                                         const double yaw_angle,
                                         const double vx,
                                         const double vy,
                                         const double vz,
                                         const double ax,
                                         const double ay,
                                         const double az);

  /**
   * @brief sendTrajectoryCommandWithYawAngle sends a trajectory command to the robot.
   * @param frame_id frame id of trayectory point.
   * @param yaw_angle yaw angle of the trajectory point.
   * @param positions vector of positions of the trajectory point (x,y,z).
   * @param velocities vector of velocities of the trajectory point (vx,vy,vz).
   * @param accelerations vector of accelerations of the trajectory point (ax,ay,az).
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendTrajectoryCommandWithYawAngle(const std::string &frame_id,
                                         const double yaw_angle,
                                         const std::vector<double> &positions,
                                         const std::vector<double> &velocities,
                                         const std::vector<double> &accelerations);

  /**
   * @brief sendTrajectoryCommandWithYawAngle sends a trajectory command to the robot.
   * @param frame_id frame id of trayectory point.
   * @param yaw_angle yaw angle of the trajectory point.
   * @param positions vector of positions of the trajectory point (x,y,z).
   * @param velocities vector of velocities of the trajectory point (vx,vy,vz).
   * @param accelerations vector of accelerations of the trajectory point (ax,ay,az).
   * @return true if the command was sent successfully, false otherwise.
   */
  bool sendTrajectoryCommandWithYawAngle(const std::string &frame_id,
                                         const double yaw_angle,
                                         const Eigen::Vector3d &positions,
                                         const Eigen::Vector3d &velocities,
                                         const Eigen::Vector3d &accelerations);
};
}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // TRAJECTORY_MOTION_COMMANDS_HPP
