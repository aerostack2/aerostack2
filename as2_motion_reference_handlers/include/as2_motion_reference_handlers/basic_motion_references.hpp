/*!*******************************************************************************************
 *  \file       basic_motion_references.hpp
 *  \brief      Virtual class for basic motion references headers
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

#ifndef BASIC_MOTION_COMMANDS_HPP
#define BASIC_MOTION_COMMANDS_HPP

#include <as2_msgs/msg/control_mode.hpp>
#include <as2_msgs/msg/controller_info.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <string>
#include <thread>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 {
namespace motionReferenceHandlers {
class BasicMotionReferenceHandler {
public:
  BasicMotionReferenceHandler(as2::Node *as2_ptr, const std::string &ns = "");
  ~BasicMotionReferenceHandler();

protected:
  as2::Node *node_ptr_;
  std::string namespace_;

  as2_msgs::msg::TrajectoryPoint command_trajectory_msg_;
  geometry_msgs::msg::PoseStamped command_pose_msg_;
  geometry_msgs::msg::TwistStamped command_twist_msg_;

  as2_msgs::msg::ControlMode desired_control_mode_;

  bool sendPoseCommand();
  bool sendTwistCommand();
  bool sendTrajectoryCommand();
  bool checkMode();

private:
  static int number_of_instances_;

  static rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_info_sub_;
  static as2_msgs::msg::ControlMode current_mode_;

  static rclcpp::Publisher<as2_msgs::msg::TrajectoryPoint>::SharedPtr command_traj_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;

  bool setMode(const as2_msgs::msg::ControlMode &mode);
};

}  // namespace motionReferenceHandlers
}  // namespace as2

#endif  // BASIC_MOTION_COMMANDS_HPP
