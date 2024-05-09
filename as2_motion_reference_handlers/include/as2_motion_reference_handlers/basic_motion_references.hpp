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
 *  \file       basic_motion_references.hpp
 *  \brief      Virtual class for basic motion references headers
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_
#define AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_

#include <string>

#include <as2_msgs/msg/control_mode.hpp>
#include <as2_msgs/msg/controller_info.hpp>
#include <as2_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/node.hpp"

namespace as2
{
namespace motionReferenceHandlers
{
class BasicMotionReferenceHandler
{
public:
  explicit BasicMotionReferenceHandler(as2::Node * as2_ptr, const std::string & ns = "");
  ~BasicMotionReferenceHandler();

protected:
  as2::Node * node_ptr_;
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

  static rclcpp::Subscription<as2_msgs::msg::ControllerInfo>
  ::SharedPtr controller_info_sub_;
  static as2_msgs::msg::ControlMode current_mode_;

  static rclcpp::Publisher<as2_msgs::msg::TrajectoryPoint>::SharedPtr command_traj_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;

  bool setMode(const as2_msgs::msg::ControlMode & mode);
};

}    // namespace motionReferenceHandlers
}  // namespace as2

#endif  // AS2_MOTION_REFERENCE_HANDLERS__BASIC_MOTION_REFERENCES_HPP_
