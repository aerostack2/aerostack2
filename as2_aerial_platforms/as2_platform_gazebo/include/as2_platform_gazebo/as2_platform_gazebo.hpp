// Copyright 2024 Universidad Politécnica de Madrid
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

/**
* @file as2_platform_gazebo.hpp
*
* Implementation of an Gazebo UAV platform
*
* @authors Rafael Pérez Seguí
*/

#ifndef AS2_PLATFORM_GAZEBO__AS2_PLATFORM_GAZEBO_HPP_
#define AS2_PLATFORM_GAZEBO__AS2_PLATFORM_GAZEBO_HPP_

#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"

namespace gazebo_platform
{

class GazeboPlatform : public as2::AerialPlatform
{
public:
  explicit GazeboPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GazeboPlatform() {}

public:
  void configureSensors() {}
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;
  bool ownTakeoff() override;
  bool ownLand() override;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_state_sub_;

private:
  as2_msgs::msg::ControlMode control_in_;
  double yaw_rate_limit_ = M_PI_2;

  bool enable_takeoff_ = false;
  bool enable_land_ = false;
  bool state_received_ = false;
  double current_height_ = 0.0;
  double current_vertical_speed_ = 0.0;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

private:
  void resetCommandTwistMsg();
  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);
};
}  // namespace gazebo_platform

#endif  // AS2_PLATFORM_GAZEBO__AS2_PLATFORM_GAZEBO_HPP_
