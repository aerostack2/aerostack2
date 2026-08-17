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
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"

#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/acro.hpp"

namespace gazebo_platform
{

class GazeboPlatform : public as2::AerialPlatform
{
public:
  /**
   * @brief Construct the Gazebo platform, creating the bridge publishers.
   *
   * @param options Node options.
   */
  explicit GazeboPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Gazebo Platform object.
   */
  ~GazeboPlatform() {}

public:
  /**
   * @brief Create the sensor interfaces the platform publishes.
   */
  void configureSensors() {}
  /**
   * @brief Send the current actuator commands to the vehicle.
   *
   * @return true if the command was sent.
   */
  bool ownSendCommand() override;
  /**
   * @brief Arm or disarm the vehicle.
   *
   * @param state True to arm, false to disarm.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetArmingState(bool state) override;
  /**
   * @brief Enter or leave offboard control.
   *
   * @param offboard True to take control, false to release it.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetOffboardControl(bool offboard) override;
  /**
   * @brief Accept a control mode requested through the platform interface.
   *
   * @param msg Requested control mode.
   * @return true if the platform accepts the mode.
   */
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  /**
   * @brief Stop the motors immediately, without landing.
   */
  void ownKillSwitch() override;
  /**
   * @brief Hold the vehicle in place with a zero setpoint.
   */
  void ownStopPlatform() override;
  /**
   * @brief Take off with the platform own takeoff routine.
   *
   * @return true if the takeoff finished successfully.
   */
  bool ownTakeoff() override;
  /**
   * @brief Land with the platform own landing routine.
   *
   * @return true if the landing finished successfully.
   */
  bool ownLand() override;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Publisher<as2_msgs::msg::Acro>::SharedPtr acro_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_state_sub_;

  // Servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

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
  /**
   * @brief Send a zero twist to the simulator, so the vehicle stops moving.
   */
  void resetCommandTwistMsg();
  /**
   * @brief Store the vehicle twist, used by the takeoff and land routines to
   * track the current height and vertical speed.
   *
   * @param _twist_msg Received twist, in the platform body frame.
   */
  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);
  /**
   * @brief Reset the platform to its initial state, for the simulation to be
   * restarted without relaunching the stack.
   *
   * @param request Unused.
   * @param response Always successful.
   */
  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
};
}  // namespace gazebo_platform

#endif  // AS2_PLATFORM_GAZEBO__AS2_PLATFORM_GAZEBO_HPP_
