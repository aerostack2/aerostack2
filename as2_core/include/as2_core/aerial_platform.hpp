/*!*******************************************************************************************
 *  \file       aerial_platform.hpp
 *  \brief      Aerostack2 Aerial Platformm class header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#ifndef AEROSTACK2_AERIAL_PLATFORM_HPP_
#define AEROSTACK2_AERIAL_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/platform_state_machine.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "utils/control_mode_utils.hpp"
#include "utils/yaml_utils.hpp"

namespace as2 {

/**
 * @brief Base class for all Aerial platforms. It provides the basic functionality for the platform.
 *  It is responsible for handling the platform state machine and the platform status.
 *  It also handles the command subscriptions and the basic platform services.
 */

// TODO: Validate all the system in Pixhawk Class
class AerialPlatform : public as2::Node {
private:
  void initialize();
  bool sending_commands_ = false;

  rclcpp::TimerBase::SharedPtr platform_cmd_timer_;
  rclcpp::TimerBase::SharedPtr platform_info_timer_;
  as2::PlatformStateMachine state_machine_;
  std::vector<uint8_t> available_control_modes_;

protected:
  float cmd_freq_;
  float info_freq_;

  as2_msgs::msg::TrajectoryPoint command_trajectory_msg_;
  geometry_msgs::msg::PoseStamped command_pose_msg_;
  geometry_msgs::msg::TwistStamped command_twist_msg_;
  as2_msgs::msg::Thrust command_thrust_msg_;
  as2_msgs::msg::PlatformInfo platform_info_msg_;

public:
  /**
   * @brief Construct a new Aerial Platform object, with default parameters.
   *
   */
  AerialPlatform(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Construct a new Aerial Platform object, with default parameters.
   *
   */
  AerialPlatform(const std::string &ns, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~AerialPlatform(){};

  /**
   * @brief Configures the platform sensors
   *
   */
  virtual void configureSensors() = 0;

  /**
   * @brief Handles how a command must be sended in the concrete platform.
   *
   * @return true command is sended successfully.
   * @return false command is not sended.
   */
  virtual bool ownSendCommand() = 0;

  /**
   * @brief Handles how arming state has to be settled  in the concrete platform.
   *
   * @param state true for arming the platform, false to disarm.
   * @return true Arming state is settled successfully.
   * @return false Arming state is not settled.
   */
  virtual bool ownSetArmingState(bool state) = 0;

  /**
   * @brief Handles how offboard mode has to be settled  in the concrete platform.
   *
   * @param offboard true if offboard mode is enabled.
   * @return true Offboard mode is settled successfully.
   * @return false Offboard mode is not settled.
   */
  virtual bool ownSetOffboardControl(bool offboard) = 0;

  /**
   * @brief Handles how the control mode has to be settled  in the concrete platform.
   *
   * @param control_mode as2_msgs::msg::PlatformControlMode with the new control mode.
   * @return true Control mode is settled successfully.
   * @return false Control mode is not settled.
   */
  virtual bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) = 0;

  /**
   * @brief Handles the platform takeoff command.
   *
   * @return true Takeoff command is sended successfully.
   * @return false Takeoff command is not sended.
   */
  virtual bool ownTakeoff() { return false; };

  /**
   * @brief Handles the platform landing command.
   *
   * @return true Landing command is sended successfully.
   * @return false Landing command is not sended.
   */
  virtual bool ownLand() { return false; };

  /**
   * @brief Handles the platform emergency kill switch command. This means stop the motors
   * inmediately, this cannot be reversed. USE WITH CAUTION.
   */
  virtual void ownKillSwitch() = 0;

  /**
   * @brief Handles the platform emergency stop command. STOP means to hover as best as possible.
   * This hover is different from the hover in the platform control mode. And when it is activated
   * the platform will stop hearing commands from AS2. USE WITH CAUTION.
   */
  virtual void ownStopPlatform() = 0;

private:
  /**
   * @brief Set the arm state of the platform.
   *
   * @param state True to arm the platform, false to disarm it.
   * @return true Armimg state setted successfully.
   * @return false Armimg state not setted successfully.
   */
  bool setArmingState(bool state);

  /**
   * @brief  Set the offboard control mode.
   *
   * @param offboard  True if the offboard control mode is enabled.
   * @return true if the offboard control mode is setted properly
   * @return false if the offboard control mode could not be setted.
   */
  bool setOffboardControl(bool offboard);

  /**
   * @brief  Set the control mode of the platform.
   *
   * @param msg as2_msgs::msg::ControlMode message with the new control mode desired.
   * @return true  If the control mode is set properly.
   * @return false If the control mode could not be set properly.
   */
  bool setPlatformControlMode(const as2_msgs::msg::ControlMode &msg);

protected:
  /**
   * @brief Send command to the platform.
   * @return true if the command was sent successfully, false otherwise
   */
  virtual void sendCommand();

private:
  void loadControlModes(const std::string &filename);

  // Getters
public:
  /**
   * @brief Set the State Machine Event object
   *
   * @param event Event to
   * @return true
   * @return false
   */
  bool handleStateMachineEvent(const as2_msgs::msg::PlatformStateMachineEvent &event) {
    return state_machine_.processEvent(event);
  };

  bool handleStateMachineEvent(const int8_t &event) { return state_machine_.processEvent(event); };

  /**
   * @brief Get whether the platform is armed or not.
   * @return true Armed
   * @return false Disarmed
   */
  inline bool getArmingState() const { return platform_info_msg_.armed; }

  /**
   * @brief Get wheter the connection is established or not.
   * @return true Connection active
   * @return false Connection not active
   */
  inline bool getConnectedStatus() const { return platform_info_msg_.connected; }

  /**
   * @brief Get whether offboard mode is active or not.
   * @return true Offboard mode enabled
   * @return false Offboard mode disabled
   */
  inline bool getOffboardMode() const { return platform_info_msg_.offboard; }

  /**
   * @brief Get current platform control mode.
   * @return as2_msgs::msg::PlatformControlMode current platform control mode
   */
  inline as2_msgs::msg::ControlMode &getControlMode() {
    return platform_info_msg_.current_control_mode;
  }

  /**
   * @brief Get whether a control mode is active or not.
   * @return true Control mode set and valid
   * @return false Control mode unset
   */
  inline bool isControlModeSettled() const {
    return (platform_info_msg_.current_control_mode.control_mode !=
            platform_info_msg_.current_control_mode.UNSET);
  }

protected:
  bool has_new_references_ = false;

  // ROS publishers & subscribers
private:
  rclcpp::Publisher<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_pub_;

  rclcpp::Subscription<as2_msgs::msg::TrajectoryPoint>::SharedPtr trajectory_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr thrust_command_sub_;
  rclcpp::Subscription<as2_msgs::msg::AlertEvent>::SharedPtr alert_event_sub_;

  /**
   * @brief Publishes the platform info message.
   */
  void publishPlatformInfo() {
    platform_info_msg_.header.stamp = this->now();
    platform_info_msg_.status       = state_machine_.getState();
    platform_info_pub_->publish(platform_info_msg_);
  };

  void alertEventCallback(const as2_msgs::msg::AlertEvent::SharedPtr msg);

  // ROS Services & srv callbacks
private:
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_platform_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offboard_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr platform_takeoff_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr platform_land_srv_;
  rclcpp::Service<as2_msgs::srv::ListControlModes>::SharedPtr list_control_modes_srv_;

  /**
   * @brief Set Aircraft Control Mode Service Callback
   *
   * @param request
   * @param response
   */
  void setPlatformControlModeSrvCall(
      const std::shared_ptr<as2_msgs::srv::SetControlMode::Request> request,
      std::shared_ptr<as2_msgs::srv::SetControlMode::Response> response);

  /**
   * @brief Set Aircraft Arming State Service Callback
   *
   * @param request
   * @param response
   */
  void setArmingStateSrvCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Set Aircraft Offboard Mode Service Callback
   *
   * @param request
   * @param response
   */
  void setOffboardModeSrvCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Takeoff Service Callback
   *
   * @param request
   * @param response
   */
  void platformTakeoffSrvCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   *
   * @brief Land Service Callback
   *
   * @param request
   * @param response
   */
  void platformLandSrvCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   *
   * @brief get list of available Control Modes Service Callback
   *
   * @param request
   * @param response
   */
  void listControlModesSrvCall(
      const std::shared_ptr<as2_msgs::srv::ListControlModes::Request> request,
      std::shared_ptr<as2_msgs::srv::ListControlModes::Response> response);

};  // class AerialPlatform
};  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
