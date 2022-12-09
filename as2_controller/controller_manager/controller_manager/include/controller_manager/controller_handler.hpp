/*!*******************************************************************************************
 *  \file       controller_handler.hpp
 *  \brief      Controller handler class definition
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

#ifndef CONTROLLER_HANDLER_HPP
#define CONTROLLER_HANDLER_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <rcl/time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"
#include "controller_plugin_base/controller_base.hpp"

#include "as2_core/utils/tf_utils.hpp"

#define MATCH_ALL 0b11111111
#define MATCH_MODE_AND_FRAME 0b11110011
#define MATCH_MODE 0b11110000
#define MATCH_MODE_AND_YAW 0b11111100

#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

class ControllerHandler {
private:
  std::vector<uint8_t> controller_available_modes_in_;
  std::vector<uint8_t> controller_available_modes_out_;
  std::vector<uint8_t> platform_available_modes_in_;

  std::string enu_frame_id_ = "odom";
  std::string flu_frame_id_ = "base_link";

  std::string input_pose_frame_id_  = "odom";
  std::string input_twist_frame_id_ = "odom";

  std::string output_pose_frame_id_  = "odom";
  std::string output_twist_frame_id_ = "odom";

  as2::tf::TfHandler tf_handler_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectoryPoint>::SharedPtr ref_traj_sub_;

  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  as2_msgs::msg::PlatformInfo platform_info_;

  as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;
  as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>::SharedPtr
      list_control_modes_client_;

  bool control_mode_established_  = false;
  bool motion_reference_adquired_ = false;
  bool state_adquired_            = false;

  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  uint8_t prefered_output_mode_ = 0b00000000;  // by default, no output mode is prefered

  std::shared_ptr<controller_plugin_base::ControllerBase> controller_ptr_;

public:
  ControllerHandler(std::shared_ptr<controller_plugin_base::ControllerBase> controller,
                    as2::Node* node);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  bool use_bypass_        = false;
  bool bypass_controller_ = false;

  void getMode(as2_msgs::msg::ControlMode& mode_in, as2_msgs::msg::ControlMode& mode_out) {
    mode_in  = control_mode_in_;
    mode_out = control_mode_out_;
    return;
  };

  void setInputControlModesAvailables(const std::vector<uint8_t>& available_modes) {
    controller_available_modes_in_ = available_modes;
    // sort modes in ascending order
    std::sort(controller_available_modes_in_.begin(), controller_available_modes_in_.end());
  };

  void setOutputControlModesAvailables(const std::vector<uint8_t>& available_modes) {
    controller_available_modes_out_ = available_modes;
    // sort modes in ascending order
    std::sort(controller_available_modes_out_.begin(), controller_available_modes_out_.end());
  };

  virtual ~ControllerHandler(){};

protected:
  as2::Node* node_ptr_;

private:
  rclcpp::Time last_time_;

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void ref_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void ref_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void ref_traj_callback(const as2_msgs::msg::TrajectoryPoint::SharedPtr msg);
  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  geometry_msgs::msg::PoseStamped state_pose_;
  geometry_msgs::msg::TwistStamped state_twist_;
  geometry_msgs::msg::PoseStamped ref_pose_;
  geometry_msgs::msg::TwistStamped ref_twist_;
  as2_msgs::msg::TrajectoryPoint ref_traj_;
  geometry_msgs::msg::PoseStamped command_pose_;
  geometry_msgs::msg::TwistStamped command_twist_;
  as2_msgs::msg::Thrust command_thrust_;

  void control_timer_callback();
  void setControlModeSrvCall(const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
                             as2_msgs::srv::SetControlMode::Response::SharedPtr response);
  bool listPlatformAvailableControlModes();

  std::string getFrameIdByReferenceFrame(uint8_t reference_frame);

  bool trySetPlatformHover();
  bool tryToBypassController(const uint8_t input_mode, uint8_t& output_mode);
  bool findSuitableControlModes(uint8_t& input_mode, uint8_t& output_mode);
  bool findSuitableOutputControlModeForPlatformInputMode(uint8_t& output_mode,
                                                         const uint8_t input_mode);
  bool checkSuitabilityInputMode(uint8_t& input_mode, const uint8_t output_mode);
  void sendCommand();
  bool setPlatformControlMode(const as2_msgs::msg::ControlMode& mode);

  void reset();

  void publishCommand();

};  //  ControllerBase

#endif
