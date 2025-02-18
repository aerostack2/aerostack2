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
 *  \file       controller_handler.hpp
 *  \brief      Controller handler class definition
 *  \authors    Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_
#define AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_

#include <rcl/time.h>
#include <tf2/time.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <memory>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_setpoints.hpp"
#include "as2_msgs/srv/list_control_modes.hpp"
#include "as2_msgs/srv/set_control_mode.hpp"

#include "controller_base.hpp"

namespace controller_handler
{

#define MATCH_ALL 0b11111111
#define MATCH_MODE_AND_FRAME 0b11110011
#define MATCH_MODE 0b11110000
#define MATCH_MODE_AND_YAW 0b11111100

#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

using namespace std::chrono_literals; // NOLINT

class ControllerHandler
{
public:
  ControllerHandler(
    std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller,
    as2::Node * node);

  virtual ~ControllerHandler() {}

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void getMode(as2_msgs::msg::ControlMode & mode_in, as2_msgs::msg::ControlMode & mode_out);
  void setInputControlModesAvailables(const std::vector<uint8_t> & available_modes);
  void setOutputControlModesAvailables(const std::vector<uint8_t> & available_modes);

  void reset();

protected:
  as2::Node * node_ptr_;

private:
  // Control modes availables
  std::vector<uint8_t> controller_available_modes_in_;
  std::vector<uint8_t> controller_available_modes_out_;
  std::vector<uint8_t> platform_available_modes_in_;

  // Frame ids
  std::string enu_frame_id_ = "odom";
  std::string flu_frame_id_ = "base_link";
  std::string input_pose_frame_id_ = "odom";
  std::string input_twist_frame_id_ = "odom";
  std::string output_pose_frame_id_ = "odom";
  std::string output_twist_frame_id_ = "odom";

  // TF handler
  as2::tf::TfHandler tf_handler_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectorySetpoints>::SharedPtr ref_traj_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr ref_thrust_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;

  // Publishers
  rclcpp::Publisher<as2_msgs::msg::TrajectorySetpoints>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  // Services servers
  rclcpp::Service<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;

  // Services clients
  as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;
  as2::SynchronousServiceClient<as2_msgs::srv::ListControlModes>::SharedPtr
    list_control_modes_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Internal variables
  bool control_mode_established_ = false;
  bool motion_reference_adquired_ = false;
  bool state_adquired_ = false;
  bool use_bypass_ = false;
  bool bypass_controller_ = false;

  uint8_t prefered_output_mode_ = 0b00000000;  // by default, no output mode is prefered

  rclcpp::Time last_time_;

  as2_msgs::msg::PlatformInfo platform_info_;
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  geometry_msgs::msg::PoseStamped state_pose_;
  geometry_msgs::msg::TwistStamped state_twist_;
  geometry_msgs::msg::PoseStamped ref_pose_;
  geometry_msgs::msg::TwistStamped ref_twist_;
  as2_msgs::msg::TrajectorySetpoints ref_traj_;
  as2_msgs::msg::Thrust ref_thrust_;
  geometry_msgs::msg::PoseStamped command_pose_;
  geometry_msgs::msg::TwistStamped command_twist_;
  as2_msgs::msg::Thrust command_thrust_;

  // Controller plugin
  std::shared_ptr<as2_motion_controller_plugin_base::ControllerBase> controller_ptr_;

private:
  // Subscribers callbacks
  void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void refPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void refTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void refTrajCallback(const as2_msgs::msg::TrajectorySetpoints::SharedPtr msg);
  void refThrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg);
  void platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr msg);

  // Services servers callbacks
  void setControlModeSrvCall(
    const as2_msgs::srv::SetControlMode::Request::SharedPtr request,
    as2_msgs::srv::SetControlMode::Response::SharedPtr response);
  bool listPlatformAvailableControlModes();

  // Timer callbacks
  void controlTimerCallback();

  // Internal methods
  std::string getFrameIdByReferenceFrame(uint8_t reference_frame);

  bool findSuitableOutputControlModeForPlatformInputMode(
    uint8_t & output_mode,
    const uint8_t input_mode);
  bool checkSuitabilityInputMode(uint8_t & input_mode, const uint8_t output_mode);
  bool setPlatformControlMode(const as2_msgs::msg::ControlMode & mode);

  bool findSuitableControlModes(uint8_t & input_mode, uint8_t & output_mode);
  bool trySetPlatformHover();
  bool tryToBypassController(const uint8_t input_mode, uint8_t & output_mode);

  void sendCommand();
  void publishCommand();
};  //  class ControllerBase

}  //  namespace controller_handler

#endif  // AS2_MOTION_CONTROLLER__CONTROLLER_HANDLER_HPP_
