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
 *  \file       as2_platform_dji_psdk.hpp
 *  \brief      DJI PSDK platform definition
 *  \authors    Rafael Pérez Seguí
 *              Santiago Tapia Fernandez
 ********************************************************************************/

#ifndef AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_
#define AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/msg/gimbal_control.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "psdk_interfaces/msg/position_fused.hpp"
#include "psdk_interfaces/msg/gimbal_rotation.hpp"
#include "psdk_interfaces/srv/camera_setup_streaming.hpp"
#include "psdk_interfaces/srv/gimbal_reset.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#define GIMBAL_COMMAND_TIME 0.5

namespace as2_platform_dji_psdk
{

class DJIMatricePSDKPlatform : public as2::AerialPlatform
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit DJIMatricePSDKPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~DJIMatricePSDKPlatform() = default;

  void configureSensors() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  void ownStopPlatform() override;
  void ownKillSwitch() override;
  bool ownTakeoff() override;
  bool ownLand() override;

private:
  // Internal variables
  bool ctl_authority_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> sensor_odom_ptr_;
  geometry_msgs::msg::Quaternion current_attitude_;
  geometry_msgs::msg::Vector3 current_lineal_velocity_;
  geometry_msgs::msg::Vector3 current_angular_velocity_;

  // Gimbal
  as2::tf::TfHandler tf_handler_;
  std::chrono::nanoseconds tf_timeout_;
  bool enable_gimbal_;
  std::string gimbal_base_frame_id_;
  psdk_interfaces::msg::GimbalRotation gimbal_command_msg_;
  rclcpp::Time last_gimbal_command_time_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_command_pub_;
  rclcpp::Publisher<psdk_interfaces::msg::GimbalRotation>::SharedPtr gimbal_rotation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr gimbal_attitude_pub_;

  // Subscribers
  rclcpp::Subscription<psdk_interfaces::msg::PositionFused>::SharedPtr position_fused_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_control_sub_;

  // Services clients
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr turn_on_motors_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr turn_off_motors_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr set_local_position_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr obtain_ctrl_authority_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr release_ctrl_authority_srv_;
  as2::SynchronousServiceClient<psdk_interfaces::srv::CameraSetupStreaming>::SharedPtr
    camera_setup_streaming_srv_;
  as2::SynchronousServiceClient<psdk_interfaces::srv::GimbalReset>::SharedPtr gimbal_reset_srv_;

  // Subscribers callbacks
  void positionFusedCallback(const psdk_interfaces::msg::PositionFused::SharedPtr msg);
  void attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void gimbalControlCallback(const as2_msgs::msg::GimbalControl::SharedPtr msg);

  // Utility functions
  bool setControlAuthority(bool state);
};  // class DJIMatricePSDKPlatform

}  // namespace as2_platform_dji_psdk

#endif  // AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_
