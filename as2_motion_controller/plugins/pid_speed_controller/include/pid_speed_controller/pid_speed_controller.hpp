/*!*******************************************************************************************
 *  \file       pid_speed_controller_plugin.hpp
 *  \brief      Speed PID controller plugin for the Aerostack framework.
 *  \authors    Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
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

#ifndef __SP_PLUGIN_H__
#define __SP_PLUGIN_H__

#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "as2_core/utils/control_mode_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_motion_controller/controller_base.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/msg/trajectory_point.hpp"
#include "pid_controller/pid.hpp"
#include "pid_controller/pid_1d.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace pid_speed_controller {

struct UAV_state {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw      = Eigen::Vector3d::Zero();
};

struct UAV_command {
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw_speed         = 0.0;
};

struct Control_flags {
  bool state_received                              = false;
  bool ref_received                                = false;
  bool plugin_parameters_read                      = false;
  bool position_controller_parameters_read         = false;
  bool velocity_controller_parameters_read         = false;
  bool speed_in_a_plane_controller_parameters_read = false;
  bool trajectory_controller_parameters_read       = false;
  bool yaw_controller_parameters_read              = false;
};

class Plugin : public as2_motion_controller_plugin_base::ControllerBase {
  using PID    = pid_controller::PID<double>;
  using PID_1D = pid_1d_controller::PID<double>;

public:
  Plugin(){};
  ~Plugin(){};

public:
  void ownInitialize() override;
  void updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                   const geometry_msgs::msg::TwistStamped &twist_msg) override;

  void updateReference(const geometry_msgs::msg::PoseStamped &ref) override;
  void updateReference(const geometry_msgs::msg::TwistStamped &ref) override;
  void updateReference(const as2_msgs::msg::TrajectoryPoint &ref) override;

  bool setMode(const as2_msgs::msg::ControlMode &mode_in,
               const as2_msgs::msg::ControlMode &mode_out) override;

  std::string getDesiredPoseFrameId();
  std::string getDesiredTwistFrameId();

  bool computeOutput(double dt,
                     geometry_msgs::msg::PoseStamped &pose,
                     geometry_msgs::msg::TwistStamped &twist,
                     as2_msgs::msg::Thrust &thrust) override;

  bool updateParams(const std::vector<rclcpp::Parameter> &_params_list) override;
  void reset() override;

private:
  as2_msgs::msg::ControlMode control_mode_in_;
  as2_msgs::msg::ControlMode control_mode_out_;

  Control_flags flags_;

  PID_1D pid_yaw_handler_;
  PID pid_3D_position_handler_;
  PID pid_3D_velocity_handler_;
  PID_1D pid_1D_speed_in_a_plane_handler_;
  PID pid_3D_speed_in_a_plane_handler_;
  PID pid_3D_trajectory_handler_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;

  std::vector<std::string> plugin_parameters_list_ = {"proportional_limitation", "use_bypass"};

  const std::vector<std::string> position_control_parameters_list_ = {
      "position_control.reset_integral", "position_control.antiwindup_cte",
      "position_control.alpha",          "position_control.kp.x",
      "position_control.kp.y",           "position_control.kp.z",
      "position_control.ki.x",           "position_control.ki.y",
      "position_control.ki.z",           "position_control.kd.x",
      "position_control.kd.y",           "position_control.kd.z"};

  const std::vector<std::string> velocity_control_parameters_list_ = {
      "speed_control.reset_integral", "speed_control.antiwindup_cte", "speed_control.alpha",
      "speed_control.kp.x",           "speed_control.kp.y",           "speed_control.kp.z",
      "speed_control.ki.x",           "speed_control.ki.y",           "speed_control.ki.z",
      "speed_control.kd.x",           "speed_control.kd.y",           "speed_control.kd.z"};

  const std::vector<std::string> speed_in_a_plane_control_parameters_list_ = {
      "speed_in_a_plane_control.reset_integral", "speed_in_a_plane_control.antiwindup_cte",
      "speed_in_a_plane_control.alpha",          "speed_in_a_plane_control.height.kp",
      "speed_in_a_plane_control.height.ki",      "speed_in_a_plane_control.height.kd",
      "speed_in_a_plane_control.speed.kp.x",     "speed_in_a_plane_control.speed.kp.y",
      "speed_in_a_plane_control.speed.ki.x",     "speed_in_a_plane_control.speed.ki.y",
      "speed_in_a_plane_control.speed.kd.x",     "speed_in_a_plane_control.speed.kd.y"};

  const std::vector<std::string> trajectory_control_parameters_list_ = {
      "trajectory_control.reset_integral", "trajectory_control.antiwindup_cte",
      "trajectory_control.alpha",          "trajectory_control.kp.x",
      "trajectory_control.kp.y",           "trajectory_control.kp.z",
      "trajectory_control.ki.x",           "trajectory_control.ki.y",
      "trajectory_control.ki.z",           "trajectory_control.kd.x",
      "trajectory_control.kd.y",           "trajectory_control.kd.z"};

  const std::vector<std::string> yaw_control_parameters_list_ = {"yaw_control.reset_integral",
                                                                 "yaw_control.antiwindup_cte",
                                                                 "yaw_control.alpha",
                                                                 "yaw_control.kp",
                                                                 "yaw_control.ki",
                                                                 "yaw_control.kd"};

  std::vector<std::string> plugin_parameters_to_read_{plugin_parameters_list_};
  std::vector<std::string> position_control_parameters_to_read_{position_control_parameters_list_};
  std::vector<std::string> velocity_control_parameters_to_read_{velocity_control_parameters_list_};
  std::vector<std::string> speed_in_a_plane_control_parameters_to_read_{
      speed_in_a_plane_control_parameters_list_};
  std::vector<std::string> trajectory_control_parameters_to_read_{
      trajectory_control_parameters_list_};
  std::vector<std::string> yaw_control_parameters_to_read_{yaw_control_parameters_list_};

  UAV_state uav_state_;
  UAV_state control_ref_;
  UAV_command control_command_;

  bool hover_flag_ = false;

  Eigen::Vector3d speed_limits_;
  double yaw_speed_limit_;

  bool use_bypass_              = true;
  bool proportional_limitation_ = false;

  std::string enu_frame_id_ = "odom";
  std::string flu_frame_id_ = "base_link";

  std::string input_pose_frame_id_  = enu_frame_id_;
  std::string input_twist_frame_id_ = enu_frame_id_;

  std::string output_twist_frame_id_ = enu_frame_id_;

private:
  void checkParamList(const std::string &param,
                      std::vector<std::string> &_params_list,
                      bool &_all_params_read);

  void updateControllerParameter(PID_1D &_pid_handler,
                                 const std::string &_parameter_name,
                                 const rclcpp::Parameter &_param);

  void updateController3DParameter(PID &_pid_handler,
                                   const std::string &_parameter_name,
                                   const rclcpp::Parameter &_param);

  void updateSpeedInAPlaneParameter(PID_1D &_pid_1d_handler,
                                    PID &_pid_3d_handler,
                                    const std::string &_parameter_name,
                                    const rclcpp::Parameter &_param);

  void resetState();
  void resetReferences();
  void resetCommands();

  bool getOutput(geometry_msgs::msg::TwistStamped &twist_msg);
};
};  // namespace pid_speed_controller

#endif
