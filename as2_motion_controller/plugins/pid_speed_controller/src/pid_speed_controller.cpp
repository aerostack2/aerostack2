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
 *  @file       pid_speed_controller_plugin.cpp
 *  @brief      Speed PID controller plugin for the Aerostack framework.
 *  @authors    Rafael Perez-Segui
 *              Miguel Fernández Cortizas
 ********************************************************************************************/

#include "pid_speed_controller.hpp"

namespace pid_speed_controller
{

void Plugin::ownInitialize()
{
  speed_limits_ = Eigen::Vector3d::Zero();

  // Output twist frame defaults to the configured ENU pose frame until
  // setMode() picks something else for body-frame velocity modes.
  output_twist_frame_id_ = getDesiredPoseFrameId();

  // Mutable copies of the optional-group parameter tail lists. The essential
  // groups (plugin / position / yaw) are tracked by the base via
  // pending_essentials_.
  velocity_control_parameters_to_read_ = velocity_control_parameters_tail_;
  speed_in_a_plane_control_parameters_to_read_ = speed_in_a_plane_control_parameters_tail_;
  trajectory_control_parameters_to_read_ = trajectory_control_parameters_tail_;

  reset();
}

std::vector<std::string> Plugin::getEssentialParameters() const
{
  // Only the always-active controllers (plugin globals, position and yaw) are
  // returned as essential. Mode-specific groups (trajectory / velocity /
  // speed_in_a_plane) are gated separately at setMode time using their own
  // params_read_ flags.
  std::vector<std::string> out;
  out.reserve(
    plugin_parameters_tail_.size() +
    position_control_parameters_tail_.size() +
    yaw_control_parameters_tail_.size());
  for (const auto & tail : plugin_parameters_tail_) {
    out.push_back(param(tail));
  }
  for (const auto & tail : position_control_parameters_tail_) {
    out.push_back(param(tail));
  }
  for (const auto & tail : yaw_control_parameters_tail_) {
    out.push_back(param(tail));
  }
  return out;
}

void Plugin::updateParameter(const rclcpp::Parameter & parameter)
{
  const std::string ns_prefix = getPluginParamNamespace().empty() ?
    std::string() :
    getPluginParamNamespace() + ".";
  const std::string & full_name = parameter.get_name();
  const std::string tail = ns_prefix.empty() ? full_name :
    full_name.substr(ns_prefix.size());

  RCLCPP_DEBUG(getNodePtr()->get_logger(), "Updating parameter %s", full_name.c_str());

  if (tail == "proportional_limitation") {
    proportional_limitation_ = parameter.get_value<bool>();
  } else if (tail == "use_bypass") {
    use_bypass_ = parameter.get_value<bool>();
  } else {
    const auto dot = tail.find('.');
    if (dot == std::string::npos) {return;}
    const std::string controller = tail.substr(0, dot);
    const std::string param_subname = tail.substr(dot + 1);
    if (controller == "position_control") {
      updateController3DParameter(pid_3D_position_handler_, param_subname, parameter);
    } else if (controller == "speed_control") {
      updateController3DParameter(pid_3D_velocity_handler_, param_subname, parameter);
      if (!params_read_.velocity) {
        checkParamList(tail, velocity_control_parameters_to_read_, params_read_.velocity);
      }
    } else if (controller == "speed_in_a_plane_control") {
      updateSpeedInAPlaneParameter(
        pid_1D_speed_in_a_plane_handler_,
        pid_3D_speed_in_a_plane_handler_, param_subname, parameter);
      if (!params_read_.speed_in_a_plane) {
        checkParamList(
          tail, speed_in_a_plane_control_parameters_to_read_, params_read_.speed_in_a_plane);
      }
    } else if (controller == "trajectory_control") {
      updateController3DParameter(pid_3D_trajectory_handler_, param_subname, parameter);
      if (!params_read_.trajectory) {
        checkParamList(tail, trajectory_control_parameters_to_read_, params_read_.trajectory);
      }
    } else if (controller == "yaw_control") {
      updateControllerParameter(pid_yaw_handler_, param_subname, parameter);
    }
  }
}

void Plugin::reset()
{
  ControllerBase::reset();
  resetReferences();
  resetState();
  resetCommands();
  pid_yaw_handler_.reset_controller();
  pid_3D_position_handler_.reset_controller();
  pid_3D_velocity_handler_.reset_controller();
  pid_3D_trajectory_handler_.reset_controller();
}

bool Plugin::setMode(
  const as2_msgs::msg::ControlMode & in_mode,
  const as2_msgs::msg::ControlMode & out_mode)
{
  if (!essentialParamsReady()) {
    RCLCPP_WARN(
      getNodePtr()->get_logger(),
      "Essential parameters not read yet, can not set mode");
    return false;
  }

  if (in_mode.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY &&
    !params_read_.trajectory)
  {
    RCLCPP_WARN(
      getNodePtr()->get_logger(),
      "Trajectory controller parameters not read yet, can not set mode to TRAJECTORY");
    return false;
  } else if ((in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED ||  // NOLINT
    in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
    (!params_read_.velocity && !use_bypass_))
  {
    RCLCPP_WARN(
      getNodePtr()->get_logger(),
      "Velocity controller parameters not read yet and bypass is not used, can not set "
      "mode to SPEED or SPEED_IN_A_PLANE");
    return false;
  }

  if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    control_mode_in_.control_mode = in_mode.control_mode;
    control_mode_in_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  } else {
    control_mode_in_ = in_mode;
  }

  control_mode_out_ = out_mode;

  // The desired pose frame is always the configured ENU one (the parameter
  // `desired_pose_frame` already drives it via the base). Only the twist
  // frame depends on the active control mode.
  const std::string enu_frame = getDesiredPoseFrameId();
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY)
  {
    setDesiredTwistFrameId(enu_frame);
    output_twist_frame_id_ = enu_frame;
  } else if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||  // NOLINT
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE)
  {
    switch (control_mode_out_.reference_frame) {
      case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
        setDesiredTwistFrameId(getBaseLinkFrameId());
        output_twist_frame_id_ = getBaseLinkFrameId();
        break;
      case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      default:
        setDesiredTwistFrameId(enu_frame);
        output_twist_frame_id_ = enu_frame;
        break;
    }
  }

  return true;
}

void Plugin::onUpdateState(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const geometry_msgs::msg::TwistStamped & twist_msg)
{
  uav_state_.position =
    Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  uav_state_.velocity =
    Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);
  uav_state_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
}

void Plugin::onUpdateReference(const geometry_msgs::msg::PoseStamped & pose_msg)
{
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE)
  {
    control_ref_.position = Eigen::Vector3d(
      pose_msg.pose.position.x, pose_msg.pose.position.y,
      pose_msg.pose.position.z);
  }

  if ((control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
    control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE)
  {
    control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
  }
}

void Plugin::onUpdateReference(const geometry_msgs::msg::TwistStamped & twist_msg)
{
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    speed_limits_ = Eigen::Vector3d(
      twist_msg.twist.linear.x, twist_msg.twist.linear.y,
      twist_msg.twist.linear.z);
    pid_3D_position_handler_.set_output_saturation(
      speed_limits_, -speed_limits_,
      proportional_limitation_);
    pid_3D_velocity_handler_.set_output_saturation(
      speed_limits_, -speed_limits_,
      proportional_limitation_);
    pid_3D_trajectory_handler_.set_output_saturation(
      speed_limits_, -speed_limits_,
      proportional_limitation_);
    return;
  }

  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED &&
    control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE)
  {
    return;
  }

  control_ref_.velocity =
    Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

  if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    control_ref_.yaw.y() = twist_msg.twist.angular.z;
  }
}

void Plugin::onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & traj_setpoints_msg)
{
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  as2_msgs::msg::TrajectoryPoint traj_msg = traj_setpoints_msg.setpoints[0];

  control_ref_.position =
    Eigen::Vector3d(traj_msg.position.x, traj_msg.position.y, traj_msg.position.z);
  control_ref_.velocity = Eigen::Vector3d(traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z);
  control_ref_.yaw.x() = traj_msg.yaw_angle;
}

void Plugin::latchHoverReference(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::TwistStamped & /*twist*/)
{
  // PID mode in HOVER expects control_ref_ pre-populated since
  // onUpdateReference() ignores incoming messages while in HOVER.
  control_ref_.position = Eigen::Vector3d(
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  control_ref_.velocity = Eigen::Vector3d::Zero();
  control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose.pose.orientation);
}

bool Plugin::computeOutput(
  double dt,
  geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & twist,
  as2_msgs::msg::Thrust & thrust)
{
  // The handler already gates the call by state_received and
  // motion_reference_acquired, so we don't re-check those here.
  (void)pose;
  (void)thrust;

  resetCommands();

  switch (control_mode_in_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
    case as2_msgs::msg::ControlMode::POSITION: {
        Eigen::Vector3d position_error =
          pid_3D_position_handler_.get_error(uav_state_.position, control_ref_.position);
        control_command_.velocity = pid_3D_position_handler_.compute_control(dt, position_error);
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED: {
        if (use_bypass_) {
          control_command_.velocity = control_ref_.velocity;
        } else {
          Eigen::Vector3d velocity_error =
            pid_3D_velocity_handler_.get_error(uav_state_.velocity, control_ref_.velocity);
          control_command_.velocity = pid_3D_velocity_handler_.compute_control(dt, velocity_error);
        }
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
        if (use_bypass_) {
          control_command_.velocity = control_ref_.velocity;
        } else {
          Eigen::Vector3d velocity_error =
            pid_3D_speed_in_a_plane_handler_.get_error(uav_state_.velocity, control_ref_.velocity);
          control_command_.velocity =
            pid_3D_speed_in_a_plane_handler_.compute_control(dt, velocity_error);
        }

        double position_error = pid_1D_speed_in_a_plane_handler_.get_error(
          uav_state_.position.z(),
          control_ref_.position.z());
        control_command_.velocity.z() =
          pid_1D_speed_in_a_plane_handler_.compute_control(dt, position_error);

        break;
      }
    case as2_msgs::msg::ControlMode::TRAJECTORY: {
        Eigen::Vector3d position_error =
          pid_3D_trajectory_handler_.get_error(uav_state_.position, control_ref_.position);
        Eigen::Vector3d velocity_error =
          pid_3D_trajectory_handler_.get_error(uav_state_.velocity, control_ref_.velocity);
        control_command_.velocity =
          pid_3D_trajectory_handler_.compute_control(dt, position_error, velocity_error);
        break;
      }
    default: {
        auto & clk = *getNodePtr()->get_clock();
        RCLCPP_ERROR_THROTTLE(getNodePtr()->get_logger(), clk, 5000, "Unknown control mode");
        return false;
      }
  }

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE: {
        double yaw_error = as2::frame::angleMinError(control_ref_.yaw.x(), uav_state_.yaw.x());
        control_command_.yaw_speed = pid_yaw_handler_.compute_control(dt, yaw_error);
        break;
      }
    case as2_msgs::msg::ControlMode::YAW_SPEED: {
        control_command_.yaw_speed = control_ref_.yaw.y();
        break;
      }
    default: {
        auto & clk = *getNodePtr()->get_clock();
        RCLCPP_ERROR_THROTTLE(getNodePtr()->get_logger(), clk, 5000, "Unknown yaw mode");
        return false;
      }
  }

  return getOutput(twist);
}

// ===== Internal helpers =====================================================

void Plugin::checkParamList(
  const std::string & param_name,
  std::vector<std::string> & params_list,
  bool & all_params_read)
{
  auto it = std::find(params_list.begin(), params_list.end(), param_name);
  if (it != params_list.end()) {
    params_list.erase(it);
  }
  if (params_list.empty()) {
    all_params_read = true;
  }
}

void Plugin::updateControllerParameter(
  PID_1D & _pid_handler,
  const std::string & _parameter_name,
  const rclcpp::Parameter & _param)
{
  if (_parameter_name == "reset_integral") {
    _pid_handler.set_reset_integral_saturation_flag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_handler.set_anti_windup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_handler.set_alpha(_param.get_value<double>());
  } else if (_parameter_name == "kp") {
    double kp, ki, kd;
    _pid_handler.get_gains(kp, ki, kd);
    _pid_handler.set_gains(_param.get_value<double>(), ki, kd);
  } else if (_parameter_name == "ki") {
    double kp, ki, kd;
    _pid_handler.get_gains(kp, ki, kd);
    _pid_handler.set_gains(kp, _param.get_value<double>(), kd);
  } else if (_parameter_name == "kd") {
    double kp, ki, kd;
    _pid_handler.get_gains(kp, ki, kd);
    _pid_handler.set_gains(kp, ki, _param.get_value<double>());
  }
}

void Plugin::updateController3DParameter(
  PID & _pid_handler,
  const std::string & _parameter_name,
  const rclcpp::Parameter & _param)
{
  if (_parameter_name == "reset_integral") {
    _pid_handler.set_reset_integral_saturation_flag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    Eigen::Vector3d anti_windup = Eigen::Vector3d::Constant(_param.get_value<double>());
    _pid_handler.set_anti_windup(anti_windup);
  } else if (_parameter_name == "alpha") {
    Eigen::Vector3d alpha = Eigen::Vector3d::Constant(_param.get_value<double>());
    _pid_handler.set_alpha(alpha);
  } else if (_parameter_name == "kp.x") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kp();
    current_gains.x() = _param.get_value<double>();
    _pid_handler.set_gains_kp(current_gains);
  } else if (_parameter_name == "kp.y") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kp();
    current_gains.y() = _param.get_value<double>();
    _pid_handler.set_gains_kp(current_gains);
  } else if (_parameter_name == "kp.z") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kp();
    current_gains.z() = _param.get_value<double>();
    _pid_handler.set_gains_kp(current_gains);
  } else if (_parameter_name == "ki.x") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_ki();
    current_gains.x() = _param.get_value<double>();
    _pid_handler.set_gains_ki(current_gains);
  } else if (_parameter_name == "ki.y") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_ki();
    current_gains.y() = _param.get_value<double>();
    _pid_handler.set_gains_ki(current_gains);
  } else if (_parameter_name == "ki.z") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_ki();
    current_gains.z() = _param.get_value<double>();
    _pid_handler.set_gains_ki(current_gains);
  } else if (_parameter_name == "kd.x") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kd();
    current_gains.x() = _param.get_value<double>();
    _pid_handler.set_gains_kd(current_gains);
  } else if (_parameter_name == "kd.y") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kd();
    current_gains.y() = _param.get_value<double>();
    _pid_handler.set_gains_kd(current_gains);
  } else if (_parameter_name == "kd.z") {
    Eigen::Vector3d current_gains = _pid_handler.get_gains_kd();
    current_gains.z() = _param.get_value<double>();
    _pid_handler.set_gains_kd(current_gains);
  }
}

void Plugin::updateSpeedInAPlaneParameter(
  PID_1D & _pid_1d_handler,
  PID & _pid_3d_handler,
  const std::string & _parameter_name,
  const rclcpp::Parameter & _param)
{
  if (_parameter_name == "reset_integral") {
    _pid_1d_handler.set_reset_integral_saturation_flag(_param.get_value<bool>());
    _pid_3d_handler.set_reset_integral_saturation_flag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_1d_handler.set_anti_windup(_param.get_value<double>());
    Eigen::Vector3d anti_windup = Eigen::Vector3d::Constant(_param.get_value<double>());
    _pid_3d_handler.set_alpha(anti_windup);
  } else if (_parameter_name == "alpha") {
    _pid_1d_handler.set_alpha(_param.get_value<double>());
    Eigen::Vector3d alpha = Eigen::Vector3d::Constant(_param.get_value<double>());
    _pid_3d_handler.set_alpha(alpha);
  } else if (_parameter_name == "height.kp") {
    double kp, ki, kd;
    _pid_1d_handler.get_gains(kp, ki, kd);
    _pid_1d_handler.set_gains(_param.get_value<double>(), ki, kd);
  } else if (_parameter_name == "height.ki") {
    double kp, ki, kd;
    _pid_1d_handler.get_gains(kp, ki, kd);
    _pid_1d_handler.set_gains(kp, _param.get_value<double>(), kd);
  } else if (_parameter_name == "height.kd") {
    double kp, ki, kd;
    _pid_1d_handler.get_gains(kp, ki, kd);
    _pid_1d_handler.set_gains(kp, ki, _param.get_value<double>());
  } else if (_parameter_name == "speed.kp.x") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_kp();
    current_gains.x() = _param.get_value<double>();
    _pid_3d_handler.set_gains_kp(current_gains);
  } else if (_parameter_name == "speed.kp.y") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_kp();
    current_gains.y() = _param.get_value<double>();
    _pid_3d_handler.set_gains_kp(current_gains);
  } else if (_parameter_name == "speed.ki.x") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_ki();
    current_gains.x() = _param.get_value<double>();
    _pid_3d_handler.set_gains_ki(current_gains);
  } else if (_parameter_name == "speed.ki.y") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_ki();
    current_gains.y() = _param.get_value<double>();
    _pid_3d_handler.set_gains_ki(current_gains);
  } else if (_parameter_name == "speed.kd.x") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_kd();
    current_gains.x() = _param.get_value<double>();
    _pid_3d_handler.set_gains_kd(current_gains);
  } else if (_parameter_name == "speed.kd.y") {
    Eigen::Vector3d current_gains = _pid_3d_handler.get_gains_kd();
    current_gains.y() = _param.get_value<double>();
    _pid_3d_handler.set_gains_kd(current_gains);
  }
}

void Plugin::resetState() {uav_state_ = UAV_state();}

void Plugin::resetReferences()
{
  control_ref_.position = uav_state_.position;
  control_ref_.velocity = Eigen::Vector3d::Zero();
  control_ref_.yaw = uav_state_.yaw;
}

void Plugin::resetCommands()
{
  control_command_.velocity = Eigen::Vector3d::Zero();
  control_command_.yaw_speed = 0.0;
}

bool Plugin::getOutput(geometry_msgs::msg::TwistStamped & _twist_msg)
{
  _twist_msg.header.frame_id = output_twist_frame_id_;

  _twist_msg.twist.linear.x = control_command_.velocity.x();
  _twist_msg.twist.linear.y = control_command_.velocity.y();
  _twist_msg.twist.linear.z = control_command_.velocity.z();

  _twist_msg.twist.angular.x = 0;
  _twist_msg.twist.angular.y = 0;
  _twist_msg.twist.angular.z = control_command_.yaw_speed;
  return true;
}

}  // namespace pid_speed_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  pid_speed_controller::Plugin,
  as2_motion_controller_plugin_base::ControllerBase)
