/*!*******************************************************************************************
 *  \file       pid_speed_controller_plugin.cpp
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

#include "pid_speed_controller.hpp"

namespace pid_speed_controller {

void Plugin::ownInitialize() {
  speed_limits_ = Eigen::Vector3d::Zero();

  pid_yaw_handler_                 = std::make_shared<pid_controller::PIDController>();
  pid_3D_position_handler_         = std::make_shared<pid_controller::PIDController3D>();
  pid_3D_velocity_handler_         = std::make_shared<pid_controller::PIDController3D>();
  pid_1D_speed_in_a_plane_handler_ = std::make_shared<pid_controller::PIDController>();
  pid_3D_speed_in_a_plane_handler_ = std::make_shared<pid_controller::PIDController3D>();
  pid_3D_trajectory_handler_       = std::make_shared<pid_controller::PIDController3D>();

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  enu_frame_id_ = as2::tf::generateTfName(node_ptr_, enu_frame_id_);
  flu_frame_id_ = as2::tf::generateTfName(node_ptr_, flu_frame_id_);

  input_pose_frame_id_  = as2::tf::generateTfName(node_ptr_, input_pose_frame_id_);
  input_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, input_twist_frame_id_);

  output_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, output_twist_frame_id_);

  reset();
  return;
};

bool Plugin::updateParams(const std::vector<std::string> &_params_list) {
  auto result = parametersCallback(node_ptr_->get_parameters(_params_list));
  return result.successful;
};

void Plugin::checkParamList(const std::string &param,
                            std::vector<std::string> &_params_list,
                            bool &_all_params_read) {
  if (find(_params_list.begin(), _params_list.end(), param) != _params_list.end()) {
    // Remove the parameter from the list of parameters to be read
    _params_list.erase(std::remove(_params_list.begin(), _params_list.end(), param),
                       _params_list.end());
  };
  if (_params_list.size() == 0) {
    _all_params_read = true;
  }
};

rcl_interfaces::msg::SetParametersResult Plugin::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "success";

  for (auto &param : parameters) {
    std::string param_name = param.get_name();

    if (param.get_name() == "proportional_limitation") {
      proportional_limitation_ = param.get_value<bool>();
      if (!flags_.plugin_parameters_read) {
        checkParamList(param_name, plugin_parameters_to_read_, flags_.plugin_parameters_read);
      }
    } else if (param.get_name() == "use_bypass") {
      use_bypass_ = param.get_value<bool>();
      if (!flags_.plugin_parameters_read) {
        checkParamList(param_name, plugin_parameters_to_read_, flags_.plugin_parameters_read);
      }
    } else {
      std::string controller    = param_name.substr(0, param_name.find("."));
      std::string param_subname = param_name.substr(param_name.find(".") + 1);
      if (controller == "position_control") {
        updateController3DParameter(pid_3D_position_handler_, param_subname, param);
        if (!flags_.position_controller_parameters_read) {
          checkParamList(param_name, position_control_parameters_to_read_,
                         flags_.position_controller_parameters_read);
        }
      } else if (controller == "speed_control") {
        updateController3DParameter(pid_3D_velocity_handler_, param_subname, param);
        if (!flags_.velocity_controller_parameters_read) {
          checkParamList(param_name, velocity_control_parameters_to_read_,
                         flags_.velocity_controller_parameters_read);
        }
      } else if (controller == "speed_in_a_plane_control") {
        updateSpeedInAPlaneParameter(pid_1D_speed_in_a_plane_handler_,
                                     pid_3D_speed_in_a_plane_handler_, param_subname, param);
        if (!flags_.speed_in_a_plane_controller_parameters_read) {
          checkParamList(param_name, speed_in_a_plane_control_parameters_to_read_,
                         flags_.speed_in_a_plane_controller_parameters_read);
        }
      } else if (controller == "trajectory_control") {
        updateController3DParameter(pid_3D_trajectory_handler_, param_subname, param);
        if (!flags_.trajectory_controller_parameters_read) {
          checkParamList(param_name, trajectory_control_parameters_to_read_,
                         flags_.trajectory_controller_parameters_read);
        }
      } else if (controller == "yaw_control") {
        updateControllerParameter(pid_yaw_handler_, param_subname, param);
        if (!flags_.yaw_controller_parameters_read) {
          checkParamList(param_name, yaw_control_parameters_to_read_,
                         flags_.yaw_controller_parameters_read);
        }
      }
    }
  }
  return result;
}

void Plugin::updateControllerParameter(
    const std::shared_ptr<pid_controller::PIDController> &_pid_handler,
    const std::string &_parameter_name,
    const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    _pid_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_handler->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_handler->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "kp") {
    _pid_handler->setGainKp(_param.get_value<double>());
  } else if (_parameter_name == "ki") {
    _pid_handler->setGainKi(_param.get_value<double>());
  } else if (_parameter_name == "kd") {
    _pid_handler->setGainKd(_param.get_value<double>());
  }
  return;
}

void Plugin::updateController3DParameter(
    const std::shared_ptr<pid_controller::PIDController3D> &_pid_handler,
    const std::string &_parameter_name,
    const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    _pid_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_handler->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_handler->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "kp.x") {
    _pid_handler->setGainKpX(_param.get_value<double>());
  } else if (_parameter_name == "kp.y") {
    _pid_handler->setGainKpY(_param.get_value<double>());
  } else if (_parameter_name == "kp.z") {
    _pid_handler->setGainKpZ(_param.get_value<double>());
  } else if (_parameter_name == "ki.x") {
    _pid_handler->setGainKiX(_param.get_value<double>());
  } else if (_parameter_name == "ki.y") {
    _pid_handler->setGainKiY(_param.get_value<double>());
  } else if (_parameter_name == "ki.z") {
    _pid_handler->setGainKiZ(_param.get_value<double>());
  } else if (_parameter_name == "kd.x") {
    _pid_handler->setGainKdX(_param.get_value<double>());
  } else if (_parameter_name == "kd.y") {
    _pid_handler->setGainKdY(_param.get_value<double>());
  } else if (_parameter_name == "kd.z") {
    _pid_handler->setGainKdZ(_param.get_value<double>());
  }
  return;
}

void Plugin::updateSpeedInAPlaneParameter(
    const std::shared_ptr<pid_controller::PIDController> &_pid_1d_handler,
    const std::shared_ptr<pid_controller::PIDController3D> &_pid_3d_handler,
    const std::string &_parameter_name,
    const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    _pid_1d_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
    _pid_3d_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_1d_handler->setAntiWindup(_param.get_value<double>());
    _pid_3d_handler->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_1d_handler->setAlpha(_param.get_value<double>());
    _pid_3d_handler->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "height.kp") {
    _pid_1d_handler->setGainKp(_param.get_value<double>());
  } else if (_parameter_name == "height.ki") {
    _pid_1d_handler->setGainKi(_param.get_value<double>());
  } else if (_parameter_name == "height.kd") {
    _pid_1d_handler->setGainKd(_param.get_value<double>());
  } else if (_parameter_name == "speed.kp.x") {
    _pid_3d_handler->setGainKpX(_param.get_value<double>());
  } else if (_parameter_name == "speed.kp.y") {
    _pid_3d_handler->setGainKpY(_param.get_value<double>());
  } else if (_parameter_name == "speed.ki.x") {
    _pid_3d_handler->setGainKiX(_param.get_value<double>());
  } else if (_parameter_name == "speed.ki.y") {
    _pid_3d_handler->setGainKiY(_param.get_value<double>());
  } else if (_parameter_name == "speed.kd.x") {
    _pid_3d_handler->setGainKdX(_param.get_value<double>());
  } else if (_parameter_name == "speed.kd.y") {
    _pid_3d_handler->setGainKdY(_param.get_value<double>());
  }
  return;
}

void Plugin::reset() {
  resetReferences();
  resetState();
  resetCommands();
  pid_yaw_handler_->resetController();
  pid_3D_position_handler_->resetController();
  pid_3D_velocity_handler_->resetController();
  pid_3D_trajectory_handler_->resetController();
  // Info: Yaw rate limit could be set if needed
  // pid_yaw_handler_->setOutputSaturation(yaw_speed_limit_);
}

void Plugin::resetState() {
  uav_state_ = UAV_state();
  return;
}

void Plugin::resetReferences() {
  control_ref_.position = uav_state_.position;
  control_ref_.velocity = Eigen::Vector3d::Zero();
  control_ref_.yaw      = uav_state_.yaw;
  return;
}

void Plugin::resetCommands() {
  control_command_.velocity  = Eigen::Vector3d::Zero();
  control_command_.yaw_speed = 0.0;
  return;
}

void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                         const geometry_msgs::msg::TwistStamped &twist_msg) {
  if (pose_msg.header.frame_id != input_pose_frame_id_ &&
      twist_msg.header.frame_id != input_twist_frame_id_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Pose and Twist frame_id are not desired ones");
    RCLCPP_ERROR(node_ptr_->get_logger(), "Recived: %s, %s", pose_msg.header.frame_id.c_str(),
                 twist_msg.header.frame_id.c_str());
    RCLCPP_ERROR(node_ptr_->get_logger(), "Desired: %s, %s", input_pose_frame_id_.c_str(),
                 input_twist_frame_id_.c_str());
    return;
  }

  uav_state_.position =
      Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  uav_state_.velocity =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);
  uav_state_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);

  if (hover_flag_) {
    resetReferences();
    flags_.ref_received = true;
    hover_flag_         = false;
  }

  flags_.state_received = true;
  return;
};

void Plugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg) {
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
      control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
    control_ref_.position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                                            pose_msg.pose.position.z);
    flags_.ref_received   = true;
  }

  if ((control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
       control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
       control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
      control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
  }

  return;
};

void Plugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg) {
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    speed_limits_ = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                                    twist_msg.twist.linear.z);
    pid_3D_position_handler_->setOutputSaturation(speed_limits_);
    pid_3D_velocity_handler_->setOutputSaturation(speed_limits_);
    pid_3D_trajectory_handler_->setOutputSaturation(speed_limits_);
    return;
  }

  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED &&
      control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
    return;
  }

  control_ref_.velocity =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

  if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    control_ref_.yaw.y() = twist_msg.twist.angular.z;
  }

  flags_.ref_received = true;
  return;
};

void Plugin::updateReference(const as2_msgs::msg::TrajectoryPoint &traj_msg) {
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  control_ref_.position =
      Eigen::Vector3d(traj_msg.position.x, traj_msg.position.y, traj_msg.position.z);

  control_ref_.velocity = Eigen::Vector3d(traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z);

  control_ref_.yaw.x() = traj_msg.yaw_angle;

  flags_.ref_received = true;
  return;
};

bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                     const as2_msgs::msg::ControlMode &out_mode) {
  if (!flags_.plugin_parameters_read) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Plugin parameters not read yet, can not set mode");
    return false;
  }

  if (!flags_.position_controller_parameters_read) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "Position controller parameters not read, can not set mode");
    for (auto &param : position_control_parameters_to_read_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
    }
    return false;
  }

  if (in_mode.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY &&
      !flags_.trajectory_controller_parameters_read) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "Trajectory controller parameters not read yet, can not set mode to TRAJECTORY");
    for (auto &param : trajectory_control_parameters_to_read_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
    }
    return false;
  } else if ((in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED ||
              in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
             (!flags_.velocity_controller_parameters_read && !use_bypass_)) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "Velocity controller parameters not read yet and bypass is not used, can not set "
                "mode to SPEED or SPEED_IN_A_PLANE");
    if (in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED) {
      for (auto &param : velocity_control_parameters_to_read_) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
      }
    } else {
      for (auto &param : speed_in_a_plane_control_parameters_to_read_) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
      }
    }
    return false;
  }

  if (in_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE &&
      !flags_.yaw_controller_parameters_read) {
    RCLCPP_WARN(node_ptr_->get_logger(),
                "Yaw controller parameters not read yet, can not set mode to YAW_ANGLE");
    return false;
  }

  if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    control_mode_in_.control_mode    = in_mode.control_mode;
    control_mode_in_.yaw_mode        = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
    hover_flag_                      = true;
  } else {
    control_mode_in_ = in_mode;
  }

  flags_.state_received = false;
  flags_.ref_received   = false;
  control_mode_out_     = out_mode;

  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
      control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
      control_mode_in_.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY) {
    input_pose_frame_id_   = enu_frame_id_;
    output_twist_frame_id_ = enu_frame_id_;
  } else if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
             control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
    input_pose_frame_id_ = enu_frame_id_;
    switch (control_mode_out_.reference_frame) {
      case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
        input_twist_frame_id_  = flu_frame_id_;
        output_twist_frame_id_ = flu_frame_id_;
        break;
      case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      default:
        input_twist_frame_id_  = enu_frame_id_;
        output_twist_frame_id_ = enu_frame_id_;
        break;
    }
  }

  return true;
};

std::string Plugin::getDesiredPoseFrameId() { return input_pose_frame_id_; }

std::string Plugin::getDesiredTwistFrameId() { return input_twist_frame_id_; }

bool Plugin::computeOutput(double dt,
                           geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::TwistStamped &twist,
                           as2_msgs::msg::Thrust &thrust) {
  if (!flags_.state_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
    return false;
  }

  if (!flags_.ref_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "State changed, but ref not recived yet");
    return false;
  }

  resetCommands();

  switch (control_mode_in_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
    case as2_msgs::msg::ControlMode::POSITION:
      control_command_.velocity =
          pid_3D_position_handler_->computeControl(dt, uav_state_.position, control_ref_.position);

      control_command_.velocity = pid_3D_position_handler_->saturateOutput(
          control_command_.velocity, speed_limits_, proportional_limitation_);
      break;
    case as2_msgs::msg::ControlMode::SPEED: {
      if (use_bypass_) {
        control_command_.velocity = control_ref_.velocity;
      } else {
        control_command_.velocity = pid_3D_velocity_handler_->computeControl(
            dt, uav_state_.velocity, control_ref_.velocity);
      }
      break;
    }
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
      if (use_bypass_) {
        control_command_.velocity = control_ref_.velocity;
      } else {
        control_command_.velocity = pid_3D_speed_in_a_plane_handler_->computeControl(
            dt, uav_state_.velocity, control_ref_.velocity);
      }

      control_command_.velocity.z() = pid_1D_speed_in_a_plane_handler_->computeControl(
          dt, uav_state_.position.z(), control_ref_.position.z());

      break;
    }
    case as2_msgs::msg::ControlMode::TRAJECTORY: {
      control_command_.velocity =
          pid_3D_trajectory_handler_->computeControl(dt, uav_state_.position, control_ref_.position,
                                                     uav_state_.velocity, control_ref_.velocity);

      control_command_.velocity = pid_3D_trajectory_handler_->saturateOutput(
          control_command_.velocity, speed_limits_, proportional_limitation_);
      break;
    }
    default:
      auto &clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
      return false;
      break;
  }

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE: {
      double yaw_error = as2::frame::angleMinError(control_ref_.yaw.x(), uav_state_.yaw.x());
      control_command_.yaw_speed = pid_yaw_handler_->computeControl(dt, yaw_error);
      break;
    }
    case as2_msgs::msg::ControlMode::YAW_SPEED: {
      control_command_.yaw_speed = control_ref_.yaw.y();
      break;
    }
    default:
      auto &clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
      return false;
      break;
  }

  return getOutput(twist);
}

bool Plugin::getOutput(geometry_msgs::msg::TwistStamped &_twist_msg) {
  _twist_msg.header.frame_id = output_twist_frame_id_;

  _twist_msg.twist.linear.x = control_command_.velocity.x();
  _twist_msg.twist.linear.y = control_command_.velocity.y();
  _twist_msg.twist.linear.z = control_command_.velocity.z();

  _twist_msg.twist.angular.x = 0;
  _twist_msg.twist.angular.y = 0;
  _twist_msg.twist.angular.z = control_command_.yaw_speed;
  return true;
};

}  // namespace pid_speed_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pid_speed_controller::Plugin,
                       as2_motion_controller_plugin_base::ControllerBase)
