/*!*******************************************************************************************
 *  \file       differential_flatness_controller.cpp
 *  \brief      Differential flatness controller plugin for the Aerostack framework.
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

#include "differential_flatness_controller.hpp"

namespace differential_flatness_controller {
void Plugin::ownInitialize() {
  odom_frame_id_      = as2::tf::generateTfName(node_ptr_, odom_frame_id_);
  base_link_frame_id_ = as2::tf::generateTfName(node_ptr_, base_link_frame_id_);
  reset();
  return;
};

bool Plugin::updateParams(const std::vector<rclcpp::Parameter>& _params_list) {
  for (auto& param : _params_list) {
    updateDFParameter(param.get_name(), param);
  }
  return true;
}

bool Plugin::checkParamList(const std::string& param, std::vector<std::string>& _params_list) {
  if (find(_params_list.begin(), _params_list.end(), param) != _params_list.end()) {
    // Remove the parameter from the list of parameters to be read
    _params_list.erase(std::remove(_params_list.begin(), _params_list.end(), param),
                       _params_list.end());
  };
  return !_params_list.size();  // Return true if the list is empty
};

void Plugin::updateDFParameter(std::string _parameter_name, const rclcpp::Parameter& _param) {
  std::string controller    = _parameter_name.substr(0, _parameter_name.find("."));
  std::string param_subname = _parameter_name.substr(_parameter_name.find(".") + 1);

  if (controller == "trajectory_control") {
    // TODO check if this is a good way to do it or it is better to write the full name
    _parameter_name = param_subname;
  }

  if (_parameter_name == "mass") {
    mass_ = _param.get_value<double>();
  } else if (_parameter_name == "antiwindup_cte") {
    antiwindup_cte_ = _param.get_value<double>();
  } else if (_parameter_name == "kp.x") {
    Kp_(0, 0) = _param.get_value<double>();
  } else if (_parameter_name == "kp.y") {
    Kp_(1, 1) = _param.get_value<double>();
  } else if (_parameter_name == "kp.z") {
    Kp_(2, 2) = _param.get_value<double>();
  } else if (_parameter_name == "ki.x") {
    Ki_(0, 0) = _param.get_value<double>();
  } else if (_parameter_name == "ki.y") {
    Ki_(1, 1) = _param.get_value<double>();
  } else if (_parameter_name == "ki.z") {
    Ki_(2, 2) = _param.get_value<double>();
  } else if (_parameter_name == "kd.x") {
    Kd_(0, 0) = _param.get_value<double>();
  } else if (_parameter_name == "kd.y") {
    Kd_(1, 1) = _param.get_value<double>();
  } else if (_parameter_name == "kd.z") {
    Kd_(2, 2) = _param.get_value<double>();
  } else if (_parameter_name == "roll_control.kp") {
    Kp_ang_mat_(0, 0) = _param.get_value<double>();
  } else if (_parameter_name == "pitch_control.kp") {
    Kp_ang_mat_(1, 1) = _param.get_value<double>();
  } else if (_parameter_name == "yaw_control.kp") {
    Kp_ang_mat_(2, 2) = _param.get_value<double>();
  }
  flags_.parameters_read = checkParamList(_param.get_name(), parameters_to_read_);
  return;
}

void Plugin::reset() {
  resetReferences();
  resetState();
  resetCommands();
}

inline void Plugin::resetState() { uav_state_ = UAV_state(); }

void Plugin::resetReferences() {
  control_ref_.position     = uav_state_.position;
  control_ref_.velocity     = Eigen::Vector3d::Zero();
  control_ref_.acceleration = Eigen::Vector3d::Zero();

  control_ref_.yaw = as2::frame::getYawFromQuaternion(uav_state_.attitude_state);
  return;
}

void Plugin::resetCommands() {
  control_command_.PQR    = Eigen::Vector3d::Zero();
  control_command_.thrust = 0.0;
  accum_pos_error_        = Eigen::Vector3d::Zero();
  return;
}

void Plugin::updateState(const geometry_msgs::msg::PoseStamped& pose_msg,
                         const geometry_msgs::msg::TwistStamped& twist_msg) {
  if (pose_msg.header.frame_id != odom_frame_id_ && twist_msg.header.frame_id != odom_frame_id_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Pose and Twist frame_id are not desired ones");
    RCLCPP_ERROR(node_ptr_->get_logger(), "Recived: %s, %s", pose_msg.header.frame_id.c_str(),
                 twist_msg.header.frame_id.c_str());
    RCLCPP_ERROR(node_ptr_->get_logger(), "Desired: %s, %s", odom_frame_id_.c_str(),
                 odom_frame_id_.c_str());
    return;
  }

  uav_state_.position =
      Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  uav_state_.velocity =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

  uav_state_.attitude_state =
      tf2::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

  if (hover_flag_) {
    resetReferences();
    flags_.ref_received = true;
    hover_flag_         = false;
  }

  flags_.state_received = true;
  return;
};

void Plugin::updateReference(const as2_msgs::msg::TrajectoryPoint& traj_msg) {
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  control_ref_.position =
      Eigen::Vector3d(traj_msg.position.x, traj_msg.position.y, traj_msg.position.z);

  control_ref_.velocity = Eigen::Vector3d(traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z);

  control_ref_.acceleration =
      Eigen::Vector3d(traj_msg.acceleration.x, traj_msg.acceleration.y, traj_msg.acceleration.z);

  control_ref_.yaw = traj_msg.yaw_angle;

  flags_.ref_received = true;
  return;
};

bool Plugin::setMode(const as2_msgs::msg::ControlMode& in_mode,
                     const as2_msgs::msg::ControlMode& out_mode) {
  if (!flags_.parameters_read) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Plugin parameters not read yet, can not set mode");
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

  flags_.ref_received   = false;
  flags_.state_received = false;

  control_mode_out_ = out_mode;
  return true;
};

bool Plugin::computeOutput(double dt,
                           geometry_msgs::msg::PoseStamped& pose,
                           geometry_msgs::msg::TwistStamped& twist,
                           as2_msgs::msg::Thrust& thrust) {
  auto& clk = *node_ptr_->get_clock();
  if (!flags_.state_received) {
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
    return false;
  }

  if (!flags_.ref_received) {
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "State changed, but ref not recived yet");
    return false;
  }

  if (!flags_.parameters_read) {
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Parameters not read yet");
    for (auto& param : parameters_to_read_) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read yet", param.c_str());
    }
    return false;
  }

  resetCommands();

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE: {
      break;
    }
    default:
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
      return false;
      break;
  }

  switch (control_mode_in_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      control_command_ = computeTrajectoryControl(dt, uav_state_.position, uav_state_.velocity,
                                                  uav_state_.attitude_state, control_ref_.position,
                                                  control_ref_.velocity, control_ref_.acceleration,
                                                  control_ref_.yaw);
      break;
    default:
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
      return false;
      break;
  }

  return getOutput(twist, thrust);
}

Eigen::Vector3d Plugin::getForce(const double& _dt,
                                 const Eigen::Vector3d& _pos_state,
                                 const Eigen::Vector3d& _vel_state,
                                 const Eigen::Vector3d& _pos_reference,
                                 const Eigen::Vector3d& _vel_reference,
                                 const Eigen::Vector3d& _acc_reference) {
  // Compute the error force contribution

  const Eigen::Vector3d position_error = _pos_reference - _pos_state;
  const Eigen::Vector3d velocity_error = _vel_reference - _vel_state;

  // TODO: check if apply _dt to each constant or apply it to the whole vector each iteration
  accum_pos_error_ += position_error * _dt;

  for (uint8_t j = 0; j < 3; j++) {
    double antiwindup_value = antiwindup_cte_ / Ki_.diagonal()[j];
    accum_pos_error_[j]     = std::clamp(accum_pos_error_[j], -antiwindup_value, antiwindup_value);
  }

  const Eigen::Vector3d desired_force = Kp_ * position_error + Kd_ * velocity_error +
                                        Ki_ * accum_pos_error_ - mass_ * gravitational_accel_ +
                                        mass_ * _acc_reference;

  return std::move(desired_force);  // use std::move to avoid copy (force RVO)
}

Acro_command Plugin::computeTrajectoryControl(const double& _dt,
                                              const Eigen::Vector3d& _pos_state,
                                              const Eigen::Vector3d& _vel_state,
                                              const tf2::Quaternion& _attitude_state,
                                              const Eigen::Vector3d& _pos_reference,
                                              const Eigen::Vector3d& _vel_reference,
                                              const Eigen::Vector3d& _acc_reference,
                                              const double& _yaw_angle_reference) {
  Eigen::Vector3d desired_force =
      getForce(_dt, _pos_state, _vel_state, _pos_reference, _vel_reference, _acc_reference);

  // Compute the desired attitude
  const tf2::Matrix3x3 rot_matrix_tf2(_attitude_state);

  Eigen::Matrix3d rot_matrix;
  rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
      rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2], rot_matrix_tf2[2][0],
      rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

  const Eigen::Vector3d xc_des(cos(_yaw_angle_reference), sin(_yaw_angle_reference), 0);
  const Eigen::Vector3d zb_des = desired_force.normalized();
  const Eigen::Vector3d yb_des = zb_des.cross(xc_des).normalized();
  const Eigen::Vector3d xb_des = yb_des.cross(zb_des).normalized();

  // Compute the rotation matrix desidered
  Eigen::Matrix3d R_des;
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  // Compute the rotation matrix error
  const Eigen::Matrix3d Mat_e_rot =
      (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

  const Eigen::Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
  const Eigen::Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

  Acro_command acro_command;
  acro_command.thrust = (float)desired_force.dot(rot_matrix.col(2).normalized());
  acro_command.PQR    = -Kp_ang_mat_ * E_rot;

  return std::move(acro_command);  // use std::move to avoid copy (force RVO)
}

bool Plugin::getOutput(geometry_msgs::msg::TwistStamped& twist_msg,
                       as2_msgs::msg::Thrust& thrust_msg) {
  twist_msg.header.stamp    = node_ptr_->now();
  twist_msg.header.frame_id = base_link_frame_id_;
  twist_msg.twist.angular.x = control_command_.PQR.x();
  twist_msg.twist.angular.y = control_command_.PQR.y();
  twist_msg.twist.angular.z = control_command_.PQR.z();

  thrust_msg.header.stamp    = node_ptr_->now();
  thrust_msg.header.frame_id = base_link_frame_id_;
  thrust_msg.thrust          = control_command_.thrust;
  return true;
};

}  // namespace differential_flatness_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(differential_flatness_controller::Plugin,
                       as2_motion_controller_plugin_base::ControllerBase)
