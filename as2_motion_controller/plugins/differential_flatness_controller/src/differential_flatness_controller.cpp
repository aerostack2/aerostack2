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
 *  @file       differential_flatness_controller.cpp
 *  @brief      Differential flatness controller plugin for the Aerostack framework.
 *  @authors    Miguel Fernández Cortizas
 *              Rafael Perez-Segui
 ********************************************************************************************/

#include "differential_flatness_controller.hpp"

namespace differential_flatness_controller
{

void Plugin::ownInitialize()
{
  // TrajectorySetpoints encodes pose and twist in the same frame.
  setDesiredTwistFrameId(getDesiredPoseFrameId());

  reset();
}

std::vector<std::string> Plugin::requiredParameters() const
{
  return {
    "mass",
    "trajectory_control.antiwindup_cte",
    "trajectory_control.kp.x",
    "trajectory_control.kp.y",
    "trajectory_control.kp.z",
    "trajectory_control.ki.x",
    "trajectory_control.ki.y",
    "trajectory_control.ki.z",
    "trajectory_control.kd.x",
    "trajectory_control.kd.y",
    "trajectory_control.kd.z",
    "trajectory_control.roll_control.kp",
    "trajectory_control.pitch_control.kp",
    "trajectory_control.yaw_control.kp",
  };
}

as2_msgs::msg::ControlMode Plugin::hoverMode() const
{
  as2_msgs::msg::ControlMode mode;
  mode.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  return mode;
}

void Plugin::updateParameter(const std::string & name, const rclcpp::Parameter & param)
{
  if (name == "mass") {
    mass_ = param.as_double();
  } else if (name == "trajectory_control.antiwindup_cte") {
    antiwindup_cte_ = param.as_double();
  } else if (name == "trajectory_control.kp.x") {
    Kp_(0, 0) = param.as_double();
  } else if (name == "trajectory_control.kp.y") {
    Kp_(1, 1) = param.as_double();
  } else if (name == "trajectory_control.kp.z") {
    Kp_(2, 2) = param.as_double();
  } else if (name == "trajectory_control.ki.x") {
    Ki_(0, 0) = param.as_double();
  } else if (name == "trajectory_control.ki.y") {
    Ki_(1, 1) = param.as_double();
  } else if (name == "trajectory_control.ki.z") {
    Ki_(2, 2) = param.as_double();
  } else if (name == "trajectory_control.kd.x") {
    Kd_(0, 0) = param.as_double();
  } else if (name == "trajectory_control.kd.y") {
    Kd_(1, 1) = param.as_double();
  } else if (name == "trajectory_control.kd.z") {
    Kd_(2, 2) = param.as_double();
  } else if (name == "trajectory_control.roll_control.kp") {
    Kp_ang_mat_(0, 0) = param.as_double();
  } else if (name == "trajectory_control.pitch_control.kp") {
    Kp_ang_mat_(1, 1) = param.as_double();
  } else if (name == "trajectory_control.yaw_control.kp") {
    Kp_ang_mat_(2, 2) = param.as_double();
  } else {
    RCLCPP_ERROR(
      getNodePtr()->get_logger(), "Unknown parameter '%s'", name.c_str());
  }
}

void Plugin::reset()
{
  ControllerBase::reset();
  resetReferences();
  resetState();
  resetCommands();
}

bool Plugin::onSetMode(
  const as2_msgs::msg::ControlMode & mode_in,
  const as2_msgs::msg::ControlMode & mode_out)
{
  (void)mode_in;
  (void)mode_out;

  // A zero gain block produces no command, so the mode is refused instead of
  // flying an inert controller.
  if (Kp_.isZero() && Kd_.isZero() && Ki_.isZero()) {
    RCLCPP_ERROR(
      getNodePtr()->get_logger(),
      "The position loop has all its gains at zero, the mode cannot be served");
    return false;
  }
  if (Kp_ang_mat_.isZero()) {
    RCLCPP_ERROR(
      getNodePtr()->get_logger(),
      "The attitude loop has all its gains at zero, the mode cannot be served");
    return false;
  }
  if (mass_ <= 0.0) {
    RCLCPP_ERROR(
      getNodePtr()->get_logger(), "The mass must be positive, the mode cannot be served");
    return false;
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
  uav_state_.attitude_state =
    tf2::Quaternion(
    pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
    pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
}

void Plugin::onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & trajectory_setpoints_msg)
{
  if (getControlModeIn().control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  as2_msgs::msg::TrajectoryPoint traj_msg = trajectory_setpoints_msg.setpoints[0];

  control_ref_.position =
    Eigen::Vector3d(traj_msg.position.x, traj_msg.position.y, traj_msg.position.z);
  control_ref_.velocity = Eigen::Vector3d(traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z);
  control_ref_.acceleration =
    Eigen::Vector3d(traj_msg.acceleration.x, traj_msg.acceleration.y, traj_msg.acceleration.z);
  control_ref_.yaw = traj_msg.yaw_angle;
}

bool Plugin::computeOutput(
  double dt,
  geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & twist,
  as2_msgs::msg::Thrust & thrust)
{
  (void)pose;

  resetCommands();

  switch (getControlModeIn().yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      break;
    default: {
        auto & clk = *getNodePtr()->get_clock();
        RCLCPP_ERROR_THROTTLE(getNodePtr()->get_logger(), clk, 5000, "Unknown yaw mode");
        return false;
      }
  }

  switch (getControlModeIn().control_mode) {
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      control_command_ = computeTrajectoryControl(
        dt, uav_state_.position, uav_state_.velocity,
        uav_state_.attitude_state, control_ref_.position,
        control_ref_.velocity, control_ref_.acceleration,
        control_ref_.yaw);
      break;
    default: {
        auto & clk = *getNodePtr()->get_clock();
        RCLCPP_ERROR_THROTTLE(getNodePtr()->get_logger(), clk, 5000, "Unknown control mode");
        return false;
      }
  }

  return getOutput(twist, thrust);
}

// ===== Internal helpers =====================================================


inline void Plugin::resetState() {uav_state_ = UAV_state();}

void Plugin::resetReferences()
{
  control_ref_.position = uav_state_.position;
  control_ref_.velocity = Eigen::Vector3d::Zero();
  control_ref_.acceleration = Eigen::Vector3d::Zero();
  control_ref_.yaw = as2::frame::getYawFromQuaternion(uav_state_.attitude_state);
}

void Plugin::resetCommands()
{
  control_command_.PQR = Eigen::Vector3d::Zero();
  control_command_.thrust = 0.0;
  accum_pos_error_ = Eigen::Vector3d::Zero();
}

Eigen::Vector3d Plugin::getForce(
  const double & _dt,
  const Eigen::Vector3d & _pos_state,
  const Eigen::Vector3d & _vel_state,
  const Eigen::Vector3d & _pos_reference,
  const Eigen::Vector3d & _vel_reference,
  const Eigen::Vector3d & _acc_reference)
{
  const Eigen::Vector3d position_error = _pos_reference - _pos_state;
  const Eigen::Vector3d velocity_error = _vel_reference - _vel_state;

  // TODO(miferco97): check if apply _dt to each constant or apply it to the whole vector
  // each iteration
  accum_pos_error_ += position_error * _dt;

  for (uint8_t j = 0; j < 3; j++) {
    double antiwindup_value = antiwindup_cte_ / Ki_.diagonal()[j];
    accum_pos_error_[j] = std::clamp(accum_pos_error_[j], -antiwindup_value, antiwindup_value);
  }

  const Eigen::Vector3d desired_force = Kp_ * position_error + Kd_ * velocity_error +
    Ki_ * accum_pos_error_ - mass_ * gravitational_accel_ +
    mass_ * _acc_reference;

  return std::move(desired_force);  // use std::move to avoid copy (force RVO)
}

BodyRates_command Plugin::computeTrajectoryControl(
  const double & _dt,
  const Eigen::Vector3d & _pos_state,
  const Eigen::Vector3d & _vel_state,
  const tf2::Quaternion & _attitude_state,
  const Eigen::Vector3d & _pos_reference,
  const Eigen::Vector3d & _vel_reference,
  const Eigen::Vector3d & _acc_reference,
  const double & _yaw_angle_reference)
{
  Eigen::Vector3d desired_force =
    getForce(_dt, _pos_state, _vel_state, _pos_reference, _vel_reference, _acc_reference);

  const tf2::Matrix3x3 rot_matrix_tf2(_attitude_state);

  Eigen::Matrix3d rot_matrix;
  rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
    rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2], rot_matrix_tf2[2][0],
    rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

  const Eigen::Vector3d xc_des(cos(_yaw_angle_reference), sin(_yaw_angle_reference), 0);
  const Eigen::Vector3d zb_des = desired_force.normalized();
  const Eigen::Vector3d yb_des = zb_des.cross(xc_des).normalized();
  const Eigen::Vector3d xb_des = yb_des.cross(zb_des).normalized();

  Eigen::Matrix3d R_des;
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  const Eigen::Matrix3d Mat_e_rot =
    (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

  const Eigen::Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
  const Eigen::Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

  BodyRates_command body_rates_command;
  body_rates_command.thrust =
    static_cast<double>(desired_force.dot(rot_matrix.col(2).normalized()));
  body_rates_command.PQR = -Kp_ang_mat_ * E_rot;

  return std::move(body_rates_command);  // use std::move to avoid copy (force RVO)
}

bool Plugin::getOutput(
  geometry_msgs::msg::TwistStamped & twist_msg,
  as2_msgs::msg::Thrust & thrust_msg)
{
  twist_msg.header.stamp = getNodePtr()->now();
  twist_msg.header.frame_id = getNodePtr()->getBaseFrameId();
  twist_msg.twist.angular.x = control_command_.PQR.x();
  twist_msg.twist.angular.y = control_command_.PQR.y();
  twist_msg.twist.angular.z = control_command_.PQR.z();

  thrust_msg.header.stamp = getNodePtr()->now();
  thrust_msg.header.frame_id = getNodePtr()->getBaseFrameId();
  thrust_msg.thrust = control_command_.thrust;
  return true;
}

}  // namespace differential_flatness_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  differential_flatness_controller::Plugin,
  as2_motion_controller_plugin_base::ControllerBase)
