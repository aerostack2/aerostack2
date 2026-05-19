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

namespace
{

// Lazily declare and read an optional string parameter that holds a topic
// name. An empty value disables the publisher.
std::string declareOptionalTopic(rclcpp::Node * node, const std::string & name)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::string>(name, "");
  }
  return node->get_parameter(name).as_string();
}

}  // namespace

void Plugin::ownInitialize()
{
  // TrajectorySetpoints encodes pose and twist in the same frame.
  setDesiredTwistFrameId(getDesiredPoseFrameId());

  const std::string desired_velocity_topic =
    declareOptionalTopic(getNodePtr(), param("debug.desired_velocity_topic"));
  if (!desired_velocity_topic.empty()) {
    debug_desired_velocity_pub_ =
      getNodePtr()->create_publisher<geometry_msgs::msg::TwistStamped>(
      desired_velocity_topic, rclcpp::SensorDataQoS());
  }

  reset();
}

std::vector<std::string> Plugin::getEssentialParameters() const
{
  std::vector<std::string> out;
  out.reserve(parameters_tail_.size());
  for (const auto & tail : parameters_tail_) {
    out.push_back(param(tail));
  }
  return out;
}

void Plugin::updateParameter(const rclcpp::Parameter & p)
{
  const std::string ns_prefix = getPluginParamNamespace().empty() ?
    std::string() :
    getPluginParamNamespace() + ".";
  const std::string & full_name = p.get_name();
  const std::string tail = ns_prefix.empty() ? full_name :
    full_name.substr(ns_prefix.size());
  updateDFParameter(tail, p);
}

void Plugin::reset()
{
  ControllerBase::reset();
  resetReferences();
  resetState();
  resetCommands();
}

bool Plugin::setMode(
  const as2_msgs::msg::ControlMode & in_mode,
  const as2_msgs::msg::ControlMode & out_mode)
{
  if (!essentialParamsReady()) {
    RCLCPP_WARN(getNodePtr()->get_logger(), "Plugin parameters not read yet, can not set mode");
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
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY &&
    control_mode_in_.control_mode != as2_msgs::msg::ControlMode::HOVER)
  {
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

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      break;
    default: {
        auto & clk = *getNodePtr()->get_clock();
        RCLCPP_ERROR_THROTTLE(getNodePtr()->get_logger(), clk, 5000, "Unknown yaw mode");
        return false;
      }
  }

  switch (control_mode_in_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER:
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

  if (debug_desired_velocity_pub_) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = getNodePtr()->now();
    msg.header.frame_id = getDesiredTwistFrameId();
    msg.twist.linear.x = control_ref_.velocity.x();
    msg.twist.linear.y = control_ref_.velocity.y();
    msg.twist.linear.z = control_ref_.velocity.z();
    debug_desired_velocity_pub_->publish(msg);
  }

  return getOutput(twist, thrust);
}

// ===== Internal helpers =====================================================

void Plugin::updateDFParameter(
  const std::string & _parameter_name,
  const rclcpp::Parameter & _param)
{
  // For trajectory_control.* gains the plugin code historically uses the
  // post-dot subname (e.g. "kp.x"), so strip the leading "trajectory_control."
  // when present to keep the existing dispatch.
  const std::string controller = _parameter_name.substr(0, _parameter_name.find('.'));
  const std::string subname = _parameter_name.find('.') == std::string::npos ?
    _parameter_name :
    _parameter_name.substr(_parameter_name.find('.') + 1);
  const std::string dispatch_name = (controller == "trajectory_control") ?
    subname : _parameter_name;

  if (dispatch_name == "mass") {
    mass_ = _param.get_value<double>();
  } else if (dispatch_name == "antiwindup_cte") {
    antiwindup_cte_ = _param.get_value<double>();
  } else if (dispatch_name == "kp.x") {
    Kp_(0, 0) = _param.get_value<double>();
  } else if (dispatch_name == "kp.y") {
    Kp_(1, 1) = _param.get_value<double>();
  } else if (dispatch_name == "kp.z") {
    Kp_(2, 2) = _param.get_value<double>();
  } else if (dispatch_name == "ki.x") {
    Ki_(0, 0) = _param.get_value<double>();
  } else if (dispatch_name == "ki.y") {
    Ki_(1, 1) = _param.get_value<double>();
  } else if (dispatch_name == "ki.z") {
    Ki_(2, 2) = _param.get_value<double>();
  } else if (dispatch_name == "kd.x") {
    Kd_(0, 0) = _param.get_value<double>();
  } else if (dispatch_name == "kd.y") {
    Kd_(1, 1) = _param.get_value<double>();
  } else if (dispatch_name == "kd.z") {
    Kd_(2, 2) = _param.get_value<double>();
  } else if (dispatch_name == "roll_control.kp") {
    Kp_ang_mat_(0, 0) = _param.get_value<double>();
  } else if (dispatch_name == "pitch_control.kp") {
    Kp_ang_mat_(1, 1) = _param.get_value<double>();
  } else if (dispatch_name == "yaw_control.kp") {
    Kp_ang_mat_(2, 2) = _param.get_value<double>();
  }
}

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

Acro_command Plugin::computeTrajectoryControl(
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

  Acro_command acro_command;
  acro_command.thrust = static_cast<double>(desired_force.dot(rot_matrix.col(2).normalized()));
  acro_command.PQR = -Kp_ang_mat_ * E_rot;

  return std::move(acro_command);  // use std::move to avoid copy (force RVO)
}

bool Plugin::getOutput(
  geometry_msgs::msg::TwistStamped & twist_msg,
  as2_msgs::msg::Thrust & thrust_msg)
{
  twist_msg.header.stamp = getNodePtr()->now();
  twist_msg.header.frame_id = getBaseLinkFrameId();
  twist_msg.twist.angular.x = control_command_.PQR.x();
  twist_msg.twist.angular.y = control_command_.PQR.y();
  twist_msg.twist.angular.z = control_command_.PQR.z();

  thrust_msg.header.stamp = getNodePtr()->now();
  thrust_msg.header.frame_id = getBaseLinkFrameId();
  thrust_msg.thrust = control_command_.thrust;
  return true;
}

}  // namespace differential_flatness_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  differential_flatness_controller::Plugin,
  as2_motion_controller_plugin_base::ControllerBase)
