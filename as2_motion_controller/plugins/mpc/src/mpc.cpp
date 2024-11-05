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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 *  \file       mpc.cpp
 *  \brief      Model Predictive Controller plugin for the Aerostack framework.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************************/

#include "mpc.hpp"

namespace mpc
{

void Plugin::ownInitialize()
{
  // TODO(RPS98): Remove this, it is not needed
  odom_frame_id_ = as2::tf::generateTfName(node_ptr_, odom_frame_id_);
  base_link_frame_id_ = as2::tf::generateTfName(node_ptr_, base_link_frame_id_);

  mpc_data_ = mpc_.get_data();

  RCLCPP_INFO(
    node_ptr_->get_logger(),
    "MPC plugin initialized with N = %d prediction steps and Tf = %f "
    "s sampling time",
    mpc_.get_prediction_steps(), mpc_.get_prediction_time_step());

  return;
}

void Plugin::updateState(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const geometry_msgs::msg::TwistStamped & twist_msg)
{
  // TODO(RPS98): Remove this, it is not needed
  if (pose_msg.header.frame_id != odom_frame_id_ &&
    twist_msg.header.frame_id != odom_frame_id_)
  {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "Pose and Twist frame_id are not desired ones");
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Recived: %s, %s",
      pose_msg.header.frame_id.c_str(),
      twist_msg.header.frame_id.c_str());
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Desired: %s, %s",
      odom_frame_id_.c_str(), odom_frame_id_.c_str());
    return;
  }

  mpc_data_->state.set_data(0, pose_msg.pose.position.x);
  mpc_data_->state.set_data(1, pose_msg.pose.position.y);
  mpc_data_->state.set_data(2, pose_msg.pose.position.z);
  mpc_data_->state.set_data(3, pose_msg.pose.orientation.w);
  mpc_data_->state.set_data(4, pose_msg.pose.orientation.x);
  mpc_data_->state.set_data(5, pose_msg.pose.orientation.y);
  mpc_data_->state.set_data(6, pose_msg.pose.orientation.z);
  mpc_data_->state.set_data(7, twist_msg.twist.linear.x);
  mpc_data_->state.set_data(8, twist_msg.twist.linear.y);
  mpc_data_->state.set_data(9, twist_msg.twist.linear.z);

  // TODO(RPS98): Manage hover mode
  if (flags_.hover_flag_) {
    // Set current state as reference
    as2_msgs::msg::TrajectorySetpoints trajectory_setpoints_msg;
    trajectory_setpoints_msg.header.stamp = node_ptr_->now();
    trajectory_setpoints_msg.header.frame_id = odom_frame_id_;
    trajectory_setpoints_msg.setpoints.resize(1);
    trajectory_setpoints_msg.setpoints[0].position.x = pose_msg.pose.position.x;
    trajectory_setpoints_msg.setpoints[0].position.y = pose_msg.pose.position.y;
    trajectory_setpoints_msg.setpoints[0].position.z = pose_msg.pose.position.z;
    double roll, pitch, yaw;
    as2::frame::quaternionToEuler(pose_msg.pose.orientation, roll, pitch, yaw);
    trajectory_setpoints_msg.setpoints[0].yaw_angle = yaw;
    trajectory_setpoints_msg.setpoints[0].twist.x = 0.0;
    trajectory_setpoints_msg.setpoints[0].twist.y = 0.0;
    trajectory_setpoints_msg.setpoints[0].twist.z = 0.0;
    updateReference(trajectory_setpoints_msg);

    flags_.ref_received = true;
    flags_.hover_flag_ = false;
  }
  return;
}

void Plugin::updateReference(
  const as2_msgs::msg::TrajectorySetpoints & trajectory_setpoints_msg)
{
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  // Set intermediate values
  const as2_msgs::msg::TrajectoryPoint * traj_setpoint = nullptr;
  for (int i = 0; i < mpc_.get_prediction_steps(); i++) {
    if (i >= trajectory_setpoints_msg.setpoints.size()) {
      traj_setpoint = &trajectory_setpoints_msg.setpoints.back();
    } else {
      traj_setpoint = &trajectory_setpoints_msg.setpoints[i];
    }
    mpc_data_->reference.set_data(i, 0, traj_setpoint->position.x);
    mpc_data_->reference.set_data(i, 1, traj_setpoint->position.y);
    mpc_data_->reference.set_data(i, 2, traj_setpoint->position.z);

    tf2::Quaternion q;
    as2::frame::eulerToQuaternion(0.0, 0.0, traj_setpoint->yaw_angle, q);
    mpc_data_->reference.set_data(i, 3, q.w());
    mpc_data_->reference.set_data(i, 4, q.x());
    mpc_data_->reference.set_data(i, 5, q.y());
    mpc_data_->reference.set_data(i, 6, q.z());

    mpc_data_->reference.set_data(i, 7, traj_setpoint->twist.x);
    mpc_data_->reference.set_data(i, 8, traj_setpoint->twist.y);
    mpc_data_->reference.set_data(i, 9, traj_setpoint->twist.z);
  }
  if (mpc_.get_prediction_steps() >=
    trajectory_setpoints_msg.setpoints.size())
  {
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(), "Setpoints size: %zu, prediction steps: %d",
      trajectory_setpoints_msg.setpoints.size(), mpc_.get_prediction_steps());
    RCLCPP_WARN_ONCE(
      node_ptr_->get_logger(), "Desired setpoint size: %d",
      mpc_.get_prediction_steps() + 1);
  }

  // Set end reference values
  traj_setpoint = &trajectory_setpoints_msg.setpoints.back();
  mpc_data_->reference_end.set_data(0, traj_setpoint->position.x);
  mpc_data_->reference_end.set_data(1, traj_setpoint->position.y);
  mpc_data_->reference_end.set_data(2, traj_setpoint->position.z);
  tf2::Quaternion q;
  as2::frame::eulerToQuaternion(0.0, 0.0, traj_setpoint->yaw_angle, q);
  mpc_data_->reference_end.set_data(3, q.w());
  mpc_data_->reference_end.set_data(4, q.x());
  mpc_data_->reference_end.set_data(5, q.y());
  mpc_data_->reference_end.set_data(6, q.z());
  mpc_data_->reference_end.set_data(7, traj_setpoint->twist.x);
  mpc_data_->reference_end.set_data(8, traj_setpoint->twist.y);
  mpc_data_->reference_end.set_data(9, traj_setpoint->twist.z);

  flags_.ref_received = true;
  return;
}

bool Plugin::computeOutput(
  double dt, geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & twist,
  as2_msgs::msg::Thrust & thrust)
{
  auto & clk = *node_ptr_->get_clock();
  if (!flags_.ref_received) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), clk, 5000,
      "References not recived yet");
    return false;
  }

  if (!flags_.parameters_read) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), clk, 5000,
      "Parameters not read yet");
    for (auto & param : parameters_to_read_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(), "Parameter %s not read yet",
        param.c_str());
    }
    return false;
  }

  // TODO(RPS98): Check if syncronization is needed
  mpc_.solve();

  // Update time stamp
  twist.header.stamp = node_ptr_->now();
  twist.header.frame_id = base_link_frame_id_;
  thrust.header.stamp = node_ptr_->now();
  thrust.header.frame_id = base_link_frame_id_;

  // Get the control commands
  thrust.thrust = mpc_data_->control.data[0];
  twist.twist.angular.x = mpc_data_->control.data[1];
  twist.twist.angular.y = mpc_data_->control.data[2];
  twist.twist.angular.z = mpc_data_->control.data[3];

  return true;
}

bool Plugin::setMode(
  const as2_msgs::msg::ControlMode & in_mode,
  const as2_msgs::msg::ControlMode & out_mode)
{
  if (!flags_.parameters_read) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Plugin parameters not read yet, can not set mode");
    return false;
  }

  if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    control_mode_in_.control_mode = in_mode.control_mode;
    control_mode_in_.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_in_.reference_frame =
      as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
    flags_.hover_flag_ = true;
  } else {
    control_mode_in_ = in_mode;
  }

  flags_.ref_received = false;

  return true;
}

bool Plugin::updateParams(const std::vector<rclcpp::Parameter> & _params_list)
{
  for (auto & param : _params_list) {
    if (param.get_name() == "mpc.Q") {
      auto q = param.as_double_array();
      for (size_t i = 0; i < q.size(); ++i) {
        mpc_.get_gains()->set_Q(i, q[i]);
      }
      mpc_.update_gains();
    } else if (param.get_name() == "mpc.Qe") {
      auto qe = param.as_double_array();
      for (size_t i = 0; i < qe.size(); ++i) {
        mpc_.get_gains()->set_Q_end(i, qe[i]);
      }
      mpc_.update_gains();
    } else if (param.get_name() == "mpc.R") {
      auto r = param.as_double_array();
      for (size_t i = 0; i < r.size(); ++i) {
        mpc_.get_gains()->set_R(i, r[i]);
      }
      mpc_.update_gains();
    } else if (param.get_name() == "mpc.lbu") {
      auto lbu = param.as_double_array();
      for (size_t i = 0; i < lbu.size(); ++i) {
        mpc_.get_bounds()->set_lbu(i, lbu[i]);
      }
      mpc_.update_bounds();
    } else if (param.get_name() == "mpc.ubu") {
      auto ubu = param.as_double_array();
      for (size_t i = 0; i < ubu.size(); ++i) {
        mpc_.get_bounds()->set_ubu(i, ubu[i]);
      }
      mpc_.update_bounds();
    } else if (param.get_name() == "mpc.p") {
      auto p = param.as_double_array();
      for (size_t i = 0; i < p.size(); ++i) {
        mpc_.get_online_params()->set_data(i, p[i]);
      }
      mpc_.update_online_params();
    }
    flags_.parameters_read =
      checkParamList(param.get_name(), parameters_to_read_);
  }
  if (flags_.parameters_read) {
    RCLCPP_INFO(node_ptr_->get_logger(), "All parameters read");
  } else {
    for (auto & param : parameters_to_read_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(), "Parameter %s not read yet",
        param.c_str());
    }
  }
  return true;
}

bool Plugin::checkParamList(
  const std::string & param,
  std::vector<std::string> & _params_list)
{
  if (find(_params_list.begin(), _params_list.end(), param) !=
    _params_list.end())
  {
    // Remove the parameter from the list of parameters to be read
    _params_list.erase(
      std::remove(_params_list.begin(), _params_list.end(), param),
      _params_list.end());
  }
  return !_params_list.size();  // Return true if the list is empty
}

void Plugin::reset()
{
  // TODO(RPS98): Implement
}

}  // namespace mpc

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mpc::Plugin,
  as2_motion_controller_plugin_base::ControllerBase)
