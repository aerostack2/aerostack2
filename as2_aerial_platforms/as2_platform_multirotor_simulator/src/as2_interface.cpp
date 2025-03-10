// Copyright 2025 Universidad Politécnica de Madrid
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

/**
 * @file as2_interface.cpp
 *
 * As2MultirotorSimulatorInterface class implementation
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "as2_platform_multirotor_simulator/as2_interface.hpp"

#include "as2_core/core_functions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

namespace as2_platform_multirotor_simulator
{

As2MultirotorSimulatorInterface::As2MultirotorSimulatorInterface(
  as2::Node * node_ptr)
: node_ptr_(node_ptr), tf_handler_(node_ptr)
{
  getParam("global_ref_frame", frame_id_earth_);
  getParam("odom_frame", frame_id_odom_);
  getParam("base_frame", frame_id_baselink_);
  frame_id_odom_ = as2::tf::generateTfName(node_ptr, frame_id_odom_);
  frame_id_baselink_ = as2::tf::generateTfName(node_ptr, frame_id_baselink_);

  getParam("use_odom_for_control", using_odom_for_control_);

  // Initial position
  getParam("vehicle_initial_pose.x", initial_position_.x());
  getParam("vehicle_initial_pose.y", initial_position_.y());
  getParam("vehicle_initial_pose.z", initial_position_.z());

  // Initial orientation
  double roll, pitch, yaw;
  getParam("vehicle_initial_pose.yaw", yaw);
  getParam("vehicle_initial_pose.pitch", pitch);
  getParam("vehicle_initial_pose.roll", roll);
  as2::frame::eulerToQuaternion(roll, pitch, yaw, initial_orientation_);
  initial_orientation_ = initial_orientation_.normalized();
}

void As2MultirotorSimulatorInterface::convertToOdom(
  const Kinematics & kinematics, nav_msgs::msg::Odometry & odometry,
  const builtin_interfaces::msg::Time & current_time)
{
  odometry.header.stamp = current_time;
  odometry.header.frame_id = frame_id_odom_;
  odometry.child_frame_id = frame_id_baselink_;
  odometry.pose.pose.position.x = kinematics.position.x();
  odometry.pose.pose.position.y = kinematics.position.y();
  odometry.pose.pose.position.z = kinematics.position.z();
  odometry.pose.pose.orientation.w = kinematics.orientation.w();
  odometry.pose.pose.orientation.x = kinematics.orientation.x();
  odometry.pose.pose.orientation.y = kinematics.orientation.y();
  odometry.pose.pose.orientation.z = kinematics.orientation.z();
  Eigen::Vector3d odom_linear_velocity_flu =
    as2::frame::transform(
    kinematics.orientation.inverse(), kinematics.linear_velocity);
  odometry.twist.twist.linear.x = odom_linear_velocity_flu.x();
  odometry.twist.twist.linear.y = odom_linear_velocity_flu.y();
  odometry.twist.twist.linear.z = odom_linear_velocity_flu.z();
  odometry.twist.twist.angular.x = kinematics.angular_velocity.x();
  odometry.twist.twist.angular.y = kinematics.angular_velocity.y();
  odometry.twist.twist.angular.z = kinematics.angular_velocity.z();
  return;
}

void As2MultirotorSimulatorInterface::convertToGroundTruth(
  const Kinematics & kinematics, geometry_msgs::msg::PoseStamped & ground_truth_pose,
  geometry_msgs::msg::TwistStamped & ground_truth_twist,
  const builtin_interfaces::msg::Time & current_time)
{
  // Convert to ground truth
  Eigen::Vector3d gt_linear_velocity_flu =
    as2::frame::transform(kinematics.orientation.inverse(), kinematics.linear_velocity);

  // Ground truth pose
  ground_truth_pose.header.stamp = current_time;
  ground_truth_pose.header.frame_id = frame_id_earth_;
  ground_truth_pose.pose.position.x = kinematics.position.x();
  ground_truth_pose.pose.position.y = kinematics.position.y();
  ground_truth_pose.pose.position.z = kinematics.position.z();
  ground_truth_pose.pose.orientation.w = kinematics.orientation.w();
  ground_truth_pose.pose.orientation.x = kinematics.orientation.x();
  ground_truth_pose.pose.orientation.y = kinematics.orientation.y();
  ground_truth_pose.pose.orientation.z = kinematics.orientation.z();

  // Ground truth twist
  ground_truth_twist.header.stamp = current_time;
  ground_truth_twist.header.frame_id = frame_id_baselink_;
  ground_truth_twist.twist.linear.x = gt_linear_velocity_flu.x();
  ground_truth_twist.twist.linear.y = gt_linear_velocity_flu.y();
  ground_truth_twist.twist.linear.z = gt_linear_velocity_flu.z();
  ground_truth_twist.twist.angular.x = kinematics.angular_velocity.x();
  ground_truth_twist.twist.angular.y = kinematics.angular_velocity.y();
  ground_truth_twist.twist.angular.z = kinematics.angular_velocity.z();
  return;
}

bool As2MultirotorSimulatorInterface::processCommand(
  geometry_msgs::msg::PoseStamped & pose_command)
{
  if (pose_command.header.frame_id != frame_id_earth_ &&
    !tf_handler_.tryConvert(pose_command, frame_id_earth_))
  {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Error converting pose command to %s frame from frame: %s",
      frame_id_earth_.c_str(), pose_command.header.frame_id.c_str());
    return false;
  }
  return true;
}

bool As2MultirotorSimulatorInterface::processCommand(
  geometry_msgs::msg::TwistStamped & twist_command)
{
  if (twist_command.header.frame_id != frame_id_earth_ &&
    !tf_handler_.tryConvert(twist_command, frame_id_earth_))
  {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Error converting twist command to %s frame from frame: %s",
      frame_id_earth_.c_str(), twist_command.header.frame_id.c_str());
    return false;
  }
  return true;
}

bool As2MultirotorSimulatorInterface::processCommand(
  as2_msgs::msg::TrajectorySetpoints trajectory_command)
{
  // TODO(RPS98): Improve this function for better performance
  as2_msgs::msg::TrajectoryPoint trajectory_point = trajectory_command.setpoints[0];
  geometry_msgs::msg::PoseStamped pose_command;
  pose_command.header = trajectory_command.header;
  pose_command.pose.position.x = trajectory_point.position.x;
  pose_command.pose.position.y = trajectory_point.position.y;
  pose_command.pose.position.z = trajectory_point.position.z;

  as2::frame::eulerToQuaternion(
    0.0, 0.0, trajectory_point.yaw_angle,
    pose_command.pose.orientation);

  geometry_msgs::msg::TwistStamped twist_command;
  twist_command.header = trajectory_command.header;
  twist_command.twist.linear.x = trajectory_point.twist.x;
  twist_command.twist.linear.y = trajectory_point.twist.y;
  twist_command.twist.linear.z = trajectory_point.twist.z;
  twist_command.twist.angular.x = 0.0;
  twist_command.twist.angular.y = 0.0;
  twist_command.twist.angular.z = 0.0;

  if (!processCommand(pose_command)) {
    return false;
  }
  if (!processCommand(twist_command)) {
    return false;
  }
  trajectory_point.position.x = pose_command.pose.position.x;
  trajectory_point.position.y = pose_command.pose.position.y;
  trajectory_point.position.z = pose_command.pose.position.z;
  trajectory_point.yaw_angle = as2::frame::getYawFromQuaternion(pose_command.pose.orientation);
  trajectory_point.twist.x = twist_command.twist.linear.x;
  trajectory_point.twist.y = twist_command.twist.linear.y;
  trajectory_point.twist.z = twist_command.twist.linear.z;
  trajectory_command.setpoints[0] = trajectory_point;
  return true;
}

}   // namespace as2_platform_multirotor_simulator
