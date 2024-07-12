// Copyright 2024 Universidad Politécnica de Madrid
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
 *  \file       spin_behavior.cpp
 *  \brief      Spin behavior implementation file.
 *  \authors    Tomás Sánchez Villauenga
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "spin_behavior.hpp"

namespace spin_behavior
{

SpinBehavior::SpinBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::Spin>("SpinBehavior", options),
  tf_handler_(this)
{
    // Spin angle (yaw)
    try {
    this->declare_parameter<double>("spin_angle");
  } catch (const rclcpp::ParameterTypeException& e) {
    // RCLCPP_FATAL(this->get_logger(),
                //  "Launch argument <spin_angle> not defined or "
                //  "malformed: %s",
                //  e.what());
    this->~SpinBehavior();
  }
  // Spin speed
  try {
    this->declare_parameter<double>("spin_speed");
  } catch (const rclcpp::ParameterTypeException& e) {
    // RCLCPP_FATAL(this->get_logger(),
                //  "Launch argument <spin_speed> not defined or "
                //  "malformed: %s",
                //  e.what());
    this->~SpinBehavior();
  }
  
  // Behavior name to publish commands
  this->declare_parameter<std::string>("behavior_name", "spin_bahavior");
  this->get_parameter("behavior_name", behavior_name_);

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  // Set internal variables frame ids
  current_goal_position_.header.frame_id = base_link_frame_id_;

  // Angle threshold
  this->declare_parameter<double>("angle_threshold", 0.1);
  this->get_parameter("angle_threshold", angle_threshold);

  // RCLCPP_INFO(
    // this->get_logger(), "SpinBehavior created for behavior name %s in frame %s",
    // behavior_name_.c_str(), base_link_frame_id_.c_str());

}

bool SpinBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::Spin::Goal> goal)
{
   // Process goal
  if (desired_goal_position_.header.frame_id == "") {
    desired_goal_position_.header.frame_id = base_link_frame_id_;
    // RCLCPP_INFO(
      // this->get_logger(), "Goal frame id not set, using base_link frame id %s",
      // desired_goal_position_.header.frame_id.c_str());
  }

  if (goal->speed < 0.0f) {
    // RCLCPP_ERROR(this->get_logger(), "SpinBehavior: Invalid spin speed");
    return false;
  }

  desired_goal_position_.yaw = goal->yaw;
  desired_goal_position_.speed = goal->speed;

  // RCLCPP_INFO(
    // this->get_logger(),
    // "SpinBehavior: desired yaw=%f and speed=%f",
    // desired_goal_position_.yaw, desired_goal_position_.speed);

  goal_init_time_ = this->now();
  // RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

bool SpinBehavior::on_modify(std::shared_ptr<const as2_msgs::action::Spin::Goal> goal)
{
  // RCLCPP_INFO(this->get_logger(), "Goal modified not available for this behavior");
  return false;
}

bool SpinBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "SpinBehavior cancelled");
  return true;
}

bool SpinBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "SpinBehavior paused");
  return true;
}

bool SpinBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  // RCLCPP_INFO(this->get_logger(), "SpinBehavior resumed");
  goal_init_time_ = this->now();
  return true;
}

as2_behavior::ExecutionStatus SpinBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::Spin::Goal> & goal,
  std::shared_ptr<as2_msgs::action::Spin::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::Spin::Result> & result_msg)
{
  // Check timeout
  auto behavior_time = this->now() - goal_init_time_;
  if (behavior_time.seconds() > behavior_timeout_.seconds()) {
    // RCLCPP_ERROR(this->get_logger(), "SpinBehavior: goal timeout");
    result_msg->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (!update_spin_state()) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (check_finished()) {
    result_msg->success = true;
    // RCLCPP_INFO(this->get_logger(), "Spin goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // Feedback
  feedback_msg->header.stamp = this->now();
  feedback_msg->header.frame_id = current_goal_position_.header.frame_id;    // base_link_frame_id_
  feedback_msg->current_yaw = current_spin_position;
  feedback_msg->current_speed = current_spin_speed;

  return as2_behavior::ExecutionStatus::RUNNING;
}




void SpinBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  // RCLCPP_INFO(this->get_logger(), "SpinBehavior execution ended");
}


bool SpinBehavior::update_spin_state() 
{
  try {
    // tf2::TimePoint time = tf2::TimePointZero;
    rclcpp::Time time = this->now();
  } catch (const std::exception & e) {
    // RCLCPP_ERROR(
      // this->get_logger(),
      // "SpinBehavior: could not get current spin information");
    return false;
  }
}

bool SpinBehavior::update_spin_angle(const geometry_msgs::msg::Twist &msg) 
{
  current_spin_position = msg.angular.z;
}

bool SpinBehavior::update_spin_speed(const geometry_msgs::msg::Twist &msg) 
{
  current_spin_speed = msg.angular.z;
}

bool SpinBehavior::check_finished()
{
  // Check angle between vectors:
  // - desired_goal_position (in gimbal frame)
  // - current_goal_position (in gimbal frame)

  if (desired_goal_position_.yaw - current_spin_position < angle_threshold && desired_goal_position_.yaw - current_spin_position > -angle_threshold) {
    // RCLCPP_INFO(
      // this->get_logger(), "SpinBehavior: goal reached, angle between vectors %f", desired_goal_position_.z - current_spin_position );
    return true;
  }

  return false;
}

}  // namespace spin_behavior