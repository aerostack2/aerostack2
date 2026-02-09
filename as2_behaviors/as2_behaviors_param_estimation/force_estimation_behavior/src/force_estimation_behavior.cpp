// Copyright 2026 Universidad Politécnica de Madrid
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

/*!******************************************************************************
 *  \file       force_estimation_behavior.cpp
 *  \brief      force_estimation_behavior implementation file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

#include "force_estimation_behavior/force_estimation_behavior.hpp"

namespace force_estimation_behavior
{
ForceEstimationBehavior::ForceEstimationBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::ForceEstimation>(
    "ForceEstimationBehavior", options)
{
  // Read parameters
  controler_node_ = this->declare_parameter<std::string>("controller_node");
  force_param_name_ = this->declare_parameter<std::string>("force_param_name");
  mass_param_name_ = this->declare_parameter<std::string>("mass_param_name");
  alpha_ = this->declare_parameter<double>("alpha");
  n_samples_ = this->declare_parameter<int>("n_samples");
  last_filtered_error_ = this->declare_parameter<double>("initial_force_error");
  threshold_time_sync = this->declare_parameter<double>("threshold_time_sync");
  fz_filtered_error_ = this->declare_parameter<double>("fz_filtered_error");
  fz_update_error = this->declare_parameter<double>("fz_update_error");
  minimum_error_ = this->declare_parameter<double>("minimum_error");
  maximum_error_ = this->declare_parameter<double>("maximum_error");
  // Debug parameters
  force_error_topic_ = this->declare_parameter<std::string>("debug.force_error_topic");
  force_filtered_error_topic_ =
    this->declare_parameter<std::string>("debug.force_filtered_error_topic");
  force_update_error_topic_ =
    this->declare_parameter<std::string>("debug.force_update_error_topic");
  force_limited_error_topic_ =
    this->declare_parameter<std::string>("debug.force_limited_error_topic");

  threshold_time_sync_ = rclcpp::Duration::from_seconds(threshold_time_sync);
  published_force_error_ = 1.0 / fz_update_error;
  // INFO
  RCLCPP_INFO(this->get_logger(), "ForceEstimationBehavior Initialized");
  RCLCPP_INFO(
    this->get_logger(), "Controller node: %s", controler_node_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Force parameter name: %s", force_param_name_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Mass parameter name: %s", mass_param_name_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "alpha: %f", alpha_);
  RCLCPP_INFO(this->get_logger(), "n_samples: %ld", n_samples_);
  RCLCPP_INFO(
    this->get_logger(), "initial_force_error: %f", last_filtered_error_);
  RCLCPP_INFO(
    this->get_logger(), "threshold_time_sync: %f", threshold_time_sync);
  RCLCPP_INFO(
    this->get_logger(), "fz_filtered_error: %f", fz_filtered_error_);
  RCLCPP_INFO(
    this->get_logger(), "fz_update_error: %f", fz_update_error);
  RCLCPP_INFO(
    this->get_logger(), "minimum_error: %f", minimum_error_);
  RCLCPP_INFO(
    this->get_logger(), "maximum_error: %f", maximum_error_);
}
bool ForceEstimationBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> goal)
{
  as2_msgs::action::ForceEstimation::Goal new_goal = *goal;
  // Subscribers
  commanded_thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos,
    std::bind(&ForceEstimationBehavior::commandedThrustCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    as2_names::topics::sensor_measurements::imu,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&ForceEstimationBehavior::imuCallback, this, std::placeholders::_1));


  // Server clients
  if (!controler_node_.empty() && !force_param_name_.empty()) {
    set_parameters_client_ =
      std::make_shared<as2::SynchronousServiceClient<rcl_interfaces::srv::SetParameters>>(
      controler_node_ + std::string("/set_parameters"), this);
    get_parameters_client_ =
      std::make_shared<as2::SynchronousServiceClient<rcl_interfaces::srv::GetParameters>>(
      controler_node_ + std::string("/get_parameters"), this);
    RCLCPP_INFO(
      this->get_logger(), "GetParameters client created for node %s",
      controler_node_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "SetParameters client created for node %s",
      controler_node_.c_str());

    param_.name = force_param_name_;
    rcl_interfaces::msg::ParameterValue value;
    value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param_.value = value;
  }
  if (set_parameters_client_ == nullptr) {
    RCLCPP_WARN(
      this->get_logger(),
      "Force error update is disabled");
  }

  // Debug publishers
  if (!force_error_topic_.empty()) {
    force_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      force_error_topic_, 1);
    RCLCPP_INFO(
      this->get_logger(), "Debugging estimated force in: %s",
      force_error_topic_.c_str());
  }

  if (!force_filtered_error_topic_.empty()) {
    force_filtered_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      force_filtered_error_topic_, 1);
    RCLCPP_INFO(
      this->get_logger(), "Debugging filtered force in: %s", force_filtered_error_topic_.c_str());
  }
  if (!force_update_error_topic_.empty()) {
    force_update_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      force_update_error_topic_, 1);
    RCLCPP_INFO(
      this->get_logger(), "Debugging updated force in: %s", force_update_error_topic_.c_str());
  }
  if (!force_limited_error_topic_.empty()) {
    force_limited_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      force_limited_error_topic_, 1);
    RCLCPP_INFO(
      this->get_logger(), "Debugging limited force in: %s", force_limited_error_topic_.c_str());
  }
  // Force estimation library
  force_estimation_lib = std::make_shared<ForceEstimation>(
    alpha_, n_samples_);

  // Ask for the current mass parameter
  // TODO(CARMEN)
  // if (get_parameters_client_ != nullptr) {
  //   auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  //   auto respond = std::make_shared<rcl_interfaces::srv::GetParameters::Response>();
  //   request->names.push_back(mass_param_name_);
  //   auto out = get_parameters_client_->sendRequest(request, respond, 3);
  //   if (out) {
  //     if (!respond->values.empty()) {
  //       if (respond->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
  //         force_estimation_lib->setMass(respond->values[0].double_value);
  //         RCLCPP_INFO(
  //           this->get_logger(), "Mass parameter %s asked. Setting to: %f",
  //           mass_param_name_.c_str(), respond->values[0].double_value);
  //       }
  //     }
  //   }
  // } else {
  //   RCLCPP_WARN(
  //     this->get_logger(), "GetParameters client not available. Waiting for it to be available...");
  // }

  mass_ = 1.0;
  last_force_error_update_time_ = this->now();
  return true;
}

bool ForceEstimationBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> goal)
{
  return true;
}
bool ForceEstimationBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ForceEstimationBehavior Deactivated");
  return true;
}
bool ForceEstimationBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ForceEstimationBehavior Paused");
  return true;
}

bool ForceEstimationBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ForceEstimationBehavior Resumed");
  return true;
}
as2_behavior::ExecutionStatus ForceEstimationBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> & goal,
  std::shared_ptr<as2_msgs::action::ForceEstimation::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::ForceEstimation::Result> & result_msg)
{
  if (!first_thrust_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for the first thrust command to be received...");
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  if (imu_time_.seconds() <
    thrust_time_.seconds() + threshold_time_sync_.seconds())
  {
    double current_force_error;
    double a_z_mean = std::abs(force_estimation_lib->computedMeanFromVector(measured_az_stack_));
    current_force_error = force_estimation_lib->computeThrustError(
      mass_, a_z_mean,
      thrust_comanded_msg_);
    estimated_thrust_error_vector_.push_back(current_force_error);
    measured_az_stack_.clear();
    if (force_error_pub_ != nullptr) {
      std_msgs::msg::Float64 error_msg;
      error_msg.data = current_force_error;
      force_error_pub_->publish(error_msg);
    }
    if (!filter_force_error_timer_) {
      printf("Creating filter thrust error timer\n");
      filter_force_error_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0f / fz_filtered_error_),
        std::bind(&ForceEstimationBehavior::filterForceError, this)
      );
    }
  }
  const auto current_time = this->now();
  if ((current_time - last_force_error_update_time_).seconds() >= published_force_error_) {
    updateForceParameter();
    last_force_error_update_time_ = current_time;
  }
  return as2_behavior::ExecutionStatus::RUNNING;
}

void ForceEstimationBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    RCLCPP_INFO(this->get_logger(), "ForceEstimationBehavior Finished");
  }
  // Cleanup Subscribers
  commanded_thrust_sub_.reset();
  imu_sub_.reset();

  // Cleanup Publishers
  force_error_pub_.reset();
  force_filtered_error_pub_.reset();
  force_update_error_pub_.reset();
  force_limited_error_pub_.reset();

  // Cleanup Clients
  get_parameters_client_.reset();
  set_parameters_client_.reset();

  // Cleanup Force Error Estimator
  force_estimation_lib.reset();

  // Cleanup Timers
  filter_force_error_timer_.reset();

  // Reset variables
  measured_az_stack_.clear();
  estimated_thrust_error_vector_.clear();
  first_thrust_ = false;
  force_error_ = 0.0;
  return;
}
void ForceEstimationBehavior::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (!first_thrust_) {
    return;
  }
  measured_az_stack_.push_back(imu_msg->linear_acceleration.z);
  imu_time_ = rclcpp::Time(imu_msg->header.stamp);

}
void ForceEstimationBehavior::commandedThrustCallback(
  const as2_msgs::msg::Thrust::SharedPtr thrust_msg)
{
  first_thrust_ = true;
  thrust_comanded_msg_ = thrust_msg->thrust;
  thrust_time_ = rclcpp::Time(thrust_msg->header.stamp);
}

void ForceEstimationBehavior::filterForceError()
{
  RCLCPP_INFO(this->get_logger(), "Filtering thrust error");
  double mean_n_samples_thrust_error = force_estimation_lib->computedMeanFromNSamples(
    estimated_thrust_error_vector_);
  estimated_thrust_error_vector_.clear();
  double filtered_error = force_estimation_lib->lowPassFiltered(
    mean_n_samples_thrust_error,
    last_filtered_error_);
  last_filtered_error_ = filtered_error;
  if (force_filtered_error_pub_ != nullptr) {
    std_msgs::msg::Float64 filtered_msg;
    filtered_msg.data = filtered_error;
    force_filtered_error_pub_->publish(filtered_msg);
  }
  if (filtered_error < minimum_error_) {
    RCLCPP_ERROR(
      this->get_logger(), "Force error %f is under %f", filtered_error,
      minimum_error_);
    force_error_ = minimum_error_;
  } else if (filtered_error > maximum_error_) {
    RCLCPP_ERROR(
      this->get_logger(), "Force error %f is over %f", filtered_error, maximum_error_);
    force_error_ = maximum_error_;
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Force error %f is between %f and %f. Setting it to: %f",
      filtered_error, minimum_error_, maximum_error_, filtered_error);
    force_error_ = filtered_error;
  }
  if (force_limited_error_pub_ != nullptr) {
    std_msgs::msg::Float64 limited_msg;
    limited_msg.data = force_error_;
    force_limited_error_pub_->publish(limited_msg);
  }
}
void ForceEstimationBehavior::updateForceParameter()
{
  if (set_parameters_client_ != nullptr) {
    // Set controller mass
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto respond = std::make_shared<rcl_interfaces::srv::SetParameters::Response>();
    param_.value.double_value = force_error_;
    request->parameters.push_back(param_);
    auto out = set_parameters_client_->sendRequest(request, respond, 3);
    if (out) {
      if (!respond->results.empty()) {
        if (respond->results[0].successful) {
          RCLCPP_DEBUG(
            this->get_logger(), "Error force parameter %s updated to: %f",
            force_param_name_.c_str(), force_error_);
          if (force_update_error_pub_ != nullptr) {
            std_msgs::msg::Float64 debug_force;
            debug_force.data = force_error_;
            force_update_error_pub_->publish(debug_force);
          }
          if (force_update_error_pub_ != nullptr) {
            std_msgs::msg::Float64 debug_force;
            debug_force.data = force_error_;
            force_update_error_pub_->publish(debug_force);
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Force parameter %s failed to be updated to: %f",
            force_param_name_.c_str(), force_error_);
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Force parameter %s could not be updated to: %f",
          force_param_name_.c_str(), force_error_);
      }
    }
  }
}
}  // namespace force_estimation_behavior
