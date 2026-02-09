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

/*!******************************************************************************
 *  \file       mass_estimation_behavior.cpp
 *  \brief      mass_estimation_behavior implementation file.
 *  \authors    Carmen De Rojas Pita-Romero
 *              Rafael Perez-Segui
 ********************************************************************************/

#include "mass_estimation_behavior/mass_estimation_behavior.hpp"

namespace mass_estimation_behavior
{
MassEstimationBehavior::MassEstimationBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::MassEstimation>(
    "MassEstimationBehavior", options)
{
  mass_threshold_ = this->declare_parameter<double>("mass_threshold");
  thrust_threshold_ = this->declare_parameter<double>("thrust_threshold");
  alpha_ = this->declare_parameter<double>("alpha");
  int n_samples = this->declare_parameter<int>("n_samples");
  if (n_samples > 0) {
    n_samples_ = static_cast<size_t>(n_samples);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "n_samples parameter must be a positive integer. Setting to 1.");
    n_samples_ = 1;
  }
  double mass_fz = this->declare_parameter<double>("mass_fz");
  if (mass_fz <= 0) {
    RCLCPP_WARN(
      this->get_logger(), "mass_fz parameter must be a positive number. Setting to 1 Hz.");
    mass_fz = 1.0;
  }
  mass_publish_interval_ = 1.0 / mass_fz;
  minimum_mass_ = this->declare_parameter<double>("minimum_mass");
  maximum_mass_ = this->declare_parameter<double>("maximum_mass");

  controler_node_ = this->declare_parameter<std::string>("controller_node");
  mass_param_name_ = this->declare_parameter<std::string>("mass_param_name");
  initial_mass_ = this->declare_parameter<double>("initial_mass");
  mass_estimation_topic_ = this->declare_parameter<std::string>(
    "debug.mass_estimation_topic", "");
  mass_filtered_topic_ = this->declare_parameter<std::string>(
    "debug.mass_filtered_topic", "");

  mass_update_topic_ = this->declare_parameter<std::string>(
    "debug.mass_update_topic", "");

  // Info all params
  RCLCPP_INFO(this->get_logger(), "Mass threshold: %f", mass_threshold_);
  RCLCPP_INFO(this->get_logger(), "Thrust threshold: %f", thrust_threshold_);
  RCLCPP_INFO(this->get_logger(), "Mass update frequency: %f", mass_fz);
  RCLCPP_INFO(this->get_logger(), "Minimum mass: %f", minimum_mass_);
  RCLCPP_INFO(this->get_logger(), "Maximum mass: %f", maximum_mass_);
  RCLCPP_INFO(this->get_logger(), "Controller node: %s", controler_node_.c_str());
  RCLCPP_INFO(this->get_logger(), "Mass parameter name: %s", mass_param_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Initial mass: %f", initial_mass_);
  RCLCPP_INFO(this->get_logger(), "Mass estimation topic: %s", mass_estimation_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Mass filtered topic: %s", mass_filtered_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Mass update topic: %s", mass_update_topic_.c_str());
}

bool MassEstimationBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> goal)
{
  as2_msgs::action::MassEstimation::Goal new_goal = *goal;

  // Subscribers
  comanded_thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos,
    std::bind(&MassEstimationBehavior::comandedThrustCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    as2_names::topics::sensor_measurements::imu,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&MassEstimationBehavior::imuCallback, this, std::placeholders::_1));

  // Server clients
  if (!controler_node_.empty() && !mass_param_name_.empty()) {
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

    param_.name = mass_param_name_;
    rcl_interfaces::msg::ParameterValue value;
    value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param_.value = value;
  }

  // Publishers
  if (!mass_estimation_topic_.empty()) {
    debug_mass_estimation_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      mass_estimation_topic_, 1);
    RCLCPP_INFO(
      this->get_logger(), "Debugging estimated mass in: %s",
      mass_estimation_topic_.c_str());
  }

  if (!mass_filtered_topic_.empty()) {
    debug_mass_filtered_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      mass_filtered_topic_, 1);
    RCLCPP_INFO(this->get_logger(), "Debugging filtered mass in: %s", mass_filtered_topic_.c_str());
  }
  if (!mass_update_topic_.empty()) {
    debug_mass_update_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      mass_update_topic_, 1);
    RCLCPP_INFO(this->get_logger(), "Debugging updated mass in: %s", mass_update_topic_.c_str());
  }

  // Mass estimator
  param_estimation_lib = std::make_shared<ParamEstimation>(
    initial_mass_, thrust_threshold_, alpha_, n_samples_);
  param_estimation_lib->set_threshold(mass_threshold_);

  // Ask for the current mass parameter
  if (get_parameters_client_ != nullptr) {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    auto respond = std::make_shared<rcl_interfaces::srv::GetParameters::Response>();
    request->names.push_back(mass_param_name_);
    auto out = get_parameters_client_->sendRequest(request, respond, 3);
    if (out) {
      if (!respond->values.empty()) {
        if (respond->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
          initial_mass_ = respond->values[0].double_value;
          RCLCPP_INFO(
            this->get_logger(), "Mass parameter %s asked. Setting to: %f",
            mass_param_name_.c_str(), initial_mass_);
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Mass parameter %s is not a double. Setting to default: %f",
            mass_param_name_.c_str(), initial_mass_);
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Mass parameter %s not found. Setting to default: %f",
          mass_param_name_.c_str(), initial_mass_);
      }
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(), "GetParameters client not available. Setting mass to default: %f",
      initial_mass_);
  }

  // Reset data
  measured_az_stack_.clear();
  estimated_mass_ = initial_mass_;
  filtered_mass_ = initial_mass_;
  last_filtered_mass_ = initial_mass_;
  last_mass_update_time_ = this->now();
  thrust_received_ = false;
  behvaior_paused_ = false;

  if (set_parameters_client_ == nullptr) {
    RCLCPP_WARN(
      this->get_logger(),
      "Mass update is disabled");
  }

  return true;
}

bool MassEstimationBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> goal)
{
  return true;
}

bool MassEstimationBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "MassEstimationBehavior Deactivated");
  return true;
}

bool MassEstimationBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "MassEstimationBehavior Paused");
  behvaior_paused_ = true;
  return true;
}

bool MassEstimationBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "MassEstimationBehavior Resumed");
  behvaior_paused_ = false;
  return true;
}

as2_behavior::ExecutionStatus MassEstimationBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> & goal,
  std::shared_ptr<as2_msgs::action::MassEstimation::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::MassEstimation::Result> & result_msg)
{
  if (!thrust_received_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for thrust commands over %s topic...", as2_names::topics::actuator_command::thrust);
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  // Estimate the mass
  param_estimation_lib->computeMass(last_commanded_thrust_, measured_az_stack_);
  estimated_mass_ = this->param_estimation_lib->getEstimatedMass();
  measured_az_stack_.clear();
  auto & clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), clk, 5000, "Estimated mass: %f", estimated_mass_);
  if (debug_mass_estimation_pub_ != nullptr) {
    std_msgs::msg::Float64 debug_mass;
    debug_mass.data = estimated_mass_;
    debug_mass_estimation_pub_->publish(debug_mass);
  }

  if (estimated_mass_ < minimum_mass_) {
    RCLCPP_ERROR(
      this->get_logger(), "Estimated mass %f is under %f", estimated_mass_, minimum_mass_);
    estimated_mass_ = minimum_mass_;
  } else if (estimated_mass_ > maximum_mass_) {
    RCLCPP_ERROR(
      this->get_logger(), "Estimated mass %f is over %f", estimated_mass_, maximum_mass_);
    estimated_mass_ = maximum_mass_;
  }

  const auto current_time = this->now();
  if ((current_time - last_mass_update_time_).seconds() >= mass_publish_interval_) {
    updateMassParameter();
    last_mass_update_time_ = current_time;
  }

  thrust_received_ = false;
  result_msg->success = true;
  return as2_behavior::ExecutionStatus::RUNNING;
}

void MassEstimationBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    RCLCPP_INFO(this->get_logger(), "MassEstimationBehavior Finished");
  }
  // Cleanup Subscribers
  comanded_thrust_sub_.reset();
  imu_sub_.reset();

  // Cleanup Publishers
  debug_mass_estimation_pub_.reset();
  debug_mass_filtered_pub_.reset();
  debug_mass_update_pub_.reset();

  // Cleanup Clients
  get_parameters_client_.reset();
  set_parameters_client_.reset();

  // Cleanup Mass Estimator
  param_estimation_lib.reset();
  return;
}

void MassEstimationBehavior::comandedThrustCallback(
  const as2_msgs::msg::Thrust::SharedPtr thrust_msg)
{
  last_commanded_thrust_ = next_commanded_thrust_;
  next_commanded_thrust_ = thrust_msg->thrust;
  if (last_commanded_thrust_ > thrust_threshold_) {
    thrust_received_ = true;
  }
}

void MassEstimationBehavior::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (last_commanded_thrust_ > thrust_threshold_) {
    measured_az_stack_.push_back(imu_msg->linear_acceleration.z);
  }
}
void MassEstimationBehavior::updateMassParameter()
{
  if (set_parameters_client_ != nullptr) {
    // Set controller mass
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto respond = std::make_shared<rcl_interfaces::srv::SetParameters::Response>();
    param_.value.double_value = estimated_mass_;
    request->parameters.push_back(param_);
    auto out = set_parameters_client_->sendRequest(request, respond, 3);
    if (out) {
      if (!respond->results.empty()) {
        if (respond->results[0].successful) {
          RCLCPP_DEBUG(
            this->get_logger(), "Mass parameter %s updated to: %f",
            mass_param_name_.c_str(), estimated_mass_);
          if (debug_mass_update_pub_ != nullptr) {
            std_msgs::msg::Float64 debug_mass;
            debug_mass.data = estimated_mass_;
            debug_mass_update_pub_->publish(debug_mass);
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Mass parameter %s failed to be updated to: %f",
            mass_param_name_.c_str(), estimated_mass_);
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Mass parameter %s could not be updated to: %f",
          mass_param_name_.c_str(), estimated_mass_);
      }
    }
  }

  if (debug_mass_filtered_pub_ != nullptr) {
    std_msgs::msg::Float64 debug_mass;
    debug_mass.data = estimated_mass_;
    debug_mass_filtered_pub_->publish(debug_mass);
  }
}
}   //  namespace mass_estimation_behavior
