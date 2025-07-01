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
 * @file param_estimation_behavior.cpp
 *
 * Param Estimation behavior file
 *
 * @authors Carmen De Rojas Pita-Romero
 */

 #include "as2_behaviors_param_estimation/param_estimation_behavior.hpp"


ParamEstimationBehavior::ParamEstimationBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::ParamEstimation>(
    "param_estimation_behavior", options)
{
  thrust_threshold_ = this->declare_parameter<double>("thrust_threshold", 0.1);
  initial_mass_ = this->declare_parameter<double>("initial_mass", 1.0);
  minimum_mass_ = this->declare_parameter<double>("minimum_mass", 0.5);
  param_estimation_lib = std::make_shared<ParamEstimation>(
    this->initial_mass_);
}

bool ParamEstimationBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> goal)
{
  as2_msgs::action::ParamEstimation::Goal new_goal = *goal;
  param_estimation_lib->set_threshold(new_goal.mass_threshold);
  comanded_thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    std::string(
      "/drone0/") + as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos,
    std::bind(&ParamEstimationBehavior::comandedThrustCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    std::string("/drone0/") + as2_names::topics::sensor_measurements::imu,
    as2_names::topics::sensor_measurements::qos,
    std::bind(&ParamEstimationBehavior::imuCallback, this, std::placeholders::_1));
  set_parameters_client_ =
    std::make_shared<as2::SynchronousServiceClient<rcl_interfaces::srv::SetParameters>>(
    "/drone0/controller_manager/set_parameters", this);
  return true;
}

bool ParamEstimationBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> goal)
{
  goal_ = *goal;
  RCLCPP_INFO(
    this->get_logger(), "ParamEstimationBehavior: Modifying mass threshold to: %f",
    goal_.mass_threshold);
  param_estimation_lib->set_threshold(goal_.mass_threshold);
  return true;
}

bool ParamEstimationBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ParamEstimationBehavior Deactivated");
  return true;
}

bool ParamEstimationBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ParamEstimationBehavior Paused");
  behvaior_paused_ = true;
  return true;
}

bool ParamEstimationBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "ParamEstimationBehavior Resumed");
  behvaior_paused_ = false;
  return true;
}

as2_behavior::ExecutionStatus ParamEstimationBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> & goal,
  std::shared_ptr<as2_msgs::action::ParamEstimation::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::ParamEstimation::Result> & result_msg)
{
  result_msg = std::make_shared<as2_msgs::action::ParamEstimation::Result>(result_);
  if (!behvaior_paused_) {
    if (thrust_received_ && imu_received_) {
      RCLCPP_INFO(
        this->get_logger(), "Thrust: %f, Measured az: %f",
        commanded_thrust_, measured_az_);
      param_estimation_lib->computeMass(commanded_thrust_, measured_az_);
      estimated_mass_ = this->param_estimation_lib->getEstimatedMass();
      RCLCPP_INFO(this->get_logger(), "Estimated mass: %f", estimated_mass_);
      // TODO: Compare the estimated mass with the minimum mass of the drone + a threshold
      if (estimated_mass_ < minimum_mass_) {
        RCLCPP_ERROR(
          this->get_logger(), "Estimated mass is negative, something went wrong.");
        result_.success = false;
        return as2_behavior::ExecutionStatus::FAILURE;

      }
      updateMassParameter();
      thrust_received_ = false;
      imu_received_ = false;
      result_.success = true;
      return as2_behavior::ExecutionStatus::RUNNING;

    }
  }
  result_.success = true;
  return as2_behavior::ExecutionStatus::RUNNING;

}

void ParamEstimationBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  if (state == as2_behavior::ExecutionStatus::SUCCESS ||
    state == as2_behavior::ExecutionStatus::ABORTED)
  {
    RCLCPP_INFO(this->get_logger(), "ParamEstimationBehavior Finished");
  }
  return;
}

void ParamEstimationBehavior::comandedThrustCallback(
  const as2_msgs::msg::Thrust::SharedPtr msg)
{
  auto thrust_msg = *msg;
  commanded_thrust_ = std::abs(thrust_msg.thrust);
  if (commanded_thrust_ > thrust_threshold_) {
    thrust_received_ = true;
  }
}

void ParamEstimationBehavior::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto imu_msg = *msg;
  measured_az_ = std::abs(imu_msg.linear_acceleration.z);
  imu_received_ = true;
}
void ParamEstimationBehavior::updateMassParameter()
{
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto respond = std::make_shared<rcl_interfaces::srv::SetParameters::Response>();
  rcl_interfaces::msg::Parameter param;
  param.name = "mass";
  rcl_interfaces::msg::ParameterValue value;
  value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  value.double_value = estimated_mass_;
  param.value = value;
  request->parameters.push_back(param);
  auto out = set_parameters_client_->sendRequest(request, respond, 3);
  if (out && respond.get()) {
    RCLCPP_INFO(
      this->get_logger(), "Mass parameter updated successfully to: %f",
      this->estimated_mass_);
  }
}
