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

/*!*******************************************************************************************
 *  \file       force_estimation_behavior.hpp
 *  \brief      Declares the force estimation behavior.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************************/

#ifndef FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_BEHAVIOR_HPP_
#define FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_core/synchronous_service_client.hpp>
#include <as2_behavior/behavior_server.hpp>

#include <as2_msgs/msg/thrust.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "as2_msgs/action/force_estimation.hpp"

#include "force_estimation_behavior/force_estimation.hpp"


namespace force_estimation_behavior
{
class ForceEstimationBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::ForceEstimation>
{
public:
  explicit ForceEstimationBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ForceEstimationBehavior() {}

private:
  std::shared_ptr<ForceEstimation> force_estimation_lib;
  double force_error_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr commanded_thrust_sub_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_filtered_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_limited_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_mean_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_measured_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_update_error_pub_;


  // Clients
  as2::SynchronousServiceClient<rcl_interfaces::srv::GetParameters>::SharedPtr
    get_parameters_client_;
  as2::SynchronousServiceClient<rcl_interfaces::srv::SetParameters>::SharedPtr
    set_parameters_client_;
  rcl_interfaces::msg::Parameter param_;

  // Parameters
  std::string controler_node_;
  std::string force_param_name_;
  std::string mass_param_name_;
  double alpha_;
  size_t n_samples_;
  double threshold_time_sync_;
  double fz_update_error_;
  double minimum_error_;
  double maximum_error_;

  // Internal variables
  std::vector<double> measured_az_stack_;
  as2_msgs::msg::Thrust thrust_comanded_msg_;
  sensor_msgs::msg::Imu imu_msg_;

  // Internal callbacks
  rclcpp::TimerBase::SharedPtr filter_force_error_timer_;

  // Debug
  std::string force_error_topic_;
  std::string force_filtered_error_topic_;
  std::string imu_mean_topic_;
  std::string force_measured_topic_;
  std::string force_limited_error_topic_;
  std::string force_update_error_topic_;


  // Flags
  bool first_thrust_ = false;

private:
  /**As2 Behavior methods */
  bool on_activate(
    std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> goal)
  override;

  bool on_modify(
    std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> goal)
  override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::ForceEstimation::Goal> & goal,
    std::shared_ptr<as2_msgs::action::ForceEstimation::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::ForceEstimation::Result> & result_msg)
  override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

  // Suscriber callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void commandedThrustCallback(const as2_msgs::msg::Thrust::SharedPtr thrust_msg);

  // Timer
  void filterForceError();
  // Client
  void updateForceParameter();
};
}  // namespace force_estimation_behavior
#endif  // FORCE_ESTIMATION_BEHAVIOR__FORCE_ESTIMATION_BEHAVIOR_HPP_
