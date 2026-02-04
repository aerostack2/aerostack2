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
 *  \file       mass_estimation_behavior.hpp
 *  \brief      mass_estimation_behavior header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

#ifndef MASS_ESTIMATION_BEHAVIOR__MASS_ESTIMATION_BEHAVIOR_HPP_
#define MASS_ESTIMATION_BEHAVIOR__MASS_ESTIMATION_BEHAVIOR_HPP_

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
#include "as2_msgs/action/mass_estimation.hpp"

#include "mass_estimation_behavior/param_estimation.hpp"

namespace mass_estimation_behavior
{
class MassEstimationBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::MassEstimation>
{
public:
  explicit MassEstimationBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~MassEstimationBehavior() {}

private:
  std::shared_ptr<ParamEstimation> param_estimation_lib;

  // Subscribers
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr comanded_thrust_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_mass_estimation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_mass_filtered_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_mass_update_pub_;

  // Clients
  as2::SynchronousServiceClient<rcl_interfaces::srv::GetParameters>::SharedPtr
    get_parameters_client_;
  as2::SynchronousServiceClient<rcl_interfaces::srv::SetParameters>::SharedPtr
    set_parameters_client_;
  rcl_interfaces::msg::Parameter param_;

  // Parameters
  double mass_threshold_;
  double thrust_threshold_;
  double alpha_;
  size_t n_samples_ = 3;
  double mass_publish_interval_;
  double minimum_mass_;
  double maximum_mass_;
  double initial_mass_;
  std::string controler_node_;
  std::string mass_param_name_;

  // Debug
  std::string mass_update_topic_;
  std::string mass_filtered_topic_;
  std::string mass_estimation_topic_;

  // Callbacks data
  float last_commanded_thrust_ = 0.0;
  float next_commanded_thrust_ = 0.0;
  rclcpp::Time last_mass_update_time_;

  // Mass estimation data
  double estimated_mass_;
  double filtered_mass_;
  double last_filtered_mass_;
  std::vector<double> measured_az_stack_;

  // Flags
  bool thrust_received_ = false;
  bool behvaior_paused_ = false;
  bool mass_publish_ = true;

private:
  /**As2 Behavior methods */
  bool on_activate(
    std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> goal)
  override;

  bool on_modify(
    std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> goal)
  override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::MassEstimation::Goal> & goal,
    std::shared_ptr<as2_msgs::action::MassEstimation::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::MassEstimation::Result> & result_msg)
  override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;


  // Subscribers
  void comandedThrustCallback(const as2_msgs::msg::Thrust::SharedPtr thrust_msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

  // Clients
  void updateMassParameter();
};

}  // namespace mass_estimation_behavior
#endif  // MASS_ESTIMATION_BEHAVIOR__MASS_ESTIMATION_BEHAVIOR_HPP_
