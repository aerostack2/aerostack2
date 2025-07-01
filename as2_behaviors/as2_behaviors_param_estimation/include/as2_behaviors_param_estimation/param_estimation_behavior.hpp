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
 *  \file       param_estimation_behavior.hpp
 *  \brief      path_planner_behavior header file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

 #ifndef AS2_BEHAVIORS_PARAM_ESTIMATION__PARAM_ESTIMATION_BEHAVIOR_HPP_
 #define AS2_BEHAVIORS_PARAM_ESTIMATION__PARAM_ESTIMATION_BEHAVIOR_HPP_

 #include <string>
 #include <memory>

 #include <rclcpp/rclcpp.hpp>
 #include "as2_behavior/behavior_server.hpp"
 #include "as2_msgs/action/param_estimation.hpp"
 #include "as2_msgs/msg/thrust.hpp"
 #include "as2_core/names/topics.hpp"
 #include "std_msgs/msg/float64.hpp"
 #include "sensor_msgs/msg/imu.hpp"
 #include "as2_core/synchronous_service_client.hpp"
 #include "param_estimation/param_estimation.hpp"


class ParamEstimationBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::ParamEstimation>
{
public:
  explicit ParamEstimationBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ParamEstimationBehavior() {}

private:
  as2_msgs::action::ParamEstimation::Goal goal_;
  as2_msgs::action::ParamEstimation::Feedback feedback_;
  as2_msgs::action::ParamEstimation::Result result_;
  std::shared_ptr<ParamEstimation> param_estimation_lib;
  // Subscribers
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr comanded_thrust_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr param_pub_;

  // Client
  as2::SynchronousServiceClient<rcl_interfaces::srv::SetParameters>::SharedPtr
    set_parameters_client_;

  // Parameters
  float thrust_threshold_;
  float initial_mass_;
  float minimum_mass_;
  double estimated_mass_;
  float commanded_thrust_;
  double measured_az_;

  // Flags
  bool thrust_received_ = false;
  bool imu_received_ = false;
  bool behvaior_paused_ = false;

private:
  /**As2 Behavior methods */
  bool on_activate(std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::ParamEstimation::Goal> & goal,
    std::shared_ptr<as2_msgs::action::ParamEstimation::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::ParamEstimation::Result> & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;


  // Subscribers
  void comandedThrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // Publishers

  // Clients
  void updateMassParameter();
};
#endif // AS2_BEHAVIORS_PARAM_ESTIMATION__PARAM_ESTIMATION_BEHAVIOR_HPP_
