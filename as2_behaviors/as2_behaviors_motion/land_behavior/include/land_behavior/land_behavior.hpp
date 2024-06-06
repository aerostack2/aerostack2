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

/**
 * @file land_behavior.hpp
 *
 * Land behavior class header file
 *
 * @authors Rafael Perez-Segui
 *          Pedro Arias Pérez
 */

#ifndef LAND_BEHAVIOR__LAND_BEHAVIOR_HPP_
#define LAND_BEHAVIOR__LAND_BEHAVIOR_HPP_

#include <string>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"
#include "land_base.hpp"
#include "rclcpp/rclcpp.hpp"

class LandBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Land>
{
public:
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;
  using PSME = as2_msgs::msg::PlatformStateMachineEvent;

  explicit LandBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~LandBehavior();

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

  bool sendEventFSME(const int8_t _event);

  bool sendDisarm();

  bool process_goal(
    std::shared_ptr<const as2_msgs::action::Land::Goal> goal,
    as2_msgs::action::Land::Goal & new_goal);

  bool on_activate(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override;
  bool on_modify(std::shared_ptr<const as2_msgs::action::Land::Goal> goal) override;
  bool on_deactivate(const std::shared_ptr<std::string> & message) override;
  bool on_pause(const std::shared_ptr<std::string> & message) override;
  bool on_resume(const std::shared_ptr<std::string> & message) override;
  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::Land::Goal> & goal,
    std::shared_ptr<as2_msgs::action::Land::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::Land::Result> & result_msg) override;
  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  std::string base_link_frame_id_;
  std::shared_ptr<pluginlib::ClassLoader<land_base::LandBase>> loader_;
  std::shared_ptr<land_base::LandBase> land_plugin_;
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::chrono::nanoseconds tf_timeout;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
    platform_land_cli_;
  as2::SynchronousServiceClient<std_srvs::srv::SetBool>::SharedPtr platform_disarm_cli_;
};

#endif  // LAND_BEHAVIOR__LAND_BEHAVIOR_HPP_
