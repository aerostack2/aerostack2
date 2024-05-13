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

/**
 * @file as2_platform_multirotor_simulator_fly_gtest.cpp
 *
 * MultirotorSimulatorPlatform gtets
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp"

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <as2_core/node.hpp>
#include <as2_core/names/services.hpp>
#include <as2_msgs/msg/platform_state_machine_event.hpp>

namespace as2_platform_multirotor_simulator
{

class MultirotorSimulatorPlatformTestNode : public rclcpp::Node
{
public:
  MultirotorSimulatorPlatformTestNode(
    const std::string & name_space)
  : rclcpp::Node("multirotor_simulator_test", name_space)
  {
    arm_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
      as2_names::services::platform::set_arming_state);

    offboard_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
      as2_names::services::platform::set_offboard_mode);

    state_machine_srv_cli_ = this->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(
      as2_names::services::platform::set_platform_state_machine_event);

    takeoff_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
      as2_names::services::platform::takeoff);

    land_srv_cli_ = this->create_client<std_srvs::srv::SetBool>(
      as2_names::services::platform::land);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::ground_truth::pose,
      as2_names::topics::ground_truth::qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
      });

    current_pose_.pose.position.z = -33.0;
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture setArmingState(bool arm)
  {
    RCLCPP_INFO(this->get_logger(), "Setting arming state to %s", arm ? "true" : "false");
    auto request =
      std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = arm;
    return callService<std_srvs::srv::SetBool>(request, arm_srv_cli_);
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture setOffboardControl(bool offboard)
  {
    RCLCPP_INFO(this->get_logger(), "Setting offboard control to %s", offboard ? "true" : "false");
    auto request =
      std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = offboard;
    return callService<std_srvs::srv::SetBool>(request, offboard_srv_cli_);
  }

  rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedFuture
  setPlatformStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent request_state)
  {
    RCLCPP_INFO(
      this->get_logger(), "Setting platform state machine event to %d", request_state.event);
    auto request = std::make_shared<as2_msgs::srv::SetPlatformStateMachineEvent::Request>();
    request->event = request_state;
    return callService<as2_msgs::srv::SetPlatformStateMachineEvent>(
      request,
      state_machine_srv_cli_);
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture takeoff()
  {
    RCLCPP_INFO(this->get_logger(), "Taking off");
    auto request =
      std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    return callService<std_srvs::srv::SetBool>(request, takeoff_srv_cli_);
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture land()
  {
    RCLCPP_INFO(this->get_logger(), "Landing");
    auto request =
      std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    return callService<std_srvs::srv::SetBool>(request, land_srv_cli_);
  }

  double getTime()
  {
    return this->now().seconds();
  }

  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    return current_pose_;
  }

private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr offboard_srv_cli_;
  rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr state_machine_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr takeoff_srv_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr land_srv_cli_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::PoseStamped current_pose_;

  template<typename ServiceType>
  typename rclcpp::Client<ServiceType>::SharedFuture callService(
    typename ServiceType::Request::SharedPtr request,
    typename rclcpp::Client<ServiceType>::SharedPtr srv_cli)
  {
    while (!srv_cli->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting.");
        return typename rclcpp::Client<ServiceType>::SharedFuture();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Service not available, waiting again...");
    }

    auto result_future = srv_cli->async_send_request(request);
    return result_future;
  }
};

class MultirotorSimulatorPlatformTest : public testing::Test
{
protected:
  std::shared_ptr<MultirotorSimulatorPlatform> platform_node;
  std::shared_ptr<MultirotorSimulatorPlatformTestNode> test_node;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp()
  {
    platform_node = get_node("multirotor_simulator");
    test_node = std::make_shared<MultirotorSimulatorPlatformTestNode>(
      "multirotor_simulator");

    executor.add_node(platform_node);
    executor.add_node(test_node);
  }

  void TearDown()
  {
    executor.cancel();
    executor.remove_node(platform_node);
    executor.remove_node(test_node);
    platform_node.reset();
    test_node.reset();
  }

private:
  std::shared_ptr<MultirotorSimulatorPlatform> get_node(
    const std::string & name_space = "multirotor_simulator")
  {
    const std::string package_path =
      ament_index_cpp::get_package_share_directory("as2_platform_multirotor_simulator");
    const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
    const std::string platform_config_file = package_path + "/config/platform_config_file.yaml";
    const std::string simulation_config = package_path + "/config/simulation_config.yaml";
    const std::string uav_config = package_path + "/config/uav_config.yaml";

    std::vector<std::string> node_args = {
      "--ros-args",
      "-r",
      "__ns:=/" + name_space,
      "-p",
      "namespace:=" + name_space,
      "-p",
      "control_modes_file:=" + control_modes_config_file,
      "--params-file",
      platform_config_file,
      "--params-file",
      simulation_config,
      "--params-file",
      uav_config,
    };

    rclcpp::NodeOptions node_options;
    node_options.arguments(node_args);

    return std::make_shared<MultirotorSimulatorPlatform>(node_options);
  }
};

TEST_F(MultirotorSimulatorPlatformTest, TakeoffAndLand) {
  // Spin for 2 seconds to allow the platform to initialize
  double initial_time = test_node->getTime();
  double current_time = test_node->getTime();
  while (current_time - initial_time < 2.0) {
    executor.spin_some();
    current_time = test_node->getTime();
  }

  // Check that the platform is in the ground
  auto current_pose = test_node->getCurrentPose();
  double epsilon = 1e-3;
  ASSERT_NEAR(current_pose.pose.position.x, 0.0, epsilon);

  // Arm
  auto arm_future = test_node->setArmingState(true);
  executor.spin_some();
  auto arm_result = executor.spin_until_future_complete(arm_future);
  ASSERT_EQ(arm_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(arm_future.get()->success, true);

  // Offboard
  auto offboard_future = test_node->setOffboardControl(true);
  executor.spin_some();
  auto offboard_result = executor.spin_until_future_complete(offboard_future);
  ASSERT_EQ(offboard_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(offboard_future.get()->success, true);

  // Set transition from LANDED to TAKING_OFF
  as2_msgs::msg::PlatformStateMachineEvent taking_off_event;
  taking_off_event.event = as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF;
  auto state_machine_future = test_node->setPlatformStateMachineEvent(taking_off_event);
  executor.spin_some();
  auto state_machine_result = executor.spin_until_future_complete(state_machine_future);
  ASSERT_EQ(state_machine_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(state_machine_future.get()->success, true);

  // Takeoff
  auto takeoff_future = test_node->takeoff();
  executor.spin_some();
  auto takeoff_result = executor.spin_until_future_complete(takeoff_future);
  ASSERT_EQ(takeoff_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(takeoff_future.get()->success, true);

  // Set transition from FLY to LANDING
  as2_msgs::msg::PlatformStateMachineEvent landing_event;
  landing_event.event = as2_msgs::msg::PlatformStateMachineEvent::LAND;
  state_machine_future = test_node->setPlatformStateMachineEvent(landing_event);
  executor.spin_some();
  state_machine_result = executor.spin_until_future_complete(state_machine_future);
  ASSERT_EQ(state_machine_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(state_machine_future.get()->success, true);

  // Land
  auto land_future = test_node->land();
  executor.spin_some();
  auto land_result = executor.spin_until_future_complete(land_future);
  ASSERT_EQ(land_result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(land_future.get()->success, true);
}

}  // namespace as2_platform_multirotor_simulator

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
