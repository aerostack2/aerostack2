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

#include "as2_core/mocks/aerial_platform/mock_aerial_platform.hpp"
#include "as2_core/mocks/executor_thread_util/executor_thread_util.hpp"
#include "as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp"

namespace as2_platform_multirotor_simulator
{

class MultirotorSimulatorPlatformTest : public testing::Test
{
protected:
  std::shared_ptr<MultirotorSimulatorPlatform> platform_node;
  std::shared_ptr<as2::mock::PlatformMockNode> test_node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::shared_ptr<as2::mock::ExecutorThreadUtil<rclcpp::executors::SingleThreadedExecutor>>
  executor_thread_util;

  void SetUp()
  {
    platform_node = get_node("multirotor_simulator");
    test_node = std::make_shared<as2::mock::PlatformMockNode>(
      "multirotor_simulator");

    // Executor
    executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Add nodes to executor
    executor->add_node(platform_node);
    executor->add_node(test_node);

    // Executor thread
    executor_thread_util =
      std::make_shared<as2::mock::ExecutorThreadUtil<rclcpp::executors::SingleThreadedExecutor>>(
      executor, 1000.0);
    executor_thread_util->start();

    // Enable test command send timer
    test_node->setCommandSendTimerState(true, 100.0);
  }

  void TearDown()
  {
    // Clean
    executor_thread_util->stop();
    executor->remove_node(test_node);
    executor->remove_node(platform_node);
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
    const std::string world_config = package_path + "/config/world_config.yaml";
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
      world_config,
      "--params-file",
      uav_config,
    };

    rclcpp::NodeOptions node_options;
    node_options.arguments(node_args);

    return std::make_shared<MultirotorSimulatorPlatform>(node_options);
  }
};  // class MultirotorSimulatorPlatformTest

TEST_F(MultirotorSimulatorPlatformTest, TakeoffAndLand) {
  // Wait 2 seconds
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Check that the platform is in the ground
  auto current_pose = test_node->getGroundThruthPose();
  double epsilon = 1e-3;
  ASSERT_NEAR(current_pose.pose.position.x, 0.0, epsilon);

  // Takeoff
  RCLCPP_INFO(test_node->get_logger(), "Taking off");
  EXPECT_TRUE(test_node->takeoffPlatform(false));

  // Hover and wait 2 seconds
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Land
  RCLCPP_INFO(test_node->get_logger(), "Landing");
  EXPECT_TRUE(test_node->landPlatform(false));
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
