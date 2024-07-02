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
 * @file as2_platform_multirotor_simulator_gtest.cpp
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

namespace as2_platform_multirotor_simulator
{

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

TEST(MultirotorSimulatorPlatform, test_constructor)
{
  EXPECT_NO_THROW(std::shared_ptr<MultirotorSimulatorPlatform> node = get_node("test_constructor"));
}

TEST(MultirotorSimulatorPlatform, test_virtual_methods)
{
  std::shared_ptr<MultirotorSimulatorPlatform> node = get_node("test_virtual_methods");
  EXPECT_NO_THROW(node->configureSensors());
  EXPECT_NO_THROW(node->ownSetArmingState(true));
  EXPECT_NO_THROW(node->ownSetOffboardControl(true));
  as2_msgs::msg::ControlMode msg;
  msg.control_mode = as2_msgs::msg::ControlMode::ACRO;
  EXPECT_NO_THROW(node->ownSetPlatformControlMode(msg));
  EXPECT_NO_THROW(node->ownSendCommand());
  EXPECT_NO_THROW(node->ownStopPlatform());
  EXPECT_NO_THROW(node->ownKillSwitch());
  // EXPECT_NO_THROW(node->ownTakeoff()); // Block without an Executor
  // EXPECT_NO_THROW(node->ownLand()); // Block without an Executor

  // Spin node
  rclcpp::spin_some(node);
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
