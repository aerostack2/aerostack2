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
* @file as2_motion_controller_gtest.hpp
*
* A motion controller gtest
*
* @authors Rafael Perez-Segui
*/

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "as2_motion_controller/controller_manager.hpp"

std::shared_ptr<controller_manager::ControllerManager> getControllerManagerNode(
  const std::string plugin_name, const std::string & available_modes_config_file = "")
{
  const std::string & name_space = "test_as2_motion_controller";
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_motion_controller");
  const std::string state_estimator_config_file = package_path +
    "/config/motion_controller_default.yaml";
  const std::string plugin_config_file = package_path + "/plugins/" + plugin_name +
    "/config/controller_default.yaml";
  const std::string available_modes = available_modes_config_file.empty() ?
    package_path + "/plugins/" + plugin_name + "/config/available_modes.yaml" :
    available_modes_config_file;

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "plugin_name:=" + plugin_name,
    "-p",
    "plugin_available_modes_config_file:=" + available_modes,
    "--params-file",
    state_estimator_config_file,
    "--params-file",
    plugin_config_file,
  };

  auto node_options = rclcpp::NodeOptions();
  node_options.arguments(node_args);

  return std::make_shared<controller_manager::ControllerManager>(node_options);
}

TEST(As2MotionControllerGTest, PluginLoadDifferentialFlatnessController) {
  EXPECT_NO_THROW(getControllerManagerNode("differential_flatness_controller"));
  auto node = getControllerManagerNode("differential_flatness_controller");

  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

TEST(As2MotionControllerGTest, PluginLoadPidSpeedController) {
  EXPECT_NO_THROW(getControllerManagerNode("pid_speed_controller"));
  auto node = getControllerManagerNode("pid_speed_controller");

  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

TEST(As2MotionControllerGTest, IgnoresUnrelatedYamlNextToAvailableModes) {
  const auto temp_dir = std::filesystem::temp_directory_path() /
    ("as2_motion_controller_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(temp_dir);

  const auto available_modes = temp_dir / "available_modes.yaml";
  std::ofstream(available_modes) <<
    "input_control_modes:\n"
    "  - 0b00000000\n"
    "output_control_modes:\n"
    "  - 0b00000000\n";
  std::ofstream(temp_dir / "unrelated.yaml") << "invalid: [\n";

  EXPECT_NO_THROW(getControllerManagerNode("pid_speed_controller", available_modes.string()));

  std::filesystem::remove_all(temp_dir);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
