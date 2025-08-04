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
* @file detect_behavior_gtest.cpp
*
* A detect behavior gtest file.
*
* @authors Guillermo GP-Lenza
*/

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include "detect_behavior.hpp"

std::shared_ptr<detect_behavior::DetectBehavior> getDetectBehaviorNode(
  const std::string plugin_name, const std::string node_name_prefix = "test_detect_behavior")
{
  const std::string & name_space = node_name_prefix + plugin_name;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("detect_behavior");
  const std::string detect_behavior_config_file = package_path +
    "/config/config_default.yaml";
  const std::string plugin_config_file = package_path + "/plugins/" + plugin_name +
    "/config/plugin_default.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "plugin_name:=" + plugin_name,
    "--params-file",
    detect_behavior_config_file,
    "--params-file",
    plugin_config_file,
  };

  auto node_options = rclcpp::NodeOptions();
  node_options.arguments(node_args);

  return std::make_shared<detect_behavior::DetectBehavior>(node_options);
}

TEST(DetectBehaviorGTest, PluginLoadGroundTruth) {
  EXPECT_NO_THROW(getDetectBehaviorNode("gate_color"));
  auto node = getDetectBehaviorNode("gate_color", "test_detect_behavior_gate_color");

  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
