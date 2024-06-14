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
 * @file as2_platform_gazebo_gtest.cpp
 *
 * Gazebo platform node gtest
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "as2_platform_gazebo.hpp"

namespace gazebo_platform
{

std::shared_ptr<gazebo_platform::GazeboPlatform> get_node(
  const std::string & name_space = "gazebo_platform")
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_gazebo");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_config_file.yaml";

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
    "-p",
    "cmd_vel_topic:=test_cmd_vel",
    "-p",
    "arm_topic:=test_arm",
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<gazebo_platform::GazeboPlatform>(node_options);
}

TEST(PlatformGazeboGTest, Constructor) {
  EXPECT_NO_THROW(get_node());
  auto node = get_node();

  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

}  // namespace gazebo_platform

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
