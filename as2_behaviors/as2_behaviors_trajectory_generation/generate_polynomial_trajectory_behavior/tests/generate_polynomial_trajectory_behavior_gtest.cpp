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
 * @file generate_polynomial_trajectory_behavior_gtest.cpp
 *
 * @brief Construction smoke test for GeneratePolynomialTrajectoryBehavior.
 *
 * Parametrized over every plugin declared in plugins.xml so a regression
 * in the wrapper / plugin contract is caught for all backends, not just
 * the default one.
 *
 * @authors Rafael Perez-Segui
 */

#include <gtest/gtest.h>

#include <cctype>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"
#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp"

namespace
{

using PluginBase = generate_polynomial_trajectory_behavior_plugin_base::
  GeneratePolynomialTrajectoryBase;

/// Returns plugin names declared by pluginlib (strips the trailing "::Plugin").
std::vector<std::string> getAvailablePlugins()
{
  std::vector<std::string> names;
  try {
    pluginlib::ClassLoader<PluginBase> loader(
      "as2_behaviors_trajectory_generation",
      "generate_polynomial_trajectory_behavior_plugin_base::"
      "GeneratePolynomialTrajectoryBase");
    for (const auto & declared : loader.getDeclaredClasses()) {
      const std::string suffix = "::Plugin";
      if (declared.size() > suffix.size() &&
        declared.compare(
          declared.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        names.push_back(declared.substr(0, declared.size() - suffix.size()));
      } else {
        names.push_back(declared);
      }
    }
  } catch (const std::exception & ex) {
    std::fprintf(
      stderr, "getAvailablePlugins: pluginlib query failed: %s\n", ex.what());
  }
  return names;
}

std::shared_ptr<GeneratePolynomialTrajectoryBehavior>
makeBehaviorNode(
  const std::string & plugin_name,
  const std::string & name_space)
{
  const std::string package_path = ament_index_cpp::get_package_share_directory(
    "as2_behaviors_trajectory_generation");
  const std::string config_file =
    package_path +
    "/generate_polynomial_trajectory_behavior/config/config_default.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "plugin_name:=" + plugin_name,
    "--params-file",
    config_file,
  };
  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<GeneratePolynomialTrajectoryBehavior>(node_options);
}

class GeneratePolynomialTrajectoryBehaviorTest
  : public ::testing::TestWithParam<std::string> {};

TEST_P(GeneratePolynomialTrajectoryBehaviorTest, test_constructor) {
  std::string sanitized = GetParam();
  for (char & c : sanitized) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {c = '_';}
  }
  const std::string name_space =
    "as2_behaviors_trajectory_generation_test_" + sanitized;
  EXPECT_NO_THROW(
    {
      auto node = makeBehaviorNode(GetParam(), name_space);
      (void)node;
    });
}

INSTANTIATE_TEST_SUITE_P(
  AllPlugins, GeneratePolynomialTrajectoryBehaviorTest,
  ::testing::ValuesIn(getAvailablePlugins()),
  [](const ::testing::TestParamInfo<std::string> & info) {
    std::string sanitized = info.param;
    for (char & c : sanitized) {
      if (!std::isalnum(static_cast<unsigned char>(c))) {c = '_';}
    }
    return sanitized;
  });

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
