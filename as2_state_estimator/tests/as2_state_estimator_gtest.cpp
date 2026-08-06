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
* @file as2_state_estimator_gtest.hpp
*
* An state estimation gtest
*
* @authors Rafael Pérez Seguí
*          Rodrigo Da Silva Gómez
*/

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "as2_state_estimator/as2_state_estimator.hpp"

std::shared_ptr<as2_state_estimator::StateEstimator> getStateEstimatorNode(
  const std::string plugin_name, const std::string node_name_prefix = "test_state_estimator")
{
  const std::string & name_space = node_name_prefix + plugin_name;
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_state_estimator");
  const std::string state_estimator_config_file = package_path +
    "/config/state_estimator_default.yaml";
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
    state_estimator_config_file,
    "--params-file",
    plugin_config_file,
  };

  auto node_options = rclcpp::NodeOptions();
  node_options.arguments(node_args);

  auto node = std::make_shared<as2_state_estimator::StateEstimator>(node_options);
  // PluginWrapper::create() resolves the owning node through StateEstimator::getInstance().
  // A directly constructed node must register itself, otherwise getInstance() would build a
  // second, default-configured StateEstimator and the plugin would attach to that instead.
  as2_state_estimator::StateEstimator::instance_ = node;
  return node;
}

// Plugin loading is deferred to a 1s wall timer (StateEstimator's start_timer_), so the
// executor has to be spun for longer than that with real elapsed time. spin_some() alone
// returns immediately and setup() never runs.
static bool spinUntilPluginLoaded(
  rclcpp::executors::MultiThreadedExecutor & executor,
  const rclcpp::Node::SharedPtr & observer,
  const std::string & name_space,
  const std::string & plugin_name,
  int max_iterations = 40)
{
  // state_estimation/<plugin>/pose is created by PluginWrapper::create() only AFTER
  // pluginlib has successfully instantiated the plugin, so its presence is direct evidence
  // that the plugin class was found, loaded and constructed — not merely that the node was.
  const std::string probe = "/" + name_space + "/state_estimation/" + plugin_name + "/pose";
  for (int i = 0; i < max_iterations; ++i) {
    executor.spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (observer->get_topic_names_and_types().count(probe)) {
      return true;
    }
  }
  return false;
}

class PluginLoadTest : public ::testing::TestWithParam<std::string> {};

// Every plugin in PLUGIN_LIST must actually load. This asserts on an artifact of a
// successful pluginlib instantiation rather than on construction alone: the previous
// version of this test spun once with spin_some(), which returns before the 1s deferred
// setup() timer can fire, so no plugin was ever loaded and the test would have passed even
// with the plugin deleted from plugins.xml.
TEST_P(PluginLoadTest, PluginIsActuallyLoaded) {
  const std::string plugin_name = GetParam();
  const std::string name_space = "test_state_estimator_load_" + plugin_name;

  ASSERT_NO_THROW(getStateEstimatorNode(plugin_name, "test_state_estimator_noThrow_"));

  auto node = getStateEstimatorNode(plugin_name, "test_state_estimator_load_");
  auto observer = rclcpp::Node::make_shared(plugin_name + "_load_observer");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);

  EXPECT_TRUE(spinUntilPluginLoaded(executor, observer, name_space, plugin_name))
    << "plugin '" << plugin_name << "' never advertised state_estimation/" << plugin_name
    << "/pose, which means pluginlib did not instantiate it";
}

INSTANTIATE_TEST_SUITE_P(
  AllPlugins, PluginLoadTest,
  ::testing::Values("ground_truth", "raw_odometry", "simple_ekf"),
  [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  // Explicitly release the last StateEstimator node (and its pluginlib ClassLoader) here,
  // while the process is still in a normal state. Left to the static destructor at program
  // exit, unloading the plugin's shared library races with the dynamic linker's own global
  // teardown and reliably segfaults inside class_loader.
  as2_state_estimator::StateEstimator::instance_.reset();
  rclcpp::shutdown();
  return result;
}
