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

/*!*******************************************************************************************
 *  \file       differential_flatness_controller_gtest.cpp
 *  \brief      Gtest suite for the differential flatness controller plugin.
 *
 * This file follows the canonical gtest template shared by every
 * `as2_motion_controller` plugin. To adapt it to a new plugin, edit ONLY
 * the `test_config` namespace at the top; the helpers, fixture and tests
 * below are intentionally identical across every plugin.
 *
 *  \authors    Rafael Perez-Segui
 ********************************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
#include "as2_motion_controller/controller_base.hpp"
#include "as2_motion_controller/controller_manager.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "differential_flatness_controller.hpp"

// === Plugin-specific configuration (the only block that changes per plugin) =
namespace test_config
{
using PluginType = differential_flatness_controller::Plugin;
constexpr const char * kPluginNamespace = "differential_flatness_controller";
constexpr const char * kFixtureNodeName = "differential_flatness_controller_test";

inline std::string pluginConfigFile()
{
  return ament_index_cpp::get_package_share_directory("as2_motion_controller") +
         "/plugins/differential_flatness_controller/config/controller_default.yaml";
}
inline std::string availableModesFile()
{
  return ament_index_cpp::get_package_share_directory("as2_motion_controller") +
         "/plugins/differential_flatness_controller/config/available_modes.yaml";
}
inline std::string nodeNamespace()
{
  return std::string("test_") + kPluginNamespace;
}

inline as2_msgs::msg::ControlMode modeIn()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  m.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  return m;
}
inline as2_msgs::msg::ControlMode modeOut()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::ACRO;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  m.reference_frame = as2_msgs::msg::ControlMode::BODY_FLU_FRAME;
  return m;
}

// Bad-dim test: pick a parameter the plugin validates as a fixed-size vector.
// Set kBadDimParamName to "" to skip the test for plugins that don't validate.
constexpr const char * kBadDimParamName = "";
inline std::vector<double> badDimValue()
{
  return {};
}
}  // namespace test_config

// === Common helpers (identical across plugins) ==============================

std::shared_ptr<controller_manager::ControllerManager>
getControllerManagerNode()
{
  const std::string mc_path =
    ament_index_cpp::get_package_share_directory("as2_motion_controller");
  const std::string mc_config = mc_path + "/config/motion_controller_default.yaml";
  std::vector<std::string> node_args = {
    "--ros-args",
    "-r", "__ns:=/" + test_config::nodeNamespace(),
    "-p", "namespace:=" + test_config::nodeNamespace(),
    "-p", std::string("plugin_name:=") + test_config::kPluginNamespace,
    "-p", "plugin_available_modes_config_file:=" + test_config::availableModesFile(),
    "--params-file", mc_config,
    "--params-file", test_config::pluginConfigFile(),
  };
  rclcpp::NodeOptions opts;
  opts.arguments(node_args);
  return std::make_shared<controller_manager::ControllerManager>(opts);
}

std::shared_ptr<as2::Node>
buildPluginHostNode()
{
  std::vector<std::string> node_args = {
    "--ros-args",
    "-r", "__ns:=/" + test_config::nodeNamespace(),
    "--params-file", test_config::pluginConfigFile(),
  };
  rclcpp::NodeOptions opts;
  opts.arguments(node_args);
  opts.automatically_declare_parameters_from_overrides(true);
  return std::make_shared<as2::Node>(test_config::kFixtureNodeName, opts);
}

std::vector<rclcpp::Parameter>
nodeParametersAsVector(rclcpp::Node * node)
{
  std::vector<rclcpp::Parameter> out;
  const auto names = node->list_parameters({}, 100).names;
  out.reserve(names.size());
  for (const auto & name : names) {
    if (name == "use_sim_time") {continue;}
    out.emplace_back(node->get_parameter(name));
  }
  return out;
}

void applyAllParams(
  as2_motion_controller_plugin_base::ControllerBase & plugin,
  rclcpp::Node * node)
{
  plugin.dispatchParameters(nodeParametersAsVector(node));
}

geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double z, double yaw = 0.0,
  const std::string & frame_id = "")
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id =
    frame_id.empty() ? (test_config::nodeNamespace() + "/odom") : frame_id;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.w = std::cos(yaw / 2.0);
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(yaw / 2.0);
  return msg;
}

geometry_msgs::msg::TwistStamped makeTwist(
  double vx = 0.0, double vy = 0.0, double vz = 0.0,
  const std::string & frame_id = "")
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.frame_id =
    frame_id.empty() ? (test_config::nodeNamespace() + "/odom") : frame_id;
  msg.twist.linear.x = vx;
  msg.twist.linear.y = vy;
  msg.twist.linear.z = vz;
  return msg;
}

// === Fixture ================================================================

class PluginFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = buildPluginHostNode();
    // ControllerManager normally injects this before initialize(); the test
    // fixture has to do the same so the plugin builds its parameter names
    // under the correct namespace.
    plugin_.setPluginParamNamespace(test_config::kPluginNamespace);
    plugin_.initialize(node_.get());
  }

  std::shared_ptr<as2::Node> node_;
  test_config::PluginType plugin_;
};

// === Common tests (identical across plugins) ================================

TEST(PluginGtest, PluginConstructor) {
  EXPECT_NO_THROW(test_config::PluginType());
}

TEST(PluginGtest, PluginLoadFromManager) {
  auto manager = getControllerManagerNode();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(manager);
  executor.spin_some();
  EXPECT_TRUE(manager != nullptr);
}

TEST_F(PluginFixture, DesiredFrameIds) {
  EXPECT_FALSE(plugin_.getDesiredPoseFrameId().empty());
  EXPECT_FALSE(plugin_.getDesiredTwistFrameId().empty());
}

TEST_F(PluginFixture, SetModeRejectedBeforeParameters) {
  EXPECT_FALSE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

TEST_F(PluginFixture, UpdateParamsApplyAll) {
  ASSERT_FALSE(nodeParametersAsVector(node_.get()).empty());
  applyAllParams(plugin_, node_.get());
  EXPECT_TRUE(plugin_.essentialParamsReady());
}

TEST_F(PluginFixture, UpdateParamsRejectsBadDim) {
  if (std::string(test_config::kBadDimParamName).empty()) {
    GTEST_SKIP() << "Plugin does not validate vector parameter dimensions";
  }
  // Configure once with the YAML defaults so the latch is set.
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.essentialParamsReady());

  // Overwrite the parameter with a wrong-size vector and re-dispatch; the
  // plugin re-reads from the node inside setParameters() and throws there.
  rclcpp::Parameter bad(test_config::kBadDimParamName, test_config::badDimValue());
  node_->set_parameter(bad);
  EXPECT_THROW(
    plugin_.dispatchParameters({bad}),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(PluginFixture, SetModeValidCombo) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.essentialParamsReady());
  EXPECT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

TEST_F(PluginFixture, ResetPreservesEssentialParamsLatch) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.essentialParamsReady());

  plugin_.reset();

  // essential_params_ready_ is a monotonic latch: once parameters have been
  // received, reset() must keep it true so subsequent setMode calls are
  // accepted (parameters are read once at startup, not re-fed after reset).
  EXPECT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
