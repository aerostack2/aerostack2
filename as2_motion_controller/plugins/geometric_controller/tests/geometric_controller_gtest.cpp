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
//    of its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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
 *  \file       geometric_controller_gtest.cpp
 *  \brief      Gtest suite for the geometric controller plugin.
 *
 * This file follows the canonical gtest template shared by every
 * `as2_motion_controller` plugin. To adapt it to a new plugin, edit ONLY
 * the `test_config` namespace at the top; the helpers, fixture and common
 * tests below are intentionally identical across every plugin. Plugin-
 * specific tests live in their own section at the bottom.
 *
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
#include "geometric_controller/geometric_controller.hpp"
#include "as2_motion_controller/controller_base.hpp"
#include "as2_motion_controller/controller_manager.hpp"
#include "as2_msgs/msg/control_mode.hpp"

// === Plugin-specific configuration (the only block that changes per plugin) =
namespace test_config
{
using PluginType = geometric_controller::Plugin;
constexpr const char * kPluginNamespace = "geometric_controller";
constexpr const char * kFixtureNodeName = "geometric_controller_test";

inline std::string pluginConfigFile()
{
  return ament_index_cpp::get_package_share_directory("as2_motion_controller") +
         "/plugins/geometric_controller/config/controller_default.yaml";
}
inline std::string availableModesFile()
{
  return ament_index_cpp::get_package_share_directory("as2_motion_controller") +
         "/plugins/geometric_controller/config/available_modes.yaml";
}
inline std::string nodeNamespace()
{
  return std::string("test_") + kPluginNamespace;
}

inline as2_msgs::msg::ControlMode modeIn()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::POSITION;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  return m;
}
inline as2_msgs::msg::ControlMode modeOut()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::BODY_RATES;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  return m;
}

inline as2_msgs::msg::ControlMode modeOutSpeed()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::SPEED;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  return m;
}

inline as2_msgs::msg::ControlMode modeInTrajectory()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::TRAJECTORY;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  return m;
}

inline as2_msgs::msg::ControlMode modeInPositionYawSpeed()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::POSITION;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  return m;
}

inline as2_msgs::msg::ControlMode modeInSpeed()
{
  as2_msgs::msg::ControlMode m;
  m.control_mode = as2_msgs::msg::ControlMode::SPEED;
  m.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  return m;
}

// Bad-dim test: pick a parameter the plugin validates as a fixed-size vector.
// Set kBadDimParamName to "" to skip the test for plugins that don't validate.
constexpr const char * kBadDimParamName = "geometric_controller.position.kp";
inline std::vector<double> badDimValue()
{
  return {1.0, 1.0};
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

as2_msgs::msg::TrajectorySetpoints makeTrajectorySingleton(
  double x, double y, double z, double yaw = 0.0,
  const std::string & frame_id = "")
{
  as2_msgs::msg::TrajectorySetpoints msg;
  msg.header.frame_id =
    frame_id.empty() ? (test_config::nodeNamespace() + "/odom") : frame_id;
  as2_msgs::msg::TrajectoryPoint sp;
  sp.position.x = x;
  sp.position.y = y;
  sp.position.z = z;
  sp.yaw_angle = yaw;
  msg.setpoints.push_back(sp);
  return msg;
}

// === Fixture ================================================================

// The shipped config declares every gain as zero, which the plugin reads as a
// disabled loop and refuses to control. Give the loops usable gains.
void setUsableGains(rclcpp::Node * node)
{
  const std::vector<double> gains = {1.0, 1.0, 1.0};
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  node->set_parameter(rclcpp::Parameter(ns + "position.kp", gains));
  node->set_parameter(rclcpp::Parameter(ns + "velocity.kp", gains));
  node->set_parameter(rclcpp::Parameter(ns + "trajectory.kp", gains));
  node->set_parameter(rclcpp::Parameter(ns + "geometric.rotation_kp", gains));
  node->set_parameter(rclcpp::Parameter(ns + "geometric.mass", 1.0));
  node->set_parameter(rclcpp::Parameter(ns + "yaw.kp", 1.0));
}

class PluginFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = buildPluginHostNode();
    setUsableGains(node_.get());
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
  EXPECT_NO_THROW(applyAllParams(plugin_, node_.get()));
}

TEST_F(PluginFixture, UpdateParamsRejectsBadDim) {
  if (std::string(test_config::kBadDimParamName).empty()) {
    GTEST_SKIP() << "Plugin does not validate vector parameter dimensions";
  }
  applyAllParams(plugin_, node_.get());

  // Overwrite the parameter with a wrong-size vector and re-dispatch.
  rclcpp::Parameter bad(test_config::kBadDimParamName, test_config::badDimValue());
  node_->set_parameter(bad);
  EXPECT_THROW(
    plugin_.dispatchParameters({bad}),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(PluginFixture, SetModeValidCombo) {
  applyAllParams(plugin_, node_.get());
  EXPECT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

TEST_F(PluginFixture, SetModeRejectedWithZeroGains) {
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  const std::vector<double> zero = {0.0, 0.0, 0.0};
  node_->set_parameter(rclcpp::Parameter(ns + "position.kp", zero));
  node_->set_parameter(rclcpp::Parameter(ns + "trajectory.kp", zero));
  applyAllParams(plugin_, node_.get());

  // The velocity loop still has gains, so only the modes that need the
  // disabled loops are refused.
  EXPECT_FALSE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
  EXPECT_FALSE(plugin_.setMode(test_config::modeInTrajectory(), test_config::modeOut()));
  EXPECT_TRUE(plugin_.setMode(test_config::modeInSpeed(), test_config::modeOut()));
}

TEST_F(PluginFixture, ResetKeepsModeSettable) {
  applyAllParams(plugin_, node_.get());
  plugin_.reset();
  EXPECT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

TEST_F(PluginFixture, HoverModeUsesThePositionLoopWhenItIsUsable) {
  applyAllParams(plugin_, node_.get());

  const auto mode = plugin_.hoverMode();
  EXPECT_EQ(mode.control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_EQ(mode.yaw_mode, as2_msgs::msg::ControlMode::YAW_ANGLE);
}

TEST_F(PluginFixture, HoverModeFallsBackToTheVelocityLoop) {
  // Without a position loop the hold only drives the speed to zero.
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  const std::vector<double> zero = {0.0, 0.0, 0.0};
  node_->set_parameter(rclcpp::Parameter(ns + "position.kp", zero));
  applyAllParams(plugin_, node_.get());

  EXPECT_EQ(plugin_.hoverMode().control_mode, as2_msgs::msg::ControlMode::SPEED);
}

TEST_F(PluginFixture, HoverModeIsUnsetWithoutAnyUsableLoop) {
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  const std::vector<double> zero = {0.0, 0.0, 0.0};
  node_->set_parameter(rclcpp::Parameter(ns + "position.kp", zero));
  node_->set_parameter(rclcpp::Parameter(ns + "velocity.kp", zero));
  applyAllParams(plugin_, node_.get());

  EXPECT_EQ(plugin_.hoverMode().control_mode, as2_msgs::msg::ControlMode::UNSET);
}

// === Plugin-specific tests ==================================================

TEST_F(PluginFixture, FullControlCycle) {
  // Drive a complete tick: configure, engage SPEED mode, feed state and a
  // velocity reference, then check that computeOutput passes the reference
  // through (the geometric controller is a feedforward bypass in SPEED mode).
  applyAllParams(plugin_, node_.get());

  as2_msgs::msg::ControlMode mode_in;
  mode_in.control_mode = as2_msgs::msg::ControlMode::SPEED;
  mode_in.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  as2_msgs::msg::ControlMode mode_out;
  mode_out.control_mode = as2_msgs::msg::ControlMode::BODY_RATES;
  mode_out.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  ASSERT_TRUE(plugin_.setMode(mode_in, mode_out));

  const std::string pose_frame = plugin_.getDesiredPoseFrameId();
  const std::string twist_frame = plugin_.getDesiredTwistFrameId();

  plugin_.updateState(makePose(0, 0, 0, 0, pose_frame), makeTwist(0, 0, 0, twist_frame));
  plugin_.updateReference(makeTwist(0.0, 0.0, 0.5, twist_frame));

  geometry_msgs::msg::PoseStamped pose_out;
  geometry_msgs::msg::TwistStamped twist_out;
  as2_msgs::msg::Thrust thrust_out;
  EXPECT_TRUE(plugin_.computeOutput(0.01, pose_out, twist_out, thrust_out));
  EXPECT_DOUBLE_EQ(twist_out.twist.linear.z, 0.5);
}

TEST_F(PluginFixture, ResetClearsReferenceFlag) {
  applyAllParams(plugin_, node_.get());

  as2_msgs::msg::ControlMode mode_in;
  mode_in.control_mode = as2_msgs::msg::ControlMode::SPEED;
  mode_in.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  as2_msgs::msg::ControlMode mode_out;
  mode_out.control_mode = as2_msgs::msg::ControlMode::BODY_RATES;
  mode_out.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  ASSERT_TRUE(plugin_.setMode(mode_in, mode_out));

  const std::string twist_frame = plugin_.getDesiredTwistFrameId();
  plugin_.updateReference(makeTwist(0.0, 0.0, 0.5, twist_frame));

  ASSERT_TRUE(plugin_.isReferenceReceived());

  plugin_.reset();

  // ControllerHandler gates computeOutput on this flag, so the plugin only has
  // to report that no reference is held.
  EXPECT_FALSE(plugin_.isReferenceReceived());
}

TEST_F(PluginFixture, SetModeAcceptsTrajectory) {
  applyAllParams(plugin_, node_.get());
  EXPECT_TRUE(plugin_.setMode(test_config::modeInTrajectory(), test_config::modeOut()));
}

TEST_F(PluginFixture, UpdateReferenceTrajectoryIgnoredWhenNotTrajectory) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
  EXPECT_NO_THROW(plugin_.updateReference(makeTrajectorySingleton(1.0, 2.0, 3.0, 0.5)));
}

TEST_F(PluginFixture, UpdateReferenceTrajectoryAcceptedInTrajectoryMode) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.setMode(test_config::modeInTrajectory(), test_config::modeOut()));
  EXPECT_NO_THROW(plugin_.updateReference(makeTrajectorySingleton(1.0, 2.0, 3.0, 0.5)));
}

TEST_F(PluginFixture, YawSpeedReferenceBecomesYawRateCommand) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.setMode(test_config::modeInPositionYawSpeed(), test_config::modeOut()));

  // Hold the current pose so the desired thrust points up and the desired
  // attitude is a pure yaw rotation.
  const auto state_pose = makePose(0.0, 0.0, 1.0);
  plugin_.updateState(state_pose, makeTwist());
  plugin_.updateReference(state_pose);

  constexpr double kYawSpeed = 0.05;  // rad/s, small enough for sin(x) ~ x
  auto twist_ref = makeTwist();
  twist_ref.twist.angular.z = kYawSpeed;
  plugin_.updateReference(twist_ref);

  geometry_msgs::msg::PoseStamped pose_out;
  geometry_msgs::msg::TwistStamped twist_out;
  as2_msgs::msg::Thrust thrust_out;
  ASSERT_TRUE(plugin_.computeOutput(0.01, pose_out, twist_out, thrust_out));

  // The attitude loop reproduces the requested rate, whatever the rotation gain.
  EXPECT_NEAR(twist_out.twist.angular.z, kYawSpeed, 1e-3);
}

TEST_F(PluginFixture, EveryModeRejectedWithoutYawRotationGain) {
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  node_->set_parameter(
    rclcpp::Parameter(
      ns + "geometric.rotation_kp",
      std::vector<double>{1.0, 1.0, 0.0}));
  applyAllParams(plugin_, node_.get());

  // A zero yaw gain leaves no yaw authority, so the yaw angle modes are as
  // uncontrollable as the yaw rate ones.
  EXPECT_FALSE(plugin_.setMode(test_config::modeInPositionYawSpeed(), test_config::modeOut()));
  EXPECT_FALSE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
  EXPECT_FALSE(plugin_.setMode(test_config::modeInTrajectory(), test_config::modeOut()));
}

TEST_F(PluginFixture, SpeedExitStopsAfterThePositionLoop) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOutSpeed()));

  plugin_.updateState(makePose(0.0, 0.0, 1.0), makeTwist());
  plugin_.updateReference(makePose(1.0, 0.0, 1.0));

  geometry_msgs::msg::PoseStamped pose_out;
  geometry_msgs::msg::TwistStamped twist_out;
  as2_msgs::msg::Thrust thrust_out;
  ASSERT_TRUE(plugin_.computeOutput(0.01, pose_out, twist_out, thrust_out));

  // A position error ahead becomes a velocity command, not a thrust.
  EXPECT_GT(twist_out.twist.linear.x, 0.0);
  EXPECT_NEAR(twist_out.twist.linear.y, 0.0, 1e-9);
  EXPECT_EQ(twist_out.twist.angular.x, 0.0);
  EXPECT_EQ(twist_out.twist.angular.y, 0.0);
  EXPECT_EQ(thrust_out.thrust, 0.0);
  EXPECT_FALSE(twist_out.header.frame_id.empty());
}

TEST_F(PluginFixture, SpeedExitClosesTheYawAngleAsARate) {
  applyAllParams(plugin_, node_.get());
  ASSERT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOutSpeed()));

  plugin_.updateState(makePose(0.0, 0.0, 1.0, 0.0), makeTwist());
  plugin_.updateReference(makePose(0.0, 0.0, 1.0, 0.5));

  geometry_msgs::msg::PoseStamped pose_out;
  geometry_msgs::msg::TwistStamped twist_out;
  as2_msgs::msg::Thrust thrust_out;
  ASSERT_TRUE(plugin_.computeOutput(0.01, pose_out, twist_out, thrust_out));

  // Yaw kp is 1.0, so the rate matches the angle error of the first tick.
  EXPECT_NEAR(twist_out.twist.angular.z, 0.5, 1e-3);
}

TEST_F(PluginFixture, SpeedExitRejectedWithoutYawLoopAndNeedsNoGeometricStage) {
  const std::string ns = std::string(test_config::kPluginNamespace) + ".";
  node_->set_parameter(rclcpp::Parameter(ns + "yaw.kp", 0.0));
  node_->set_parameter(rclcpp::Parameter(ns + "geometric.mass", 0.0));
  applyAllParams(plugin_, node_.get());

  EXPECT_FALSE(plugin_.setMode(test_config::modeIn(), test_config::modeOutSpeed()));

  // With the yaw loop back, the SPEED exit does not need mass nor rotation gains.
  node_->set_parameter(rclcpp::Parameter(ns + "yaw.kp", 1.0));
  applyAllParams(plugin_, node_.get());
  EXPECT_TRUE(plugin_.setMode(test_config::modeIn(), test_config::modeOutSpeed()));
  EXPECT_FALSE(plugin_.setMode(test_config::modeIn(), test_config::modeOut()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
