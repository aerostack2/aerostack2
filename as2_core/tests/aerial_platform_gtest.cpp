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
 *  \file       aerial_platform_gtest.cpp
 *  \brief      Tests for the control mode resolution and command frame conversion
 *              of the aerial platform base class.
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_core/aerial_platform.hpp"

#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

namespace as2
{

namespace
{

const char kNamespace[] = "test_platform";
const char kSourceFrame[] = "test_source";

// Control modes the test platform declares, in file order.
// Bit layout: [7:4] control mode, [3:2] yaw mode, [1:0] reserved and ignored.
// One entry still sets the reserved bits, as an unmigrated file would.
const char kControlModesFile[] =
  "available_modes:\n"
  "  - 0b00010000 # HOVER\n"
  "  - 0b00100100 # BODY_RATES, yaw speed\n"
  "  - 0b01000000 # SPEED, yaw angle\n"
  "  - 0b01000101 # SPEED, yaw speed, reserved bits set\n"
  "  - 0b01000100 # SPEED, yaw speed (same mode and yaw as the entry above)\n"
  "  - 0b01010100 # SPEED_IN_A_PLANE, yaw speed\n"
  "  - 0b01100001 # POSITION, yaw angle\n"
  "  - 0b01110001 # TRAJECTORY, yaw angle\n";

std::string writeControlModesFile()
{
  const std::string path =
    (std::filesystem::temp_directory_path() / "as2_core_test_control_modes.yaml").string();
  std::ofstream file(path);
  file << kControlModesFile;
  file.close();
  return path;
}

as2_msgs::msg::ControlMode buildMode(const int8_t control_mode, const int8_t yaw_mode)
{
  as2_msgs::msg::ControlMode mode;
  mode.control_mode = control_mode;
  mode.yaw_mode = yaw_mode;
  return mode;
}

/**
 * @class TestPlatform, minimal aerial platform recording what the base class delivers
 */
class TestPlatform : public as2::AerialPlatform
{
public:
  explicit TestPlatform(const rclcpp::NodeOptions & options)
  : as2::AerialPlatform(kNamespace, options) {}

  void configureSensors() override {}
  bool ownSetArmingState(bool state) override {(void)state; return true;}
  bool ownSetOffboardControl(bool offboard) override {(void)offboard; return true;}
  void ownKillSwitch() override {}
  void ownStopPlatform() override {}

  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override
  {
    own_mode_ = msg;
    own_mode_calls_++;
    if (!own_mode_result_) {
      return false;
    }
    if (declare_frames_) {
      if (declare_pose_frame_) {
        setCommandPoseFrameId(pose_frame_id_.empty() ? getOdomFrameId() : pose_frame_id_);
      }
      setCommandTwistFrameId(twist_frame_id_.empty() ? getOdomFrameId() : twist_frame_id_);
    }
    return true;
  }

  bool ownSendCommand() override
  {
    sent_pose_ = command_pose_msg_;
    sent_twist_ = command_twist_msg_;
    sent_trajectory_ = command_trajectory_msg_;
    send_command_calls_++;
    return true;
  }

  // Expose the base class methods under test
  using as2::AerialPlatform::setPlatformControlMode;
  using as2::AerialPlatform::sendCommand;
  using as2::AerialPlatform::getCommandPoseFrameId;
  using as2::AerialPlatform::getCommandTwistFrameId;

  /**
   * @brief Put the platform in a state where sendCommand() reaches ownSendCommand()
   */
  void enableCommandSending()
  {
    setArmingState(true);
    setOffboardControl(true);
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF);
    handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF);
  }

  void setPoseCommand(const geometry_msgs::msg::PoseStamped & pose)
  {
    command_pose_msg_ = pose;
    has_new_references_ = true;
  }

  void setTwistCommand(const geometry_msgs::msg::TwistStamped & twist)
  {
    command_twist_msg_ = twist;
    has_new_references_ = true;
  }

  void setTrajectoryCommand(const as2_msgs::msg::TrajectorySetpoints & trajectory)
  {
    command_trajectory_msg_ = trajectory;
    has_new_references_ = true;
  }

  // What ownSetPlatformControlMode() does with the mode it is given
  bool own_mode_result_ = true;
  bool declare_frames_ = true;
  bool declare_pose_frame_ = true;  // false mimics a platform that only commands twists
  std::string pose_frame_id_;   // empty declares the odom frame
  std::string twist_frame_id_;  // empty declares the odom frame

  as2_msgs::msg::ControlMode own_mode_;
  int own_mode_calls_ = 0;
  int send_command_calls_ = 0;
  geometry_msgs::msg::PoseStamped sent_pose_;
  geometry_msgs::msg::TwistStamped sent_twist_;
  as2_msgs::msg::TrajectorySetpoints sent_trajectory_;
};

std::shared_ptr<TestPlatform> makePlatform()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control_modes_file", writeControlModesFile());
  return std::make_shared<TestPlatform>(options);
}

geometry_msgs::msg::TransformStamped buildTransform(
  const rclcpp::Time & stamp, const std::string & parent_frame, const std::string & child_frame,
  const double x, const double y, const double z, const double yaw = 0.0)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.z = std::sin(yaw / 2.0);
  transform.transform.rotation.w = std::cos(yaw / 2.0);
  return transform;
}

/**
 * @brief Publish the TF tree the conversion tests need, and spin the platform until
 * its transform listener has it.
 *
 * The tree is rooted at earth, the fixed frame as2::tf::TfHandler converts through:
 * earth is the source frame of the commands, the platform odom frame is offset from
 * it, so that a point at p in the source frame is at (p - offset) in the odom frame,
 * and the platform base_link frame is yawed a quarter turn from it, so that a vector
 * along +x in the source frame points along -y in the base_link frame.
 */
bool broadcastTestFrames(
  const std::shared_ptr<TestPlatform> & platform, const double x, const double y, const double z)
{
  const std::string enu_frame = std::string(kNamespace) + "/odom";
  const std::string flu_frame = std::string(kNamespace) + "/base_link";

  tf2_ros::StaticTransformBroadcaster broadcaster(platform.get());
  broadcaster.sendTransform(
    buildTransform(platform->now(), "earth", kSourceFrame, 0.0, 0.0, 0.0));
  broadcaster.sendTransform(
    buildTransform(platform->now(), "earth", enu_frame, x, y, z));
  broadcaster.sendTransform(
    buildTransform(platform->now(), "earth", flu_frame, x, y, z, M_PI_2));

  // Static transforms are latched, but the listeners still need to discover the
  // publisher. Probe with a handler of the same node, which shares this spin loop.
  as2::tf::TfHandler probe(platform.get());
  const rclcpp::Time deadline = platform->now() + rclcpp::Duration::from_seconds(5.0);
  while (platform->now() < deadline) {
    rclcpp::spin_some(platform);
    geometry_msgs::msg::PoseStamped probe_pose;
    probe_pose.header.frame_id = kSourceFrame;
    if (probe.tryConvert(probe_pose, enu_frame, std::chrono::nanoseconds::zero())) {
      rclcpp::spin_some(platform);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

geometry_msgs::msg::PoseStamped buildPose(
  const std::string & frame_id, const double x, const double y, const double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::TwistStamped buildTwist(
  const std::string & frame_id, const double x, const double y, const double z)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame_id;
  twist.twist.linear.x = x;
  twist.twist.linear.y = y;
  twist.twist.linear.z = z;
  return twist;
}

}  // namespace

TEST(AerialPlatformTest, ResolveMatchesControlAndYawMode) {
  auto platform = makePlatform();

  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));

  EXPECT_EQ(platform->own_mode_.control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_EQ(platform->own_mode_.yaw_mode, as2_msgs::msg::ControlMode::YAW_ANGLE);
}

TEST(AerialPlatformTest, ResolveHoverIgnoresYawMode) {
  auto platform = makePlatform();

  // The file declares HOVER with yaw angle, and a HOVER request matches it
  // whatever its yaw mode is
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::HOVER,
        as2_msgs::msg::ControlMode::NONE)));
  EXPECT_EQ(platform->own_mode_.control_mode, as2_msgs::msg::ControlMode::HOVER);
}

TEST(AerialPlatformTest, DropsDuplicatedModesInFile) {
  auto platform = makePlatform();

  // The file declares SPEED with yaw speed twice, differing only in the ignored
  // reserved bits, so the second entry is dropped and the first one resolves
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
  EXPECT_EQ(platform->own_mode_.control_mode, as2_msgs::msg::ControlMode::SPEED);
  EXPECT_EQ(platform->own_mode_.yaw_mode, as2_msgs::msg::ControlMode::YAW_SPEED);
}

TEST(AerialPlatformTest, RejectsModeNotDeclaredByThePlatform) {
  auto platform = makePlatform();

  // The platform is not told about a mode it does not support
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::ATTITUDE,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));
  EXPECT_EQ(platform->own_mode_calls_, 0);
  EXPECT_FALSE(platform->isControlModeSettled());
}

TEST(AerialPlatformTest, RejectsYawModeMismatch) {
  auto platform = makePlatform();

  // The file only declares POSITION with yaw angle
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
  EXPECT_EQ(platform->own_mode_calls_, 0);
}

TEST(AerialPlatformTest, SetControlModePublishesResolvedMode) {
  auto platform = makePlatform();

  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));

  // Both the platform implementation and the platform info get the resolved mode
  EXPECT_EQ(platform->own_mode_calls_, 1);
  EXPECT_EQ(platform->own_mode_.control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_EQ(platform->own_mode_.yaw_mode, as2_msgs::msg::ControlMode::YAW_ANGLE);
  EXPECT_EQ(platform->getControlMode().control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_TRUE(platform->isControlModeSettled());
}

TEST(AerialPlatformTest, ConvertsPoseAndTwistToTheDeclaredFrame) {
  auto platform = makePlatform();
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));
  ASSERT_TRUE(broadcastTestFrames(platform, 1.0, 2.0, 3.0));
  platform->enableCommandSending();

  platform->setPoseCommand(buildPose(kSourceFrame, 5.0, 5.0, 5.0));
  platform->setTwistCommand(buildTwist(kSourceFrame, 1.0, 0.0, 0.0));
  platform->sendCommand();

  ASSERT_EQ(platform->send_command_calls_, 1);
  const std::string enu_frame = std::string(kNamespace) + "/odom";
  EXPECT_EQ(platform->sent_pose_.header.frame_id, enu_frame);
  EXPECT_DOUBLE_EQ(platform->sent_pose_.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(platform->sent_pose_.pose.position.y, 3.0);
  EXPECT_DOUBLE_EQ(platform->sent_pose_.pose.position.z, 2.0);

  // A twist is a vector: it is rotated, never translated
  EXPECT_EQ(platform->sent_twist_.header.frame_id, enu_frame);
  EXPECT_DOUBLE_EQ(platform->sent_twist_.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(platform->sent_twist_.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(platform->sent_twist_.twist.linear.z, 0.0);
}

TEST(AerialPlatformTest, ConvertsTrajectorySetpoints) {
  auto platform = makePlatform();
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::TRAJECTORY,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));
  ASSERT_TRUE(broadcastTestFrames(platform, 1.0, 2.0, 3.0));
  platform->enableCommandSending();

  as2_msgs::msg::TrajectorySetpoints trajectory;
  trajectory.header.frame_id = kSourceFrame;
  trajectory.setpoints.resize(2);
  trajectory.setpoints[0].position.x = 5.0;
  trajectory.setpoints[1].position.x = 7.0;
  platform->setTrajectoryCommand(trajectory);
  platform->sendCommand();

  ASSERT_EQ(platform->send_command_calls_, 1);
  ASSERT_EQ(platform->sent_trajectory_.setpoints.size(), 2u);
  EXPECT_EQ(platform->sent_trajectory_.header.frame_id, std::string(kNamespace) + "/odom");
  EXPECT_DOUBLE_EQ(platform->sent_trajectory_.setpoints[0].position.x, 4.0);
  EXPECT_DOUBLE_EQ(platform->sent_trajectory_.setpoints[1].position.x, 6.0);
}

TEST(AerialPlatformTest, SpeedInAPlaneUsesPoseAndTwistFramesIndependently) {
  auto platform = makePlatform();

  // The altitude of the plane is commanded in the odom frame, the velocity on it
  // in the body frame
  platform->twist_frame_id_ = platform->getBaseFrameId();
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
  ASSERT_TRUE(broadcastTestFrames(platform, 1.0, 2.0, 3.0));
  platform->enableCommandSending();

  platform->setPoseCommand(buildPose(kSourceFrame, 5.0, 5.0, 5.0));
  platform->setTwistCommand(buildTwist(kSourceFrame, 1.0, 0.0, 0.0));
  platform->sendCommand();

  ASSERT_EQ(platform->send_command_calls_, 1);
  EXPECT_EQ(platform->sent_pose_.header.frame_id, platform->getOdomFrameId());
  EXPECT_DOUBLE_EQ(platform->sent_pose_.pose.position.z, 2.0);

  // The base_link frame is yawed a quarter turn, so +x in the source points to -y in it
  EXPECT_EQ(platform->sent_twist_.header.frame_id, platform->getBaseFrameId());
  EXPECT_NEAR(platform->sent_twist_.twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(platform->sent_twist_.twist.linear.y, -1.0, 1e-9);
}

TEST(AerialPlatformTest, RejectsModeWhenPlatformDoesNotDeclareFrames) {
  auto platform = makePlatform();
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));

  platform->declare_frames_ = false;
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));

  // The mode that was active stays active, and so do its frames
  EXPECT_EQ(platform->getControlMode().control_mode, as2_msgs::msg::ControlMode::POSITION);
  EXPECT_EQ(platform->getCommandPoseFrameId(), platform->getOdomFrameId());
  EXPECT_EQ(platform->getCommandTwistFrameId(), platform->getOdomFrameId());
}

TEST(AerialPlatformTest, HoverAndBodyRatesDoNotRequireFrames) {
  auto platform = makePlatform();
  platform->declare_frames_ = false;

  // Neither mode carries references that can be placed in a frame
  EXPECT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::HOVER,
        as2_msgs::msg::ControlMode::NONE)));
  EXPECT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::BODY_RATES,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
}

TEST(AerialPlatformTest, SpeedNeedsThePoseFrameOnlyForAYawAngle) {
  auto platform = makePlatform();

  // A platform that only commands velocities declares no pose frame
  platform->declare_pose_frame_ = false;

  // A yaw rate reference travels in the twist, so the twist frame is enough
  EXPECT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));

  // A yaw angle reference travels in the pose, which has no frame to be put in
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));
}

TEST(AerialPlatformTest, TrajectoryModeRejectedWhenPoseAndTwistFramesDiffer) {
  auto platform = makePlatform();

  // TrajectorySetpoints carries a single header for both
  platform->twist_frame_id_ = platform->getBaseFrameId();
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::TRAJECTORY,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));
  EXPECT_FALSE(platform->isControlModeSettled());
}

TEST(AerialPlatformTest, FramesRestoredWhenOwnSetPlatformControlModeFails) {
  auto platform = makePlatform();
  platform->twist_frame_id_ = platform->getBaseFrameId();
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));

  platform->own_mode_result_ = false;
  EXPECT_FALSE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::POSITION,
        as2_msgs::msg::ControlMode::YAW_ANGLE)));

  EXPECT_EQ(platform->getControlMode().control_mode, as2_msgs::msg::ControlMode::SPEED);
  EXPECT_EQ(platform->getCommandTwistFrameId(), platform->getBaseFrameId());
}

TEST(AerialPlatformTest, SkipsCommandWhenConversionFails) {
  auto platform = makePlatform();

  // No transform is published, so the source frame cannot be resolved
  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
  platform->enableCommandSending();

  platform->setTwistCommand(buildTwist("unknown_frame", 1.0, 1.0, 1.0));
  platform->sendCommand();

  EXPECT_EQ(platform->send_command_calls_, 0);
}

TEST(AerialPlatformTest, SkipsCommandWithoutFrame) {
  auto platform = makePlatform();

  ASSERT_TRUE(
    platform->setPlatformControlMode(
      buildMode(
        as2_msgs::msg::ControlMode::SPEED,
        as2_msgs::msg::ControlMode::YAW_SPEED)));
  platform->enableCommandSending();

  // The frame of the data is only in its header, so a command without one
  // cannot be placed in the frame the platform works in
  platform->setTwistCommand(buildTwist("", 1.0, 1.0, 1.0));
  platform->sendCommand();

  EXPECT_EQ(platform->send_command_calls_, 0);
}

}  // namespace as2

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
