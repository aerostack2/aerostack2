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

/*!*******************************************************************************************
 *  \file       tf_utils_gtest.cpp
 *  \brief      Test file for the tf_utils library
 *  \authors    Rafael Pérez Seguí
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 ********************************************************************************/

#include "as2_core/utils/tf_utils.hpp"

#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <std_msgs/msg/bool.hpp>
#include "gtest/gtest.h"

namespace as2
{
namespace tf
{

void publicStaticTransformBroadcaster(
  rclcpp::Node * node,
  const geometry_msgs::msg::TransformStamped & transform)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
  static_broadcaster.sendTransform(transform);
}

TEST(TFHandlerTest, TfHandler) {
  // Create an as2::Node
  auto node = std::make_shared<as2::Node>("test_tf_utils_node");

  // Create a TfHandler
  auto tf_handler = std::make_shared<TfHandler>(node.get());

  // Create a tf transform static broadcaster
  std::string frame_id = "base_link";
  std::string parent_frame_id = "earth";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = parent_frame_id;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  publicStaticTransformBroadcaster(
    node.get(),
    transform);

  // Spin node to set transform
  rclcpp::spin_some(node);

  // Test public methods
  EXPECT_NO_THROW(tf_handler->setTfTimeoutThreshold(0.05));
  std::chrono::nanoseconds timeout =
    std::chrono::nanoseconds(static_cast<int>(0.05 * 1e9));
  EXPECT_NO_THROW(tf_handler->setTfTimeoutThreshold(timeout));
  EXPECT_EQ(tf_handler->getTfTimeoutThreshold(), 0.05);

  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = node->now();

  geometry_msgs::msg::PointStamped point;
  point.header = header;
  EXPECT_NO_THROW(tf_handler->convert(point, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(point, parent_frame_id));

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  EXPECT_NO_THROW(tf_handler->convert(pose, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(pose, parent_frame_id));

  geometry_msgs::msg::TwistStamped twist;
  twist.header = header;
  EXPECT_NO_THROW(tf_handler->convert(twist, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(twist, parent_frame_id));

  geometry_msgs::msg::Vector3Stamped vector;
  vector.header = header;
  EXPECT_NO_THROW(tf_handler->convert(vector, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(vector, parent_frame_id));

  nav_msgs::msg::Path path;
  path.header = header;
  EXPECT_NO_THROW(tf_handler->convert(path, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(path, parent_frame_id));

  as2_msgs::msg::TrajectorySetpoints traj;
  traj.header = header;
  EXPECT_NO_THROW(tf_handler->convert(traj, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(traj, parent_frame_id));
  EXPECT_NO_THROW(tf_handler->tryConvert(traj, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->tryConvert(traj, parent_frame_id));

  geometry_msgs::msg::QuaternionStamped quaternion;
  quaternion.header = header;
  EXPECT_NO_THROW(tf_handler->convert(quaternion, parent_frame_id, timeout));
  EXPECT_NO_THROW(tf_handler->convert(quaternion, parent_frame_id));

  EXPECT_NO_THROW(
    tf_handler->getPoseStamped(parent_frame_id, frame_id, tf2::TimePointZero, timeout));
  EXPECT_NO_THROW(
    tf_handler->getPoseStamped(parent_frame_id, frame_id, node->now(), timeout));
  EXPECT_NO_THROW(
    tf_handler->getPoseStamped(parent_frame_id, frame_id));
  EXPECT_NO_THROW(
    tf_handler->getPoseStamped(parent_frame_id, frame_id, node->now()));

  EXPECT_NO_THROW(
    tf_handler->getQuaternionStamped(parent_frame_id, frame_id, tf2::TimePointZero, timeout));
  EXPECT_NO_THROW(
    tf_handler->getQuaternionStamped(parent_frame_id, frame_id, node->now(), timeout));
  EXPECT_NO_THROW(
    tf_handler->getQuaternionStamped(parent_frame_id, frame_id));
  EXPECT_NO_THROW(
    tf_handler->getQuaternionStamped(parent_frame_id, frame_id, node->now()));

  EXPECT_NO_THROW(
    tf_handler->getTransform(parent_frame_id, frame_id, tf2::TimePointZero));

  EXPECT_NO_THROW(
    tf_handler->tryConvert(point, parent_frame_id, timeout));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(point, parent_frame_id));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(pose, parent_frame_id, timeout));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(pose, parent_frame_id));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(twist, parent_frame_id, timeout));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(twist, parent_frame_id));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(quaternion, parent_frame_id, timeout));
  EXPECT_NO_THROW(
    tf_handler->tryConvert(quaternion, parent_frame_id));

  EXPECT_NO_THROW(
    tf_handler->getState(twist, parent_frame_id, parent_frame_id, frame_id, timeout));
  EXPECT_NO_THROW(
    tf_handler->getState(twist, parent_frame_id, parent_frame_id, frame_id));
}

TEST(TFHandlerTest, convertTrajectorySetpointsIdentity) {
  // Identity case: when the trajectory frame_id matches the target frame, the
  // overload must return an unmodified copy without invoking any TF lookup.
  // This holds even if no TF buffer is available (no broadcasters needed).
  auto node = std::make_shared<as2::Node>("test_traj_identity_node");
  auto tf_handler = std::make_shared<TfHandler>(node.get());

  // Let tf initialize
  for (int i = 0; i < 30; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  as2_msgs::msg::TrajectorySetpoints traj;
  traj.header.frame_id = "odom";
  traj.header.stamp = node->now();
  traj.setpoints.resize(1);
  traj.setpoints[0].position.x = 1.0;
  traj.setpoints[0].position.y = 2.0;
  traj.setpoints[0].position.z = 3.0;
  traj.setpoints[0].twist.x = 0.5;
  traj.setpoints[0].twist.y = -0.5;
  traj.setpoints[0].twist.z = 0.1;
  traj.setpoints[0].acceleration.x = 0.0;
  traj.setpoints[0].acceleration.y = 0.0;
  traj.setpoints[0].acceleration.z = 9.81;
  traj.setpoints[0].yaw_angle = 0.5f;

  auto out = tf_handler->convert(traj, "odom");

  EXPECT_EQ(out.header.frame_id, "odom");
  ASSERT_EQ(out.setpoints.size(), 1u);
  EXPECT_DOUBLE_EQ(out.setpoints[0].position.x, 1.0);
  EXPECT_DOUBLE_EQ(out.setpoints[0].position.y, 2.0);
  EXPECT_DOUBLE_EQ(out.setpoints[0].position.z, 3.0);
  EXPECT_DOUBLE_EQ(out.setpoints[0].twist.x, 0.5);
  EXPECT_DOUBLE_EQ(out.setpoints[0].twist.y, -0.5);
  EXPECT_DOUBLE_EQ(out.setpoints[0].twist.z, 0.1);
  EXPECT_DOUBLE_EQ(out.setpoints[0].acceleration.z, 9.81);
  EXPECT_FLOAT_EQ(out.setpoints[0].yaw_angle, 0.5f);
}

TEST(TFHandlerTest, convertTrajectorySetpointsPureYawRotation) {
  // Pure yaw rotation case: with a static TF that rotates +90 deg around Z
  // from "earth" to "odom", a trajectory expressed in "earth" must come out in
  // "odom" with its position / twist / acceleration rotated and its yaw_angle
  // shifted by the TF yaw.
  auto node = std::make_shared<as2::Node>("test_traj_yaw_node");
  auto tf_handler = std::make_shared<TfHandler>(node.get());

  // Static TF: parent "odom", child "earth", rotation = +pi/2 around Z.
  // With this convention, a vector expressed in "earth" rotates by +pi/2 yaw
  // when converted to "odom".
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "earth";
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  const double half_yaw = M_PI_4;  // half angle for quaternion of yaw = pi/2
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = std::sin(half_yaw);
  tf.transform.rotation.w = std::cos(half_yaw);
  publicStaticTransformBroadcaster(node.get(), tf);
  rclcpp::spin_some(node);

  as2_msgs::msg::TrajectorySetpoints traj;
  traj.header.frame_id = "earth";
  traj.header.stamp = node->now();
  traj.setpoints.resize(1);
  traj.setpoints[0].position.x = 1.0;
  traj.setpoints[0].position.y = 0.0;
  traj.setpoints[0].position.z = 5.0;
  traj.setpoints[0].twist.x = 2.0;
  traj.setpoints[0].twist.y = 0.0;
  traj.setpoints[0].twist.z = 0.0;
  traj.setpoints[0].acceleration.x = 0.0;
  traj.setpoints[0].acceleration.y = 3.0;
  traj.setpoints[0].acceleration.z = 0.0;
  traj.setpoints[0].yaw_angle = 0.0f;

  std::chrono::nanoseconds timeout = std::chrono::milliseconds(500);
  ASSERT_NO_THROW(tf_handler->convert(traj, "odom", timeout));
  auto out = tf_handler->convert(traj, "odom", timeout);

  EXPECT_EQ(out.header.frame_id, "odom");
  ASSERT_EQ(out.setpoints.size(), 1u);

  // (1, 0, z) rotated +90 deg around Z -> (0, 1, z). Z is unchanged.
  constexpr double kTol = 1e-6;
  EXPECT_NEAR(out.setpoints[0].position.x, 0.0, kTol);
  EXPECT_NEAR(out.setpoints[0].position.y, 1.0, kTol);
  EXPECT_NEAR(out.setpoints[0].position.z, 5.0, kTol);
  // (2, 0, 0) rotated +90 deg around Z -> (0, 2, 0).
  EXPECT_NEAR(out.setpoints[0].twist.x, 0.0, kTol);
  EXPECT_NEAR(out.setpoints[0].twist.y, 2.0, kTol);
  EXPECT_NEAR(out.setpoints[0].twist.z, 0.0, kTol);
  // (0, 3, 0) rotated +90 deg around Z -> (-3, 0, 0).
  EXPECT_NEAR(out.setpoints[0].acceleration.x, -3.0, kTol);
  EXPECT_NEAR(out.setpoints[0].acceleration.y, 0.0, kTol);
  EXPECT_NEAR(out.setpoints[0].acceleration.z, 0.0, kTol);
  // Yaw is shifted by +pi/2.
  EXPECT_NEAR(out.setpoints[0].yaw_angle, static_cast<float>(M_PI_2), 1e-5);
}

TEST(TFHandlerTest, generateTfName) {
  // With empty namespace and frame name
  std::string test_ns = "/";
  std::string test_frame_name = "my_frame";
  auto result = generateTfName(test_ns, test_frame_name);
  EXPECT_EQ(result, "my_frame");

  // With namespace and relative frame name
  test_ns = "/my_ns";
  test_frame_name = "my_frame";
  result = generateTfName(test_ns, test_frame_name);
  EXPECT_EQ(result, "my_ns/my_frame");

  // With namespace and absolute frame name
  test_ns = "/my_ns";
  test_frame_name = "/my_frame";
  result = generateTfName(test_ns, test_frame_name);
  EXPECT_EQ(result, "my_frame");

  // Without namespace and relative frame name
  test_ns = "";
  test_frame_name = "my_frame";
  result = generateTfName(test_ns, test_frame_name);
  EXPECT_EQ(result, "my_frame");

  // Without namespace and absolute frame name
  test_ns = "";
  test_frame_name = "/my_frame";
  result = generateTfName(test_ns, test_frame_name);
  EXPECT_EQ(result, "my_frame");
}

}  // namespace tf
}  // namespace as2

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
