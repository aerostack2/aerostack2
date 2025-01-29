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
