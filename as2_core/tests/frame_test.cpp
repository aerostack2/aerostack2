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

#include "as2_core/utils/frame_utils.hpp"

#include <iostream>
#include <stdexcept>

#include "gtest/gtest.h"

TEST(FrameUtilsTest, transform) {
  EXPECT_NO_THROW(as2::frame::transform(tf2::Quaternion(), Eigen::Vector3d()));
  EXPECT_NO_THROW(as2::frame::transform(0.0f, 0.0f, 0.0f, Eigen::Vector3d()));
  EXPECT_NO_THROW(as2::frame::transform(geometry_msgs::msg::Quaternion(), Eigen::Vector3d()));
  EXPECT_NO_THROW(as2::frame::transform(Eigen::Quaterniond(), Eigen::Vector3d()));
}

TEST(FrameUtilsTest, transformInverse) {
  EXPECT_NO_THROW(as2::frame::transformInverse(tf2::Quaternion(), Eigen::Vector3d()));
  EXPECT_NO_THROW(as2::frame::transformInverse(0.0f, 0.0f, 0.0f, Eigen::Vector3d()));
  EXPECT_NO_THROW(
    as2::frame::transformInverse(geometry_msgs::msg::Quaternion(), Eigen::Vector3d()));
  EXPECT_NO_THROW(as2::frame::transformInverse(Eigen::Quaterniond(), Eigen::Vector3d()));
}

TEST(FrameUtilsTest, transforms) {
  EXPECT_EQ(
    as2::frame::transform(tf2::Quaternion::getIdentity(), Eigen::Vector3d::Identity()),
    as2::frame::transformInverse(
      tf2::Quaternion::getIdentity().inverse(),
      Eigen::Vector3d::Identity()));
  EXPECT_EQ(
    as2::frame::transform(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Identity()),
    as2::frame::transformInverse(
      Eigen::Quaterniond::Identity().inverse(),
      Eigen::Vector3d::Identity()));
}

TEST(FrameUtilsTest, quaternionToEuler) {
  double roll, pitch, yaw;
  EXPECT_NO_THROW(as2::frame::quaternionToEuler(tf2::Quaternion(), roll, pitch, yaw));
  EXPECT_NO_THROW(
    as2::frame::quaternionToEuler(geometry_msgs::msg::Quaternion(), roll, pitch, yaw));
  EXPECT_NO_THROW(as2::frame::quaternionToEuler(Eigen::Quaterniond(), roll, pitch, yaw));
}

TEST(FrameUtilsTest, eulerToQuaternion) {
  tf2::Quaternion q_tf2 = tf2::Quaternion();
  EXPECT_NO_THROW(as2::frame::eulerToQuaternion(0.0f, 0.0f, 0.0f, q_tf2));
  geometry_msgs::msg::Quaternion q_msg = geometry_msgs::msg::Quaternion();
  EXPECT_NO_THROW(as2::frame::eulerToQuaternion(0.0f, 0.0f, 0.0f, q_msg));
  Eigen::Quaterniond q_eigen = Eigen::Quaterniond();
  EXPECT_NO_THROW(as2::frame::eulerToQuaternion(0.0f, 0.0f, 0.0f, q_eigen));
}

TEST(FrameUtilsTest, getYawFromQuaternion) {
  EXPECT_NO_THROW(as2::frame::getYawFromQuaternion(tf2::Quaternion()));
  EXPECT_NO_THROW(as2::frame::getYawFromQuaternion(geometry_msgs::msg::Quaternion()));
  EXPECT_NO_THROW(as2::frame::getYawFromQuaternion(Eigen::Quaterniond()));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
