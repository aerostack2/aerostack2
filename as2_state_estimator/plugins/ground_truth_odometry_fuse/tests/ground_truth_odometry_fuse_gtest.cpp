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
* @file ground_truth_odometry_fuse_gtest.hpp
*
* Test for the ground_truth_odometry_fuse
*
* @authors Rafael Pérez Seguí
*/

#include <gtest/gtest.h>

#include "ground_truth_odometry_fuse.hpp"
#include <as2_core/utils/frame_utils.hpp>

/**
 * @brief Computes the transformation from the map frame to the odometry frame using Eigen::Isometry3d.
 *
 * @param odom The odometry data.
 * @param ground_truth The ground truth pose data.
 * @return geometry_msgs::msg::TransformStamped The transformation from the map to odom frame.
 */
geometry_msgs::msg::TransformStamped getTransform(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::PoseStamped & ground_truth)
{
  geometry_msgs::msg::TransformStamped map_to_odom;

  // Convert ground truth pose to Eigen Isometry3d
  Eigen::Isometry3d T_map = Eigen::Isometry3d::Identity();
  T_map.translation() = Eigen::Vector3d(
    ground_truth.pose.position.x,
    ground_truth.pose.position.y,
    ground_truth.pose.position.z);
  T_map.linear() = Eigen::Quaterniond(
    ground_truth.pose.orientation.w,
    ground_truth.pose.orientation.x,
    ground_truth.pose.orientation.y,
    ground_truth.pose.orientation.z)
    .toRotationMatrix();

  // Convert odometry pose to Eigen Isometry3d
  Eigen::Isometry3d T_odom = Eigen::Isometry3d::Identity();
  T_odom.translation() = Eigen::Vector3d(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z);
  T_odom.linear() = Eigen::Quaterniond(
    odom.pose.pose.orientation.w,
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z)
    .toRotationMatrix();

  // Compute the transformation from map to odom: T_map_to_odom = T_map * T_odom.inverse()
  Eigen::Isometry3d T_map_to_odom = T_map * T_odom.inverse();

  // Convert Eigen transformation to ROS TransformStamped
  map_to_odom.header.stamp = odom.header.stamp;
  map_to_odom.header.frame_id = ground_truth.header.frame_id;
  map_to_odom.child_frame_id = odom.header.frame_id;

  map_to_odom.transform.translation.x = T_map_to_odom.translation().x();
  map_to_odom.transform.translation.y = T_map_to_odom.translation().y();
  map_to_odom.transform.translation.z = T_map_to_odom.translation().z();

  Eigen::Quaterniond q_map_to_odom(T_map_to_odom.rotation());
  map_to_odom.transform.rotation.x = q_map_to_odom.x();
  map_to_odom.transform.rotation.y = q_map_to_odom.y();
  map_to_odom.transform.rotation.z = q_map_to_odom.z();
  map_to_odom.transform.rotation.w = q_map_to_odom.w();

  return map_to_odom;
}

TEST(As2StateEstimatorGroundTruthGTest, MapToOdomTransform)
{
  // Variables
  geometry_msgs::msg::TransformStamped transform;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "drone0/odom";
  odom.child_frame_id = "drone0/base_link";
  geometry_msgs::msg::PoseStamped ground_truth;
  ground_truth.header.frame_id = "earth";

  // Identity
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  ground_truth.pose.position.x = 1.0;
  ground_truth.pose.position.y = 0.0;
  ground_truth.pose.position.z = 0.0;
  ground_truth.pose.orientation.w = 1.0;
  ground_truth.pose.orientation.x = 0.0;
  ground_truth.pose.orientation.y = 0.0;
  ground_truth.pose.orientation.z = 0.0;

  transform = getTransform(odom, ground_truth);

  EXPECT_EQ(transform.transform.translation.x, 0.0);
  EXPECT_EQ(transform.transform.translation.y, 0.0);
  EXPECT_EQ(transform.transform.translation.z, 0.0);
  double roll, pitch, yaw;
  as2::frame::quaternionToEuler(transform.transform.rotation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);

  // Odom x=0.0, y=-1.5, z=2.0, yaw=0.0
  // Ground truth x=1.5, y=0.0, z=2.0, yaw=0.0
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = -1.5;
  odom.pose.pose.position.z = 2.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  ground_truth.pose.position.x = 1.5;
  ground_truth.pose.position.y = 0.0;
  ground_truth.pose.position.z = 2.0;
  ground_truth.pose.orientation.w = 1.0;
  ground_truth.pose.orientation.x = 0.0;
  ground_truth.pose.orientation.y = 0.0;
  ground_truth.pose.orientation.z = 0.0;

  transform = getTransform(odom, ground_truth);

  EXPECT_EQ(transform.transform.translation.x, 1.5);
  EXPECT_EQ(transform.transform.translation.y, 1.5);
  EXPECT_EQ(transform.transform.translation.z, 0.0);
  as2::frame::quaternionToEuler(transform.transform.rotation, roll, pitch, yaw);
  EXPECT_EQ(roll, 0.0);
  EXPECT_EQ(pitch, 0.0);
  EXPECT_EQ(yaw, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
