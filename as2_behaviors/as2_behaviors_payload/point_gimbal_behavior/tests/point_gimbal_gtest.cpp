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
 *  \file       point_gimbal_gtest.hpp
 *  \brief      Point Gimbal behavior test file.
 *  \authors    Rafael Perez-Segui
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "point_gimbal_behavior/point_gimbal_behavior.hpp"

namespace point_gimbal_behavior
{

#define MIN_ROLL -M_PI_2
#define MAX_ROLL M_PI_2
#define MIN_PITCH -M_PI_2
#define MAX_PITCH M_PI_2
#define MIN_YAW -M_PI_2
#define MAX_YAW M_PI_2

// Test class to make protected methods public
class PointGimbalBehaviorTest : PointGimbalBehavior
{
public:
  explicit PointGimbalBehaviorTest(rclcpp::NodeOptions options)
  : PointGimbalBehavior(options)
  {}

  using PointGimbalBehavior::check_gimbal_limits;
};

std::shared_ptr<PointGimbalBehaviorTest> get_node()
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_behaviors_perception");
  const std::string config_file =
    package_path + "/point_gimbal_behavior/config/config_default.yaml";
  rclcpp::NodeOptions node_options;
  // Add config params file
  node_options.append_parameter_override("config_file", config_file);

  // Add param gimbal_max_angle
  node_options.append_parameter_override("roll_range.min", MIN_ROLL);
  node_options.append_parameter_override("roll_range.max", MAX_ROLL);
  node_options.append_parameter_override("pitch_range.min", MIN_PITCH);
  node_options.append_parameter_override("pitch_range.max", MAX_PITCH);
  node_options.append_parameter_override("yaw_range.min", MIN_YAW);
  node_options.append_parameter_override("yaw_range.max", MAX_YAW);
  return std::make_shared<PointGimbalBehaviorTest>(node_options);
}

TEST(PointGimbalBehavior, constructor)
{
  EXPECT_NO_THROW(std::shared_ptr<PointGimbalBehaviorTest> node = get_node());
}

TEST(PointGimbalBehavior, check_gimbal_limits)
{
  std::shared_ptr<PointGimbalBehaviorTest> node = get_node();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  // Check gimbal limits
  // Roll
  roll = MAX_ROLL / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  roll = MIN_ROLL / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  roll = MIN_ROLL - 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));

  roll = MAX_ROLL + 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));
  roll = 0.0;

  // Pitch
  pitch = MAX_PITCH / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  pitch = MIN_PITCH / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  pitch = MIN_PITCH - 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));

  pitch = MAX_PITCH + 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));
  pitch = 0.0;

  // Yaw
  yaw = MAX_YAW / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  yaw = MIN_YAW / 2.0;
  EXPECT_TRUE(node->check_gimbal_limits(roll, pitch, yaw));

  yaw = MIN_YAW - 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));

  yaw = MAX_YAW + 0.1;
  EXPECT_FALSE(node->check_gimbal_limits(roll, pitch, yaw));
  yaw = 0.0;
}

}  // namespace point_gimbal_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
