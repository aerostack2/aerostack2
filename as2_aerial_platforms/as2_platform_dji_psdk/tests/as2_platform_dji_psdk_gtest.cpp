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
 *  \file       as2_platform_dji_psdk_gtest.cpp
 *  \brief      DJI PSDK platform tests
 *  \authors    Rafael Pérez Seguí
 *              Santiago Tapia Fernandez
 ********************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "as2_platform_dji_psdk/as2_platform_dji_psdk.hpp"

namespace as2_platform_dji_psdk
{

std::shared_ptr<DJIMatricePSDKPlatform> get_node()
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_dji_psdk");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {{"control_modes_file", rclcpp::ParameterValue(control_modes_config_file)}});
  return std::make_shared<DJIMatricePSDKPlatform>(node_options);
}

TEST(DJIMatricePSDKPlatform, test_constructor)
{
  EXPECT_NO_THROW(std::shared_ptr<DJIMatricePSDKPlatform> node = get_node());
}

TEST(DJIMatricePSDKPlatform, test_virtual_methods)
{
  std::shared_ptr<DJIMatricePSDKPlatform> node = get_node();
  EXPECT_NO_THROW(node->configureSensors());
  // EXPECT_NO_THROW(node->ownSetArmingState(true));
  // EXPECT_NO_THROW(node->ownSetOffboardControl(true));
  // as2_msgs::msg::ControlMode msg;
  // EXPECT_NO_THROW(node->ownSetPlatformControlMode(msg));
  // EXPECT_NO_THROW(node->ownSendCommand());
  // EXPECT_NO_THROW(node->ownStopPlatform());
  // EXPECT_NO_THROW(node->ownKillSwitch());
  // EXPECT_NO_THROW(node->ownTakeoff());
  // EXPECT_NO_THROW(node->ownLand());
}

}  // namespace as2_platform_dji_psdk

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
