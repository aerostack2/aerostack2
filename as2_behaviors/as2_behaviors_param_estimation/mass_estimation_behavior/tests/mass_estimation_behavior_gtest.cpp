// Copyright 2025 Universidad Politécnica de Madrid
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


/*!******************************************************************************
 *  \file       mass_estimation_behavior_gtest.cpp
 *  \brief      mass_estimation_behavior gtest file.
 *  \authors    Carmen De Rojas Pita-Romero
 ********************************************************************************/

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mass_estimation_behavior.hpp"

std::shared_ptr<mass_estimation_behavior::MassEstimationBehavior> getMassEstimationBehaviorNode()
{
  const std::string & name_space = "test_mass_estimation_behavior";
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_behaviors_param_estimation");
  const std::string sub_package = "mass_estimation_behavior";
  const std::string config_file = package_path + "/" + sub_package +
    "/config/config_default.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "--params-file",
    config_file
  };
  auto node_options = rclcpp::NodeOptions();
  node_options.arguments(node_args);

  return std::make_shared<mass_estimation_behavior::MassEstimationBehavior>(node_options);
}

TEST(MassEstimationGTest, NodeCreation) {
  EXPECT_NO_THROW(getMassEstimationBehaviorNode());
  auto node = getMassEstimationBehaviorNode();
  // Spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
