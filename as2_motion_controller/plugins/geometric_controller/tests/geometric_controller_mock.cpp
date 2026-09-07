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
 *  @file       geometric_controller_mock.cpp
 *  @brief      End-to-end smoke executable for the geometric controller plugin.
 *  @authors    Rafael Pérez Seguí
 ********************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "as2_motion_controller/controller_manager.hpp"
#include "as2_motion_controller/testing/mock_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto controller_node = std::make_shared<controller_manager::ControllerManager>();
  RCLCPP_INFO(controller_node->get_logger(), "Controller manager node created");

  as2_motion_controller_test::MockPlatform::ControlModeRequest request;
  request.control_mode = as2_msgs::msg::ControlMode::POSITION;
  request.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;

  const std::vector<uint8_t> available_modes = {0b00010000, 0b00100100};

  auto mock_node = std::make_shared<as2_motion_controller_test::MockPlatform>(
    available_modes, request);
  RCLCPP_INFO(mock_node->get_logger(), "Mock platform node created");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_node);
  executor.add_node(mock_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning controller + mock platform");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
