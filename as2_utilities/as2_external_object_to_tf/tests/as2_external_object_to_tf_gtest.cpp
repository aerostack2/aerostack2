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
* @file as2_external_object_to_tf_node.cpp
*
* as2_external_object_to_tf test file.
*
* @author Javilinos
*/

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <as2_core/names/services.hpp>
#include <as2_msgs/srv/add_static_transform_gps.hpp>
#include <as2_msgs/srv/get_origin.hpp>
#include "as2_external_object_to_tf/as2_external_object_to_tf.hpp"

TEST(As2ExternalObjectToTf, test_constructor) {
  EXPECT_NO_THROW(
    std::shared_ptr<As2ExternalObjectToTf> node =
    std::make_shared<As2ExternalObjectToTf>());
}

TEST(As2ExternalObjectToTf, test_add_static_transform_gps_when_spinning) {
  auto tf_node = std::make_shared<As2ExternalObjectToTf>();
  tf_node->configure();
  tf_node->activate();

  // Mock server node providing get_origin service
  auto mock_origin_node = rclcpp::Node::make_shared("mock_gps_origin_node");
  auto origin_srv = mock_origin_node->create_service<as2_msgs::srv::GetOrigin>(
    as2_names::services::gps::get_origin,
    [](const std::shared_ptr<as2_msgs::srv::GetOrigin::Request>,
      std::shared_ptr<as2_msgs::srv::GetOrigin::Response> response) {
      response->origin.latitude = 40.45;
      response->origin.longitude = -3.73;
      response->origin.altitude = 650.0;
      response->success = true;
    });

  rclcpp::executors::SingleThreadedExecutor tf_executor;
  tf_executor.add_node(tf_node->get_node_base_interface());

  rclcpp::executors::SingleThreadedExecutor origin_executor;
  origin_executor.add_node(mock_origin_node);

  std::thread tf_spin_thread([&tf_executor]() {
    tf_executor.spin();
  });

  std::thread origin_spin_thread([&origin_executor]() {
    origin_executor.spin();
  });

  // Client node to invoke add_static_transform_gps while tf_node is spinning
  auto client_node = rclcpp::Node::make_shared("test_gps_client_node");
  auto client = client_node->create_client<as2_msgs::srv::AddStaticTransformGps>(
    tf_node->generate_local_name("add_static_transform_gps"));

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<as2_msgs::srv::AddStaticTransformGps::Request>();
  request->frame_id = "earth";
  request->child_frame_id = "target_obj";
  request->gps_position.latitude = 40.4501;
  request->gps_position.longitude = -3.7301;
  request->gps_position.altitude = 652.0;
  request->azimuth = 0.0;
  request->elevation = 0.0;

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node->get_node_base_interface(), future, std::chrono::seconds(10)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_TRUE(response->success);

  tf_executor.cancel();
  origin_executor.cancel();
  if (tf_spin_thread.joinable()) {
    tf_spin_thread.join();
  }
  if (origin_spin_thread.joinable()) {
    origin_spin_thread.join();
  }
}

TEST(As2ExternalObjectToTf, test_add_static_transform_gps_service_unavailable) {
  auto tf_node = std::make_shared<As2ExternalObjectToTf>();
  tf_node->configure();
  tf_node->activate();

  rclcpp::executors::SingleThreadedExecutor tf_executor;
  tf_executor.add_node(tf_node->get_node_base_interface());

  std::thread tf_spin_thread([&tf_executor]() {
    tf_executor.spin();
  });

  // Client node to invoke add_static_transform_gps when no origin service is available
  auto client_node = rclcpp::Node::make_shared("test_gps_client_node_fail");
  auto client = client_node->create_client<as2_msgs::srv::AddStaticTransformGps>(
    tf_node->generate_local_name("add_static_transform_gps"));

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<as2_msgs::srv::AddStaticTransformGps::Request>();
  request->frame_id = "earth";
  request->child_frame_id = "target_obj";
  request->gps_position.latitude = 40.4501;
  request->gps_position.longitude = -3.7301;
  request->gps_position.altitude = 652.0;
  request->azimuth = 0.0;
  request->elevation = 0.0;

  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      client_node->get_node_base_interface(), future, std::chrono::seconds(15)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  // Should fail gracefully without crashing or throwing
  EXPECT_FALSE(response->success);

  tf_executor.cancel();
  if (tf_spin_thread.joinable()) {
    tf_spin_thread.join();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
