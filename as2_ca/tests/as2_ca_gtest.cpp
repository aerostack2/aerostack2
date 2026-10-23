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

/**
* @file as2__gtest.cpp
*
* A collective awareness structure gtest
*
* @authors Guillermo GP-Lenza
*/

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executor.hpp"

#include "as2_ca/ca_gateway.hpp"


class CA_GatewayTest : public ::testing::Test
{
protected:
  CA_GatewayTest()
  : node_(
      std::make_shared<as2_ca::CA_Gateway>(
        rclcpp::NodeOptions().arguments({"--ros-args", "--remap", "__ns:=/drone0"}))),
    executor_(),
    test_node_(std::make_shared<rclcpp::Node>("test_node", "drone0"))
  {
    executor_.add_node(node_);
    executor_.add_node(test_node_);
    received = false;
    test_data_ = "test_data";
  }

  std::shared_ptr<as2_ca::CA_Gateway> node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> test_node_;

  std::string test_data_;

  bool received;
};

TEST_F(CA_GatewayTest, RegisterModuleService)
{
  // Test that the service is available
  auto client = test_node_->create_client<as2_msgs::srv::RegisterModule>(
    "/drone0/register_module");
  EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  as2_msgs::srv::RegisterModule::Request request;
  request.type = "test_type";
  request.module_name = "test_module";
  auto future =
    client->async_send_request(
    std::make_shared<
      as2_msgs::srv::RegisterModule::Request>(request));
  // Spin until the response is received
  while (rclcpp::ok()) {
    executor_.spin_some();
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      break;
    }
  }

  // Wait for response
  auto response = future.get();
  EXPECT_FALSE(response->topic.empty());
  EXPECT_TRUE(response->topic.find("test_type_in") != std::string::npos);
}

TEST_F(CA_GatewayTest, CorrectForwarding)
{
  // Test that the service is available
  auto client = test_node_->create_client<as2_msgs::srv::RegisterModule>(
    "/drone0/register_module");
  EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  as2_msgs::srv::RegisterModule::Request request;
  request.type = "test_type";
  request.module_name = "test_module";
  auto future =
    client->async_send_request(
    std::make_shared<as2_msgs::srv::RegisterModule::Request>(request)
    );

  // Spin until the response is received
  while (rclcpp::ok()) {
    executor_.spin_some();
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      break;
    }
  }

  // Wait for response
  auto response = future.get();
  EXPECT_FALSE(response->topic.empty());
  EXPECT_TRUE(response->topic.find("test_type_in") != std::string::npos);

  // Create subscriber to the local topic
  std::string local_topic = response->topic;
  auto subscription = test_node_->create_subscription<as2_msgs::msg::LocalGenericMessage>(
    local_topic, rclcpp::QoS(10),
    [this](
      const as2_msgs::msg::LocalGenericMessage::SharedPtr msg) {
      std::cout << "Received message on local topic: " << msg->agents[0] <<
        ", " << msg->type << std::endl;
      EXPECT_EQ(msg->type, "test_type");
      EXPECT_EQ(
        msg->data,
        std::vector<uint8_t>(test_data_.begin(), test_data_.end()));
      this->received = true;
    });

  std::shared_ptr<rclcpp::Node> test_node2 = std::make_shared<rclcpp::Node>("test_node2");
  executor_.add_node(test_node2);

  // Create a publisher to the inter-agent topic
  auto publisher = test_node2->create_publisher<as2_msgs::msg::InterAgentMessage>(
    "/drone0/gateway_in", rclcpp::QoS(
      10));

  // Create a message to inter_agent topic
  as2_msgs::msg::InterAgentMessage msg;
  msg.sender = "test_sender";
  msg.type = "test_type";
  msg.data = std::vector<uint8_t>(test_data_.begin(), test_data_.end());

  // Publish the message
  publisher->publish(msg);

  // Spin until the message is received
  while (rclcpp::ok() && !received) {
    executor_.spin_some();
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
