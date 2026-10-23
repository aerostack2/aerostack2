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
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executor.hpp"

#include "as2_ca/ca_gateway.hpp"
#include "as2_ca/ca_gateway_client.hpp"

class CA_Gateway_ClientTest : public ::testing::Test
{
protected:
  CA_Gateway_ClientTest()
  : executor_(), executor_gateway_(),
    gateway_node_(
      std::make_shared<as2_ca::CA_Gateway>(
        rclcpp::NodeOptions().arguments({"--ros-args", "--remap", "__ns:=/drone0"}))),
    publisher_node(std::make_shared<rclcpp::Node>("test_publisher_node")),
    listener_node(std::make_shared<rclcpp::Node>("test_listener_node", "drone0")),
    client_(listener_node.get())
  {
    executor_gateway_.add_node(gateway_node_);
    executor_.add_node(publisher_node);
    executor_.add_node(listener_node);

    test_data_ = "test_data";
    test_topic_ = "test_topic";
    test_type_ = "test_type";

    local_publisher_ =
      publisher_node->create_publisher<as2_msgs::msg::InterAgentMessage>(
      "/drone0/gateway_in", rclcpp::QoS(10));

    register_client_ = listener_node->create_client<as2_msgs::srv::RegisterModule>(
      "/drone0/register_module");

    received_ = false;

    auto logger = listener_node->get_logger();
    RCLCPP_INFO(logger, "Set up test");
  }

  rclcpp::executors::MultiThreadedExecutor executor_;

  rclcpp::executors::MultiThreadedExecutor executor_gateway_;

  // CA Gateway node receiving inter agent messages
  std::shared_ptr<as2_ca::CA_Gateway> gateway_node_;

  // Publisher for inter agent messages
  rclcpp::Node::SharedPtr publisher_node;
  // Local module node that registers with the CA Gateway and receives forwarded messages
  rclcpp::Node::SharedPtr listener_node;

  rclcpp::Publisher<as2_msgs::msg::InterAgentMessage>::SharedPtr local_publisher_;

  as2_ca::CAGatewayClient client_;
  rclcpp::Client<as2_msgs::srv::RegisterModule>::SharedPtr register_client_;

  // Test data
  std::string test_data_;
  std::string test_topic_;
  std::string test_type_;

  bool received_;
};

TEST_F(CA_Gateway_ClientTest, CA_Gateway_ClientTest)
{
  // Spin the gateway executor in a separate thread to process incoming registration and messages
  std::thread gateway_thread([this]() {executor_gateway_.spin();});

  std::thread client_thread([this]() {executor_.spin();});

  // Register local module in gateway using the GatewayClient
  client_.register_module<std_msgs::msg::String>(
    test_type_, "test_module",
    [&](const std_msgs::msg::String & msg, const std::string & agent_id) {
      RCLCPP_INFO(
        listener_node->get_logger(), "Received message in local module callback: %s",
        msg.data.c_str());
      RCLCPP_INFO(listener_node->get_logger(), "Message received from agent: %s", agent_id.c_str());
      received_ = true;
      EXPECT_EQ(msg.data, "test_data");
    });

  while (client_.get_subscriber_count() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std_msgs::msg::String test_msg;
  test_msg.data = test_data_;

  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  serializer.serialize_message(&test_msg, &serialized);

  // Publish a message to the InterAgent topic with the correct type and data
  as2_msgs::msg::InterAgentMessage msg;
  msg.sender = "test_sender";
  msg.type = test_type_;
  msg.data = std::vector<uint8_t>(
    serialized.get_rcl_serialized_message().buffer,
    serialized.get_rcl_serialized_message().buffer +
    serialized.get_rcl_serialized_message().buffer_length);
  local_publisher_->publish(msg);

  while (!received_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor_gateway_.cancel();
  gateway_thread.join();

  executor_.cancel();
  client_thread.join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
