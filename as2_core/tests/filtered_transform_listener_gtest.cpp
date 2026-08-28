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

/*!*******************************************************************************************
 *  \file       filtered_transform_listener_gtest.cpp
 *  \brief      Test file for the filtered transform listener
 *  \authors    Rafael Pérez Seguí
 ********************************************************************************/

#include "as2_core/utils/filtered_transform_listener.hpp"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace as2
{
namespace tf
{

// subscription_callback() is public, so the filter can be driven straight from the test
// thread with a hand made message: no publisher, no spin, no dedicated listener thread.
class FilteredTransformListenerTest : public ::testing::Test
{
protected:
  void SetUp() override {node_ = rclcpp::Node::make_shared("filtered_transform_listener_test");}

  std::shared_ptr<FilteredTransformListener> makeListener(
    tf2::BufferCore & buffer, std::vector<FilteredTransformListener::FilterRule> rules)
  {
    return std::make_shared<FilteredTransformListener>(
      buffer,
      rules,
      node_->get_node_base_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_parameters_interface(),
      node_->get_node_topics_interface(),
      false);
  }

  static tf2_msgs::msg::TFMessage::ConstSharedPtr makeMessage(
    const std::string & frame_id, const std::string & child_frame_id)
  {
    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    transform.transform.rotation.w = 1.0;
    msg->transforms.push_back(transform);
    return msg;
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(FilteredTransformListenerTest, RuleDropsMatchingTransform) {
  tf2::BufferCore buffer;
  // Drop anything published under "odom", accept the rest
  auto listener = makeListener(
    buffer,
    {[](const geometry_msgs::msg::TransformStamped & transform) {
        return transform.header.frame_id != "odom";
      }});

  listener->subscription_callback(makeMessage("odom", "base_link"), true);
  EXPECT_FALSE(buffer.canTransform("odom", "base_link", tf2::TimePointZero));

  listener->subscription_callback(makeMessage("map", "odom"), true);
  EXPECT_TRUE(buffer.canTransform("map", "odom", tf2::TimePointZero));
}

TEST_F(FilteredTransformListenerTest, NoRulesAcceptsEverything) {
  tf2::BufferCore buffer;
  auto listener = makeListener(buffer, {});

  listener->subscription_callback(makeMessage("odom", "base_link"), true);
  EXPECT_TRUE(buffer.canTransform("odom", "base_link", tf2::TimePointZero));
}

TEST_F(FilteredTransformListenerTest, TransformIsDroppedWhenAnyRuleRejectsIt) {
  tf2::BufferCore buffer;
  auto listener = makeListener(
    buffer,
    {[](const geometry_msgs::msg::TransformStamped &) {return true;},
      [](const geometry_msgs::msg::TransformStamped & transform) {
        return transform.child_frame_id != "base_link";
      }});

  listener->subscription_callback(makeMessage("odom", "base_link"), true);
  EXPECT_FALSE(buffer.canTransform("odom", "base_link", tf2::TimePointZero));
}

// Several handlers on one node is the state estimator's case: one TfHandler per plugin
TEST_F(FilteredTransformListenerTest, SeveralListenersShareTheNode) {
  tf2::BufferCore first_buffer;
  tf2::BufferCore second_buffer;
  auto first = makeListener(first_buffer, {});
  auto second = makeListener(second_buffer, {});

  first->subscription_callback(makeMessage("odom", "base_link"), true);
  EXPECT_TRUE(first_buffer.canTransform("odom", "base_link", tf2::TimePointZero));
  EXPECT_FALSE(second_buffer.canTransform("odom", "base_link", tf2::TimePointZero));
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
