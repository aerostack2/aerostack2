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
 * @file is_flying_condition.hpp
 *
 * behavior tree node to check if an aircraft is flying
 *
 * @authors Pedro Arias Pérez
 *          Rafael Perez-Segui
 *          Miguel Fernández Cortizas
 */

#include "as2_behavior_tree/condition/is_flying_condition.hpp"

namespace as2_behavior_tree
{
IsFlyingCondition::IsFlyingCondition(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf), topic_name_(as2_names::topics::platform::info)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  std::string remapped_topic_name;
  if (getInput("topic_name", remapped_topic_name)) {
    topic_name_ = remapped_topic_name;
  }

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  state_sub_ = node_->create_subscription<as2_msgs::msg::PlatformInfo>(
    topic_name_, as2_names::topics::platform::qos,
    std::bind(&IsFlyingCondition::state_callback, this, std::placeholders::_1),
    sub_option);

  wait_for_message();
}


BT::PortsList IsFlyingCondition::providedPorts() {
  return {
      BT::InputPort<std::string>("topic_name", "Platform info topic name")
    };
}

BT::NodeStatus IsFlyingCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_flying_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsFlyingCondition::state_callback(as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
  msg_arrived_ = true;
  is_flying_ = msg->status.state == as2_msgs::msg::PlatformStatus::FLYING;
}

void IsFlyingCondition::wait_for_message()
{
  RCLCPP_INFO(node_->get_logger(),
              "Waiting for message on topic %s...",
              topic_name_.c_str());

  rclcpp::Rate rate(100);
  auto start = node_->now();

  rclcpp::Duration timeout(wait_for_timeout_);
  while (rclcpp::ok())
  {
    callback_group_executor_.spin_some();

    if (msg_arrived_)
    {
      RCLCPP_INFO(node_->get_logger(), "... message received");
      return;
    }

    if (wait_for_timeout_.count() > 0 &&
        (node_->now() - start) > timeout)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "\"%s\" topic not available after waiting for %ld ms",
                   topic_name_.c_str(),
                   wait_for_timeout_.count());

      throw std::runtime_error(
        "Topic " + topic_name_ + " not available");
    }

    rate.sleep();
  }
}

}  // namespace as2_behavior_tree
