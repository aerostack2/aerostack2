// Copyright 2023 Universidad Politécnica de Madrid
// Copyright (c) 2008, Willow Garage, Inc.
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
 *  \file       filtered_transform_listener.hpp
 *  \brief      filtered_transform_listener header file.
 *  \authors    Tully Foote
 *              Miguel Fernandez Cortizas
 *              Rafael Perez Segui
 ********************************************************************************/


#ifndef AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_
#define AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <vector>


#include "tf2/tf2/buffer_core.h"
#include "tf2/tf2/time.h"
#include "tf2_ros/visibility_control.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_ros/qos.hpp"

namespace as2
{

namespace tf
{

// /**
//  * @brief This class is similar to the tf2_ros::TransformListener but it allows to filter the
//  * transforms that are received by the listener and added to the buffer. We based on the code of
//  * the tf2_ros::TransformListener and we added the filter functionality in the subscribe function.
//  */
class FilteredTransformListener
{
public:
  /** \brief Simplified constructor for transform listener.
   *
   * This constructor will create a new ROS 2 node under the hood.
   * If you already have access to a ROS 2 node and you want to associate the TransformListener
   * to it, then it's recommended to use one of the other constructors.
   */
  TF2_ROS_PUBLIC
  explicit FilteredTransformListener(
    tf2::BufferCore & buffer,
    bool spin_thread = true,
    bool static_only = false);

  /** \brief Node constructor */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  FilteredTransformListener(
    tf2::BufferCore & buffer,
    NodeT && node,
    bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    tf2_ros::detail::get_default_transform_listener_sub_options<AllocatorT>(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options =
    tf2_ros::detail::get_default_transform_listener_static_sub_options<AllocatorT>(),
    bool static_only = false)
  : FilteredTransformListener(
      buffer,
      node->get_node_base_interface(),
      node->get_node_logging_interface(),
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      spin_thread,
      qos,
      static_qos,
      options,
      static_options,
      static_only)
  {}

  /** \brief Node interface constructor */
  template<class AllocatorT = std::allocator<void>>
  FilteredTransformListener(
    tf2::BufferCore & buffer,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    tf2_ros::detail::get_default_transform_listener_sub_options<AllocatorT>(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options =
    tf2_ros::detail::get_default_transform_listener_static_sub_options<AllocatorT>(),
    bool static_only = false)
  : buffer_(buffer)
  {
    init(
      node_base,
      node_logging,
      node_parameters,
      node_topics,
      spin_thread,
      qos,
      static_qos,
      options,
      static_options,
      static_only);
  }

  TF2_ROS_PUBLIC
  virtual ~FilteredTransformListener();

  /// Callback function for ros message subscription
  TF2_ROS_PUBLIC
  virtual void subscription_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static);

  // added filter rules vector
  bool add_filter_rule(
    std::function<bool(const geometry_msgs::msg::TransformStamped &)> _filter_rule)
  {
    filter_rules_.push_back(_filter_rule);
    return true;
  }

  // added clear filter rules function
  void clear_filter_rules()
  {
    filter_rules_.clear();
  }

private:
  template<class AllocatorT = std::allocator<void>>
  void init(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    bool spin_thread,
    const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options,
    bool static_only = false)
  {
    spin_thread_ = spin_thread;
    node_base_interface_ = node_base;
    node_logging_interface_ = node_logging;

    using callback_t = std::function<void (tf2_msgs::msg::TFMessage::ConstSharedPtr)>;
    callback_t cb = std::bind(
      &FilteredTransformListener::subscription_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(
      &FilteredTransformListener::subscription_callback, this, std::placeholders::_1, true);

    if (spin_thread_) {
      // Create new callback group for message_subscription of tf and tf_static
      callback_group_ = node_base_interface_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);

      if (!static_only) {
        // Duplicate to modify subscription options
        rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_options = options;
        tf_options.callback_group = callback_group_;

        message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
          node_parameters, node_topics, "/tf", qos, std::move(cb), tf_options);
      }

      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_static_options = static_options;
      tf_static_options.callback_group = callback_group_;

      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters,
        node_topics,
        "/tf_static",
        static_qos,
        std::move(static_cb),
        tf_static_options);

      // Create executor with dedicated thread to spin.
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
      // Tell the buffer we have a dedicated thread to enable timeouts
      buffer_.setUsingDedicatedThread(true);
    } else {
      if (!static_only) {
        message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
          node_parameters, node_topics, "/tf", qos, std::move(cb), options);
      }
      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters,
        node_topics,
        "/tf_static",
        static_qos,
        std::move(static_cb),
        static_options);
    }
  }

  bool spin_thread_{false};
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};

  rclcpp::Node::SharedPtr optional_default_node_ {nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
    message_subscription_tf_ {nullptr};
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
    message_subscription_tf_static_ {nullptr};
  tf2::BufferCore & buffer_;
  tf2::TimePoint last_update_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ {nullptr};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};


  std::vector<std::function<bool(const geometry_msgs::msg::TransformStamped &)>> filter_rules_;
};

}  // namespace tf

}  // namespace as2

#endif  // AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_
