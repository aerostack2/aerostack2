// Copyright 2026 Universidad Politécnica de Madrid
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
 *
 *  Copy of tf2_ros/include/tf2_ros/transform_listener.hpp, from ros2/geometry2 at
 *  b950c7ea204d (2026-05-29, jazzy). Copyright (c) 2008 Willow Garage, Inc., released under
 *  the 3-clause BSD licence reproduced above. Kept line by line identical to that revision
 *  except for three things: the filter rules, the tf2_ros::detail helpers, which are used
 *  from the original header instead of copied, and the StaticTransformListener convenience
 *  class, which is dropped.
 *
 *  \authors    Tully Foote
 *              Miguel Fernandez Cortizas
 *              Rafael Perez Segui
 *              Rodrigo Da Silva Gómez
 ********************************************************************************/

#ifndef AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_
#define AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "tf2/buffer_core.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/visibility_control.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/qos.hpp"

namespace as2
{

namespace tf
{

/** \brief This class provides an easy way to request and receive coordinate frame transform
 * information, dropping the transforms rejected by a set of filter rules.
 *
 * The rules cannot be changed after construction: the listener thread reads them without a lock
 * and is already running when the constructor returns.
 *
 * From jazzy on, tf2_ros::TransformListener::subscription_callback() is public and virtual, so
 * this whole file can be replaced by a subclass overriding it. It is private in humble, which is
 * why the class is copied instead of derived.
 */
class FilteredTransformListener
{
public:
  /// Rule applied to an incoming transform. Returning false keeps it out of the buffer.
  using FilterRule = std::function<bool (const geometry_msgs::msg::TransformStamped &)>;

  /** \brief Simplified constructor for transform listener.
   *
   * This constructor will create a new ROS 2 node under the hood.
   * If you already have access to a ROS 2 node and you want to associate the
   * FilteredTransformListener to it, then it's recommended to use one of the other constructors.
   */
  TF2_ROS_PUBLIC
  explicit FilteredTransformListener(
    tf2::BufferCore & buffer,
    std::vector<FilterRule> filter_rules = {},
    bool spin_thread = true);

  /** \brief Simplified constructor for transform listener with static_only option.
   *
   * This constructor will create a new ROS 2 node under the hood.
   * If you already have access to a ROS 2 node and you want to associate the
   * FilteredTransformListener to it, then it's recommended to use one of the other constructors.
   */
  TF2_ROS_PUBLIC
  explicit FilteredTransformListener(
    tf2::BufferCore & buffer,
    std::vector<FilterRule> filter_rules,
    bool spin_thread,
    bool static_only);

  /** \brief Node constructor */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  FilteredTransformListener(
    tf2::BufferCore & buffer,
    std::vector<FilterRule> filter_rules,
    NodeT && node,
    bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    tf2_ros::detail::get_default_transform_listener_sub_options<AllocatorT>(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options =
    tf2_ros::detail::get_default_transform_listener_static_sub_options<AllocatorT>())
  : FilteredTransformListener(
      buffer,
      std::move(filter_rules),
      node->get_node_base_interface(),
      node->get_node_logging_interface(),
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      spin_thread,
      qos,
      static_qos,
      options,
      static_options)
  {}

  /** \brief Node constructor with static_only option */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  FilteredTransformListener(
    tf2::BufferCore & buffer,
    std::vector<FilterRule> filter_rules,
    NodeT && node,
    bool spin_thread,
    const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options,
    bool static_only)
  : FilteredTransformListener(
      buffer,
      std::move(filter_rules),
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
    std::vector<FilterRule> filter_rules,
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
    tf2_ros::detail::get_default_transform_listener_static_sub_options<AllocatorT>())
  : buffer_(buffer), filter_rules_(std::move(filter_rules))
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
      static_options);
  }

  /** \brief Node interface constructor with static_only option */
  template<class AllocatorT = std::allocator<void>>
  FilteredTransformListener(
    tf2::BufferCore & buffer,
    std::vector<FilterRule> filter_rules,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    bool spin_thread,
    const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options,
    bool static_only)
  : buffer_(buffer), filter_rules_(std::move(filter_rules))
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
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options)
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
      // Duplicate to modify option of subscription
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_options = options;
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_static_options = static_options;
      tf_options.callback_group = callback_group_;
      tf_static_options.callback_group = callback_group_;

      message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters, node_topics, "/tf", qos, std::move(cb), tf_options);
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
      message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters, node_topics, "/tf", qos, std::move(cb), options);
      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters,
        node_topics,
        "/tf_static",
        static_qos,
        std::move(static_cb),
        static_options);
    }
  }

  // Overload of init() with the static_only flag
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
    bool static_only)
  {
    if (!static_only) {
      init(
        node_base,
        node_logging,
        node_parameters,
        node_topics,
        spin_thread,
        qos,
        static_qos,
        options,
        static_options);
      return;
    }

    spin_thread_ = spin_thread;
    node_base_interface_ = node_base;
    node_logging_interface_ = node_logging;

    using callback_t = std::function<void (tf2_msgs::msg::TFMessage::ConstSharedPtr)>;
    callback_t static_cb = std::bind(
      &FilteredTransformListener::subscription_callback, this, std::placeholders::_1, true);

    if (spin_thread_) {
      callback_group_ = node_base_interface_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_static_options = static_options;
      tf_static_options.callback_group = callback_group_;

      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node_parameters,
        node_topics,
        "/tf_static",
        static_qos,
        std::move(static_cb),
        tf_static_options);

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
      buffer_.setUsingDedicatedThread(true);
    } else {
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
  const std::vector<FilterRule> filter_rules_;
  tf2::TimePoint last_update_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ {nullptr};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
};

}  // namespace tf

}  // namespace as2

#endif  // AS2_CORE__UTILS__FILTERED_TRANSFORM_LISTENER_HPP_
