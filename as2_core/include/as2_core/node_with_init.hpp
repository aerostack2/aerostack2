/*!*******************************************************************************************
 *  \file       node.hpp
 *  \brief      Aerostack2 node header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef AEROSTACK2_NODE_WITH_HPP_
#define AEROSTACK2_NODE_WITH_HPP_

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/timer.hpp>
#include <string>

#include "as2_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Comment out next line to get LifecycleNode
#define AS2_RCLCPP_NODE

namespace as2 {

#ifdef AS2_RCLCPP_NODE
  typedef rclcpp::Node RosNode;
#else
#define AS2_LIFECYLCE_NODE
  typedef rclcpp_lifecycle::LifecycleNode RosNode;
#endif

/**
 * @brief Basic Aerostack2 Node, it heritages all the functionality of an rclcpp::Node
 */
class NodeWithInit {
protected:
  // Atributte to store Dependency Inyection (DI):
  RosNode* ros_node;

public:
  // typedef std::shared_ptr<as2::Node> SharedPtr;
  /**
   * @brief Construct a new Node object 
   * 
   * Empty construct, this class has two steps initialization, should be followed by init
   *
   */
  NodeWithInit() : ros_node(nullptr) {
  }

  virtual ~NodeWithInit() {
  }

  /**
   * @brief Initialize a Node Object (two steps initialization)
   *
   * @param node either a rclcpp::Node* or rclcpp_lifecycle::LifecycleNode*
   */
  virtual void init(RosNode* node) {
    ros_node = node;
    RCLCPP_INFO(ros_node->get_logger(), "Construct with name [%s] and namespace [%s]", ros_node->get_name(),
                ros_node->get_namespace());
    ros_node->declare_parameter<float>("node_frequency", -1.0);
    ros_node->get_parameter("node_frequency", loop_frequency_);
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] base frequency= %f", ros_node->get_name(),
                 loop_frequency_);
    if (loop_frequency_ > 0.0) {
      loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
    }
  }


#ifdef AS2_LIFECYLCE_NODE
  template <typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>> create_publisher(
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options =
          rclcpp::PublisherOptionsWithAllocator<AllocatorT>()) {
    using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;
    // RCLCPP_DEBUG(this->get_logger(), "PUB %s", topic_name.c_str());
    std::shared_ptr<PublisherT> pub =
        rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(*this, topic_name, qos, options);
    pub->on_activate();
    return pub;
  }

#elif defined( AS2_RCLCPP_NODE )

public:
  void configure() { this->on_configure(rclcpp_lifecycle::State()); }
  void activate() { this->on_activate(rclcpp_lifecycle::State()); }
  void deactivate() { this->on_deactivate(rclcpp_lifecycle::State()); }
  void cleanup() { this->on_cleanup(rclcpp_lifecycle::State()); }
  void shutdown() { this->on_shutdown(rclcpp_lifecycle::State()); }
  void error() { this->on_error(rclcpp_lifecycle::State()); }
#endif

  /**
   * @brief transform an string into local topic name inside drone namespace and node namespace
   *
   * @param name source string
   * @return std::string  result name
   */
  std::string generate_local_name(const std::string &name);

  /**
   * @brief transform an string into global topic name inside drone namespace
   *
   * @param name source string
   * @return std::string result name
   */
  std::string generate_global_name(const std::string &name);

protected:
  /**
   * @brief Callback for the activate state
   * @param state
   * @return CallbackReturn
   *
   */

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] on_activate", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

  /**
   * @brief Callback for the deactivate state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] on_deactivate", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

  /**
   * @brief Callback for the configure state
   * @param state
   * @return CallbackReturn
   */

  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] on_configure", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

  /**
   * @brief Callback for the cleanup state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] on_cleanup", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

  /**
   * @brief Callback for the shutdown state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_DEBUG(ros_node->get_logger(), "node [%s] on_shutdown", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

  /**
   * @brief Callback for the error state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
    RCLCPP_ERROR(ros_node->get_logger(), "node [%s] on_error", ros_node->get_name());
    return CallbackReturn::SUCCESS;
  };

private:
  /**
   * @brief frequency of the spin cycle of the node
   */
  double loop_frequency_;
  std::shared_ptr<as2::Rate> loop_rate_ptr_;

public:
  /**
   * @brief create a timer with the node clock
   *
   * @return rclcpp::TimerBase::SharedPtr rclcpp timer using node clock
   */
  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr create_timer(std::chrono::duration<DurationRepT, DurationT> period,
                                            CallbackT callback,
                                            rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_timer(ros_node, ros_node->get_clock(), period, std::move(callback), group);
  }

  /**
   * @brief sleeps the node to ensure node_frecuency desired
   *
   * @return true the node is sleeping
   * @return false the node is not sleeping, this means that desired frequency is not reached
   */
  bool sleep() {
    if (loop_rate_ptr_) {
      return loop_rate_ptr_->sleep();
    } else {
      throw std::runtime_error("Node::sleep() called but no node_frequency defined");
      return false;
    };
  };

  /**
   * @brief Get the loop frequency object
   *
   * @return double frequency in Hz
   */
  inline double get_loop_frequency() { return loop_frequency_; }

  bool preset_loop_frequency(double frequency) {
    if (frequency <= 0) return true;  // default frequency is -1
    if (loop_rate_ptr_) {
      RCLCPP_INFO(ros_node->get_logger(),
                  "Preset Loop Frequency [%d Hz] was overwrite by launcher params to %d",
                  (int)frequency, (int)loop_frequency_);
      return false;
    }
    loop_frequency_ = frequency;
    loop_rate_ptr_  = std::make_shared<Rate>(loop_frequency_);
    return true;
  };
};  // namespace as2

}  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
