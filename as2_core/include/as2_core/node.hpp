// Copyright 2023 Universidad Politécnica de Madrid
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
 *  \file       node.hpp
 *  \brief      Aerostack2 node header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 ********************************************************************************/

#ifndef AS2_CORE__NODE_HPP_
#define AS2_CORE__NODE_HPP_

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/timer.hpp>

#include "as2_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#define AS2_RCLCPP_NODE 1
#define AS2_LIFECYLCE_NODE 2

#define AS2_NODE_FATHER AS2_RCLCPP_NODE
// #define AS2_NODE_FATHER AS2_LIFECYLCE_NODE

#if AS2_NODE_FATHER == AS2_RCLCPP_NODE
#define AS2_NODE_FATHER_TYPE rclcpp::Node
#elif AS2_NODE_FATHER == AS2_LIFECYLCE_NODE
#define AS2_NODE_FATHER_TYPE rclcpp_lifecycle::LifecycleNode
#endif

namespace as2
{
/**
 * @brief Basic Aerostack2 Node, it heritages all the functionality of an rclcpp::Node
 */

class Node : public AS2_NODE_FATHER_TYPE
{
private:
  /**
   * @brief Read a parameter that is already declared, and log its value.
   *
   * @tparam T Parameter type.
   * @param name Parameter name.
   * @return Value of the parameter.
   */
  template<typename T>
  T readParameter(const std::string & name)
  {
    const rclcpp::Parameter parameter = this->get_parameter(name);
    RCLCPP_INFO(
      this->get_logger(), "[%s] = %s", name.c_str(), parameter.value_to_string().c_str());
    return parameter.template get_value<T>();
  }

  /**
   * @brief Read the node frequency parameter and create the loop rate.
   * Called by both constructors.
   */
  void init()
  {
    loop_frequency_ = getParameter<float>("node_frequency", -1.0);
    RCLCPP_DEBUG(
      this->get_logger(), "node [%s] base frequency= %f", this->get_name(), loop_frequency_);

    if (loop_frequency_ > 0.0) {
      loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
    }
  }

public:
  // typedef std::shared_ptr<as2::Node> SharedPtr;
  /**
   * @brief Construct a new Node object
   *
   * @param name Node name
   */

  Node(
    const std::string & name, const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : AS2_NODE_FATHER_TYPE(name, ns, options)
  {
    RCLCPP_INFO(
      this->get_logger(), "Construct with name [%s] and namespace [%s]", name.c_str(), ns.c_str());
    init();
  }

  /**
   * @brief Construct a new Node object, in the default namespace.
   *
   * @param name Node name.
   * @param options Node options.
   */
  explicit Node(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : AS2_NODE_FATHER_TYPE(name, options)
  {
    RCLCPP_INFO(this->get_logger(), "Construct with name [%s]", name.c_str());
    init();
  }

#if AS2_NODE_FATHER == AS2_LIFECYLCE_NODE
  /**
   * @brief Create a lifecycle publisher, already activated.
   *
   * The lifecycle node only publishes while it is active, and most aerostack2
   * nodes publish from construction, so the publisher is activated here.
   *
   * @tparam MessageT Message type.
   * @tparam AllocatorT Allocator type.
   * @param topic_name Topic to publish on.
   * @param qos Quality of service of the publisher.
   * @param options Publisher options.
   * @return Activated lifecycle publisher.
   */
  template<typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>> create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;
    // RCLCPP_DEBUG(this->get_logger(), "PUB %s", topic_name.c_str());
    std::shared_ptr<PublisherT> pub =
      rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(*this, topic_name, qos, options);
    pub->on_activate();
    return pub;
  }

#elif AS2_NODE_FATHER == AS2_RCLCPP_NODE

public:
  /**
   * @brief Trigger the configure transition from code, without a lifecycle client.
   */
  void configure() {this->on_configure(rclcpp_lifecycle::State());}
  /**
   * @brief Trigger the activate transition from code, without a lifecycle client.
   */
  void activate() {this->on_activate(rclcpp_lifecycle::State());}
  /**
   * @brief Trigger the deactivate transition from code, without a lifecycle client.
   */
  void deactivate() {this->on_deactivate(rclcpp_lifecycle::State());}
  /**
   * @brief Trigger the cleanup transition from code, without a lifecycle client.
   */
  void cleanup() {this->on_cleanup(rclcpp_lifecycle::State());}
  /**
   * @brief Trigger the shutdown transition from code, without a lifecycle client.
   */
  void shutdown() {this->on_shutdown(rclcpp_lifecycle::State());}
  /**
   * @brief Trigger the error transition from code, without a lifecycle client.
   */
  void error() {this->on_error(rclcpp_lifecycle::State());}
#endif

  /**
   * @brief transform an string into local topic name inside drone namespace and node namespace
   *
   * @param name source string
   * @return std::string  result name
   */
  std::string generate_local_name(const std::string & name);

  /**
   * @brief transform an string into global topic name inside drone namespace
   *
   * @param name source string
   * @return std::string result name
   */
  /**
   * @brief Read an optional node parameter, declaring it with a default value
   * when it is not declared yet.
   *
   * @tparam T Parameter type, deduced from @p default_value.
   * @param name Parameter name.
   * @param default_value Value the parameter takes when nothing provides it.
   * @return Value of the parameter.
   */
  template<typename T>
  T getParameter(const std::string & name, const T & default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    return readParameter<T>(name);
  }

  /**
   * @brief Read a required node parameter, declaring it without a default value
   * when it is not declared yet.
   *
   * @tparam T Parameter type.
   * @param name Parameter name.
   * @return Value of the parameter.
   * @throw rclcpp::exceptions::ParameterUninitializedException when nothing
   *        provides the parameter.
   */
  template<typename T>
  T getParameter(const std::string & name)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name);
    }
    return readParameter<T>(name);
  }

  std::string generate_global_name(const std::string & name);

protected:
  /**
   * @brief Callback for the activate state
   * @param state
   * @return CallbackReturn
   *
   */

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Callback for the activate state.
   *
   * @return CallbackReturn::SUCCESS
   */
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_DEBUG(this->get_logger(), "node [%s] on_activate", this->get_name());
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for the deactivate state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_DEBUG(this->get_logger(), "node [%s] on_deactivate", this->get_name());
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for the configure state
   * @param state
   * @return CallbackReturn
   */

  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_DEBUG(this->get_logger(), "node [%s] on_configure", this->get_name());
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for the cleanup state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_DEBUG(this->get_logger(), "node [%s] on_cleanup", this->get_name());
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for the shutdown state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_DEBUG(this->get_logger(), "node [%s] on_shutdown", this->get_name());
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for the error state
   * @param state
   * @return CallbackReturn
   */
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State())
  {
    RCLCPP_ERROR(this->get_logger(), "node [%s] on_error", this->get_name());
    return CallbackReturn::SUCCESS;
  }

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
  template<typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr create_timer(
    std::chrono::duration<DurationRepT, DurationT> period, CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_timer(this, this->get_clock(), period, std::move(callback), group);
  }

  /**
   * @brief sleeps the node to ensure node_frecuency desired
   *
   * @return true the node is sleeping
   * @return false the node is not sleeping, this means that desired frequency is not reached
   */
  bool sleep()
  {
    if (loop_rate_ptr_) {
      return loop_rate_ptr_->sleep();
    } else {
      throw std::runtime_error("Node::sleep() called but no node_frequency defined");
    }
  }

  /**
   * @brief Get the loop frequency object
   *
   * @return double frequency in Hz
   */
  inline double get_loop_frequency() {return loop_frequency_;}

  /**
   * @brief Propose a loop frequency from code, as a default.
   *
   * The node_frequency parameter wins: a frequency set from the launch is kept
   * and this call is rejected.
   *
   * @param frequency Proposed frequency, in Hz. Values <= 0 are ignored.
   * @return true if the proposed frequency was taken.
   */
  bool preset_loop_frequency(double frequency)
  {
    if (frequency <= 0) {
      return true;  // default frequency is -1
    }
    if (loop_rate_ptr_) {
      RCLCPP_INFO(
        this->get_logger(), "Preset Loop Frequency [%d Hz] was overwrite by launcher params to %d",
        (int)frequency, (int)loop_frequency_);
      return false;
    }
    loop_frequency_ = frequency;
    loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
    return true;
  }
};  // namespace as2

}  // namespace as2

#endif  // AS2_CORE__NODE_HPP_
