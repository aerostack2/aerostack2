// Copyright 2025 UNIVERSIDAD POLITÉCNICA DE MADRID
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
//    * Neither the name of the UNIVERSIDAD POLITÉCNICA DE MADRID nor the names of its
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
 * @file common.hpp
 *
 * Common utilities
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef AS2_USB_CAMERA_INTERFACE__COMMON_HPP_
#define AS2_USB_CAMERA_INTERFACE__COMMON_HPP_

#include <string>
#include <queue>
#include <mutex>
#include <utility>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <as2_core/node.hpp>

namespace as2_usb_camera_interface
{

template<typename T>
std::string paramToString(const T & value)
{
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

template<typename T>
std::string paramToString(const std::vector<T> & vec)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i + 1 < vec.size()) {
      oss << ", ";
    }
  }
  oss << "]";
  return oss.str();
}

template<typename T>
T getParameter(as2::Node * node_ptr, const std::string & param_name)
{
  T param_value;
  try {
    if (!node_ptr->has_parameter(param_name)) {
      param_value = node_ptr->declare_parameter<T>(param_name);
    } else {
      node_ptr->get_parameter(param_name, param_value);
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      node_ptr->get_logger(), "Launch argument <%s> not defined or malformed: %s",
      param_name.c_str(), e.what());
    node_ptr->~Node();
  }

  const std::string value_str = paramToString(param_value);
  RCLCPP_INFO(node_ptr->get_logger(), "%s = %s", param_name.c_str(), value_str.c_str());
  return param_value;
}

// Thread-safe queue with max size
template<typename T>
class MutexQueue
{
public:
  explicit MutexQueue(size_t max_size = 10)
  : max_size_(max_size) {}

  /**
   * @brief Constructor that reads max_size from ROS2 parameters
   */
  MutexQueue(
    as2::Node * node_ptr,
    const std::string & param_base_name,
    size_t default_size = 10)
  {
    std::string param_name = param_base_name + "_queue_size";
    int size_param = getParameter<int>(node_ptr, param_name);
    max_size_ = (size_param > 0) ? static_cast<size_t>(size_param) : default_size;
  }

  // Push with drop policy if full
  bool push(const T & item, bool drop_if_full = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= max_size_) {
      if (drop_if_full) {
        queue_.pop();  // Always drop oldest when full
      } else {
        return false;  // Queue full, don't add
      }
    }
    queue_.push(item);
    return true;
  }

  // Try to pop (non-blocking)
  bool tryPop(T & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    item = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  // Get current size
  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  // Check if empty
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  size_t max_size_;
};

}  // namespace as2_usb_camera_interface

#endif  // AS2_USB_CAMERA_INTERFACE__COMMON_HPP_
