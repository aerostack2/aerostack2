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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file executor_thread_util.hpp
 *
 * Class to add executor thread to the node
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef AS2_CORE__TESTS__MOCKS__EXECUTOR_THREAD_UTIL__EXECUTOR_THREAD_UTIL_HPP_
#define AS2_CORE__TESTS__MOCKS__EXECUTOR_THREAD_UTIL__EXECUTOR_THREAD_UTIL_HPP_

#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

namespace as2
{

namespace mock
{

/**
 * @brief Class to add executor thread to the node
 *
 * @tparam ExecutorT Executor type
*/
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ExecutorThreadUtil
{
public:
  /**
   * @brief Constructor
   *
   * @param executor Executor to spin
   * @param frequency Frequency to spin the executor
   */
  explicit ExecutorThreadUtil(std::shared_ptr<ExecutorT> executor, const double frequency = 100.0)
  : frequency_(frequency),
    executor_(executor)
  {
  }

  /**
   * @brief Destructor
   */
  ~ExecutorThreadUtil()
  {
    stop();
  }

  /**
   * @brief Start the executor thread
   */
  void start()
  {
    running_ = true;
    executor_thread_ = std::thread(&ExecutorThreadUtil::thread_function, this);
  }

  /**
   * @brief Stop the executor thread
   */
  void stop()
  {
    running_ = false;
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

private:
  double frequency_;
  std::shared_ptr<ExecutorT> executor_;
  std::thread executor_thread_;
  std::atomic<bool> running_;

  /**
   * @brief Thread function to spin the executor
   */
  void thread_function()
  {
    rclcpp::Rate rate(frequency_);
    executor_->spin_some();
    while (rclcpp::ok() && running_) {
      executor_->spin_some();
      rate.sleep();
    }
  }
};  // class ExecutorThreadUtil

}  // namespace mock
}  // namespace as2

#endif  // AS2_CORE__TESTS__MOCKS__EXECUTOR_THREAD_UTIL__EXECUTOR_THREAD_UTIL_HPP_
