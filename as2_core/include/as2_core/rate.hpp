/*!*******************************************************************************************
 *  \file       rate.hpp
 *  \brief      ROS2 rate class modified to work with the as2_core namespace and add more
 *functionalities. \authors    Miguel Fernández Cortizas
 *
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

#ifndef _AS2_RATE_HPP_
#define _AS2_RATE_HPP_

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/macros.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace as2 {
namespace rate {
class RateBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(RateBase)

  virtual bool sleep()                                     = 0;
  virtual bool is_steady() const                           = 0;
  virtual void reset()                                     = 0;
  virtual void set_period(std::chrono::nanoseconds period) = 0;
};

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

template <class Clock = std::chrono::high_resolution_clock>
class GenericRate : public RateBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericRate)

  explicit GenericRate(double rate)
      : GenericRate<Clock>(duration_cast<nanoseconds>(duration<double>(1.0 / rate))) {}
  explicit GenericRate(std::chrono::nanoseconds period)
      : period_(period), last_interval_(Clock::now()) {}

  virtual bool sleep() {
    // Time coming into sleep
    auto now = Clock::now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= std::chrono::seconds(0)) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    rclcpp::sleep_for(time_to_sleep);
    return true;
  }

  virtual void set_period(double rate) {
    period_ = duration_cast<nanoseconds>(duration<double>(1.0 / rate));
  }

  virtual void set_period(std::chrono::nanoseconds period) { period_ = period; }

  virtual bool is_steady() const { return Clock::is_steady; }

  virtual void reset() { last_interval_ = Clock::now(); }

  std::chrono::nanoseconds period() const { return period_; }

private:
  RCLCPP_DISABLE_COPY(GenericRate)

  std::chrono::nanoseconds period_;
  using ClockDurationNano = std::chrono::duration<typename Clock::rep, std::nano>;
  std::chrono::time_point<Clock, ClockDurationNano> last_interval_;
};

}  // namespace rate
using Rate     = as2::rate::GenericRate<std::chrono::system_clock>;
using WallRate = as2::rate::GenericRate<std::chrono::steady_clock>;
}  // namespace as2

#endif  // _AS2_RATE_HPP_
