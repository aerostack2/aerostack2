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
* @file logging.hpp
*
* Where the filter's log lines go, so that the library has something to say without
* knowing who is listening
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF_CORE__LOGGING_HPP_
#define SIMPLE_EKF_CORE__LOGGING_HPP_

#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "simple_ekf_core/types.hpp"

namespace simple_ekf_core
{

enum class LogLevel
{
  INFO,
  WARN,
  ERROR
};

/**
 * @brief Where a log line goes. An empty sink discards it.
 */
using LogSink = std::function<void (LogLevel, const std::string &)>;

/**
 * @brief printf-style formatting in front of a LogSink
 */
class Logger
{
public:
  explicit Logger(LogSink sink = nullptr)
  : sink_(std::move(sink))
  {
  }

  void log(LogLevel level, const char * format, ...) const
  __attribute__((format(printf, 3, 4)));

private:
  LogSink sink_;
};

/**
 * @brief Lets something through at most once per period
 *
 * The filter reads no clock, so the time a call is judged against is whatever time its
 * caller was given.
 */
class Throttle
{
public:
  explicit Throttle(Nanoseconds period)
  : period_(period)
  {
  }

  /**
   * @brief Whether a period has gone by since the last call that returned true.
   */
  bool allow(Nanoseconds now)
  {
    if (last_ && now - *last_ < period_) {
      return false;
    }
    last_ = now;
    return true;
  }

private:
  Nanoseconds period_;
  std::optional<Nanoseconds> last_;
};

}  // namespace simple_ekf_core

#endif  // SIMPLE_EKF_CORE__LOGGING_HPP_
