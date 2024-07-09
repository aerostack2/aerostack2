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
* @file behavior_utils.hpp
*
* @brief Class definition for behavior utils
*
* @author Miguel Fernández Cortizas
*         Pedro Arias Pérez
*         David Pérez Saura
*         Rafael Pérez Seguí
*/

#ifndef AS2_BEHAVIOR__BEHAVIOR_UTILS_HPP_
#define AS2_BEHAVIOR__BEHAVIOR_UTILS_HPP_

#include <string>
#include <as2_msgs/msg/behavior_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace as2_behavior
{

// enum class BehaviorStatus { IDLE, RUNNING, PAUSED };
enum class ExecutionStatus { SUCCESS, RUNNING, FAILURE, ABORTED };

/* template <typename actionT>
using start_srv = typename actionT::Impl::SendGoalService;
template <typename actionT>
using modify_srv = start_srv<actionT>;
template <typename actionT>
using result_srv = typename actionT::Impl::GetResultService;
template <typename actionT>
using feedback_msg = typename actionT::Impl::FeedbackMessage;
template <typename actionT>
using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
template <typename actionT>
using cancel_srv = typename actionT::Impl::CancelGoalService; */

}  // namespace as2_behavior
#endif  // AS2_BEHAVIOR__BEHAVIOR_UTILS_HPP_
