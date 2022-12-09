#ifndef __AS2_BEHAVIOR_UTILS_HPP__
#define __AS2_BEHAVIOR_UTILS_HPP__

#include <as2_msgs/msg/behavior_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace as2_behavior {

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
#endif
