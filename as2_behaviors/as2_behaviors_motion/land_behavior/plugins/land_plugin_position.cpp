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
 * @file land_plugin_position.cpp
 *
 * This file contains the implementation of the land behavior position plugin.
 *
 * The plugin commands a position reference with a very negative target z while
 * keeping the activation x, y and yaw. Land is considered finished when both
 * |vz| stays below a velocity threshold AND the absolute altitude is below an
 * altitude threshold for a parametrized amount of time.
 *
 * @authors Rafael Perez-Segui
 */

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_motion_reference_handlers/position_motion.hpp"
#include "land_behavior/land_base.hpp"

namespace land_plugin_position
{

class Plugin : public land_base::LandBase
{
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

  // Parameters (loaded in ownInit)
  double land_trajectory_height_ = -10.0;        // target z in earth frame [m]
  double land_speed_condition_percentage_ = 0.2;  // |vz| < pct * |land_speed|
  double land_speed_condition_height_ = 0.2;      // absolute altitude threshold [m]
  double land_position_condition_time_ = 1.0;     // continuous time threshold [s]

  // Per-activation state
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::TwistStamped target_twist_;
  double velocity_threshold_ = 0.0;
  rclcpp::Time condition_start_time_;

public:
  void ownInit()
  {
    position_motion_handler_ =
      std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);

    // Target z (m, earth frame) for the position reference. Should be well
    // below the ground (e.g. -10 m) so that the platform keeps descending
    // until ground contact stops it.
    node_ptr_->declare_parameter<double>("land_trajectory_height");
    node_ptr_->get_parameter("land_trajectory_height", land_trajectory_height_);

    // Fraction of |land_speed| used as the |vz| threshold of the
    // finish-condition: velocity_threshold_ = |land_speed| * pct. The
    // smaller, the stricter "stopped descending" is interpreted.
    node_ptr_->declare_parameter<double>("land_speed_condition_percentage");
    node_ptr_->get_parameter("land_speed_condition_percentage", land_speed_condition_percentage_);

    // Absolute altitude threshold (m, earth frame). The finish condition
    // requires the current z to be below this value, guarding against
    // accepting "no descent" while still high in the air.
    node_ptr_->declare_parameter<double>("land_speed_condition_height");
    node_ptr_->get_parameter("land_speed_condition_height", land_speed_condition_height_);

    // Time (s) the slow + low conditions must hold continuously to declare
    // the land as finished. The internal timer resets any time either
    // condition fails.
    node_ptr_->declare_parameter<double>("land_position_condition_time");
    node_ptr_->get_parameter("land_position_condition_time", land_position_condition_time_);
  }

  bool own_activate(as2_msgs::action::Land::Goal & _goal) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", _goal.land_speed);

    // Snapshot the current pose (in earth frame) and rewrite z to the descent
    // target. Only header.stamp will change at each iteration in own_run().
    target_pose_ = actual_pose_;
    target_pose_.header.frame_id = "earth";
    target_pose_.pose.position.z = land_trajectory_height_;

    const double v_max = std::fabs(_goal.land_speed);
    target_twist_ = geometry_msgs::msg::TwistStamped();
    target_twist_.header.frame_id = "earth";
    target_twist_.twist.linear.x = v_max;
    target_twist_.twist.linear.y = v_max;
    target_twist_.twist.linear.z = v_max;

    velocity_threshold_ = v_max * land_speed_condition_percentage_;
    condition_start_time_ = node_ptr_->now();

    RCLCPP_INFO(
      node_ptr_->get_logger(), "Land to position: %f, %f, %f",
      target_pose_.pose.position.x, target_pose_.pose.position.y,
      target_pose_.pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land velocity limit: %f", v_max);
    RCLCPP_INFO(node_ptr_->get_logger(), "Land velocity threshold: %f", velocity_threshold_);
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Land altitude threshold: %f", land_speed_condition_height_);
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Land condition time: %f", land_position_condition_time_);
    return true;
  }

  bool own_modify(as2_msgs::action::Land::Goal & _goal) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land goal modified");
    RCLCPP_INFO(node_ptr_->get_logger(), "Land with speed: %f", _goal.land_speed);
    const double v_max = std::fabs(_goal.land_speed);
    target_twist_.twist.linear.x = v_max;
    target_twist_.twist.linear.y = v_max;
    target_twist_.twist.linear.z = v_max;
    velocity_threshold_ = v_max * land_speed_condition_percentage_;
    condition_start_time_ = node_ptr_->now();
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land canceled, set to hover");
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land resumed");
    condition_start_time_ = node_ptr_->now();
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Land end");
    if (state != as2_behavior::ExecutionStatus::SUCCESS) {
      sendHover();
    }
    return;
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    if (checkGoalCondition()) {
      result_.land_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    const rclcpp::Time stamp = node_ptr_->now();
    target_pose_.header.stamp = stamp;
    target_twist_.header.stamp = stamp;

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(target_pose_, target_twist_)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "LAND PLUGIN: Error sending position command");
      result_.land_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  /**
   * @brief Decide whether the land action should be considered finished.
   *
   * The plugin commands a position reference well below the ground; the
   * platform stops descending in practice when it touches the floor and
   * the position controller can no longer reduce the altitude error.
   * Two conditions must hold **simultaneously and continuously** during
   * @c land_position_condition_time_ seconds for the action to succeed:
   *
   *   1. **Slow enough**: |vz| (vertical velocity in earth frame, taken
   *      from feedback_.actual_land_speed) is below velocity_threshold_,
   *      computed at activation as
   *      |land_speed| * land_speed_condition_percentage_. This detects
   *      that the descent has effectively stopped.
   *
   *   2. **Low enough**: the absolute altitude
   *      actual_pose_.pose.position.z is below
   *      land_speed_condition_height_. This guards against false positives
   *      right after activation, when |vz| has not yet built up but the
   *      drone is still high. It also enables success when land is
   *      requested with the drone already on the ground (z ~ 0, vz ~ 0).
   *
   * The "continuous" requirement is implemented by resetting
   * condition_start_time_ to "now" any time either condition fails. Only
   * after both conditions have remained true uninterruptedly for longer
   * than land_position_condition_time_ is true returned.
   *
   * If localization is not yet available (localization_flag_ == false),
   * the function returns false without touching the timer.
   *
   * @return true if land must be considered finished; false otherwise.
   */
  bool checkGoalCondition()
  {
    if (!localization_flag_) {
      return false;
    }
    const bool slow_enough = std::fabs(feedback_.actual_land_speed) < velocity_threshold_;
    const bool low_enough = actual_pose_.pose.position.z < land_speed_condition_height_;

    if (slow_enough && low_enough) {
      if ((node_ptr_->now() - condition_start_time_).seconds() > land_position_condition_time_) {
        return true;
      }
    } else {
      condition_start_time_ = node_ptr_->now();
    }
    return false;
  }
};  // Plugin class
}  // namespace land_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(land_plugin_position::Plugin, land_base::LandBase)
