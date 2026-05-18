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
 * @file follow_reference_plugin_position.cpp
 *
 * Position-control follow_reference plugin. Tracks the target point by sending
 * position setpoints to the platform, keeping the behavior in RUNNING state
 * indefinitely until cancelled.
 *
 * @authors Rafael Perez-Segui
 */

#include <tf2/exceptions.h>

#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>

#include "as2_motion_reference_handlers/position_motion.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"
#include "follow_reference_behavior/follow_reference_base.hpp"

namespace follow_reference_plugin_position
{
class Plugin : public follow_reference_base::FollowReferenceBase
{
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion>
  position_motion_handler_ = nullptr;

public:
  void ownInit() override
  {
    position_motion_handler_ =
      std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool own_activate(as2_msgs::action::FollowReference::Goal & _goal) override
  {
    if (!computeYaw(
        _goal.yaw.mode, _goal.target_pose.point,
        actual_pose_.pose.position, _goal.yaw.angle))
    {
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference position goal accepted");
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "FollowReference target: %f, %f, %f", _goal.target_pose.point.x,
      _goal.target_pose.point.y, _goal.target_pose.point.z);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "FollowReference max speed: %f, %f, %f", _goal.max_speed_x,
      _goal.max_speed_y, _goal.max_speed_z);
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference yaw: %f", _goal.yaw.angle);
    return true;
  }

  bool own_modify(as2_msgs::action::FollowReference::Goal & _goal) override
  {
    if (!computeYaw(
        _goal.yaw.mode, _goal.target_pose.point,
        actual_pose_.pose.position, _goal.yaw.angle))
    {
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference goal modified");
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference stopped");
    // Leave the drone in the last position
    goal_.target_pose.header.frame_id = "";
    sendHover();
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference paused");
    sendHover();
    return true;
  }

  bool own_resume(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference resumed");
    return true;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference position end");
    sendHover();
    return;
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    // Resolve the live target in "earth" so the published setpoint is
    // frame-agnostic w.r.t. the (possibly moving) goal frame. Publishing
    // directly in the goal frame causes the controller-side TF conversion
    // to drag any non-identity rotation from the goal frame into the
    // commanded orientation, which corrupts KEEP_YAW for moving targets
    // (e.g. moving_path TF). Working in "earth" decouples both axes.
    geometry_msgs::msg::PointStamped target_stamped = goal_.target_pose;
    target_stamped.header.stamp = node_ptr_->now();
    geometry_msgs::msg::PointStamped target_in_earth;
    try {
      target_in_earth = tf_handler_->convert(target_stamped, "earth");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
        "FOLLOW REFERENCE POSITION: TF lookup '%s' -> 'earth' failed: %s",
        goal_.target_pose.header.frame_id.c_str(), ex.what());
      return as2_behavior::ExecutionStatus::RUNNING;
    }

    // Refresh KEEP_YAW each cycle so the setpoint yaw always tracks the
    // drone's current yaw. The activation-time snapshot would otherwise
    // freeze stale data and drift if the controller introduces yaw error
    // (e.g. small initial rotation during go_to settle).
    float yaw_cmd = goal_.yaw.angle;
    if (goal_.yaw.mode == as2_msgs::msg::YawMode::KEEP_YAW) {
      yaw_cmd = getActualYaw();
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
        "earth",
        static_cast<float>(target_in_earth.point.x),
        static_cast<float>(target_in_earth.point.y),
        static_cast<float>(target_in_earth.point.z),
        yaw_cmd,
        "earth",
        goal_.max_speed_x, goal_.max_speed_y, goal_.max_speed_z))
    {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "FOLLOW REFERENCE POSITION: Error sending position command");
      result_.follow_reference_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }
    result_.follow_reference_success = true;
    return as2_behavior::ExecutionStatus::RUNNING;
  }

private:
  inline float getActualYaw()
  {
    return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
  }

  bool computeYaw(
    const uint8_t yaw_mode,
    const geometry_msgs::msg::Point & target,
    const geometry_msgs::msg::Point & actual,
    float & yaw)
  {
    switch (yaw_mode) {
      case as2_msgs::msg::YawMode::PATH_FACING: {
          Eigen::Vector2d diff(target.x - actual.x, target.y - actual.y);
          if (diff.norm() < 0.1) {
            RCLCPP_WARN_THROTTLE(
              node_ptr_->get_logger(),
              *node_ptr_->get_clock(), 1000,
              "Goal is too close to the current position in the plane, "
              "setting yaw_mode to KEEP_YAW");
            yaw = getActualYaw();
          } else {
            yaw = as2::frame::getVector2DAngle(diff.x(), diff.y());
          }
        } break;
      case as2_msgs::msg::YawMode::YAW_TO_FRAME:
        yaw = std::atan2(actual.y, actual.x);
        yaw = yaw + (yaw > 0 ? -M_PI : M_PI);
        break;
      case as2_msgs::msg::YawMode::FIXED_YAW:
        RCLCPP_DEBUG_THROTTLE(
          node_ptr_->get_logger(),
          *node_ptr_->get_clock(), 5000, "Yaw mode FIXED_YAW");
        break;
      case as2_msgs::msg::YawMode::KEEP_YAW:
        RCLCPP_DEBUG_THROTTLE(
          node_ptr_->get_logger(),
          *node_ptr_->get_clock(), 5000, "Yaw mode KEEP_YAW");
        yaw = getActualYaw();
        break;
      case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
        RCLCPP_ERROR(
          node_ptr_->get_logger(), "Yaw mode YAW_FROM_TOPIC, not supported");
        return false;
        break;
      default:
        RCLCPP_ERROR(
          node_ptr_->get_logger(), "Yaw mode %d not supported", yaw_mode);
        return false;
        break;
    }
    return true;
  }
};  // Plugin class
}  // namespace follow_reference_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  follow_reference_plugin_position::Plugin,
  follow_reference_base::FollowReferenceBase)
