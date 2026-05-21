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
 * @file follow_reference_plugin_trajectory.cpp
 *
 * Trajectory-mode follow_reference plugin. Delegates the actual reference
 * tracking to generate_polynomial_trajectory_behavior in follow_reference_mode
 * via its action server, and forwards goal modifications by publishing to the
 * modify_waypoint topic so the trajectory is updated in flight.
 *
 * @authors Rafael Perez-Segui
 */

#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "as2_msgs/action/generate_polynomial_trajectory.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "as2_msgs/msg/pose_stamped_with_id_array.hpp"
#include "follow_reference_behavior/follow_reference_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace follow_reference_plugin_trajectory
{

// Stable id used to refer to the single follow-reference waypoint
constexpr char kFollowReferenceWaypointId[] = "follow_reference_target";

class Plugin : public follow_reference_base::FollowReferenceBase
{
  using TrajectoryGeneratorAction =
    as2_msgs::action::GeneratePolynomialTrajectory;
  using GoalHandleTrajectoryGenerator =
    rclcpp_action::ClientGoalHandle<TrajectoryGeneratorAction>;

public:
  void ownInit() override
  {
    traj_gen_client_ = rclcpp_action::create_client<TrajectoryGeneratorAction>(
      node_ptr_, as2_names::actions::behaviors::trajectorygenerator);

    traj_gen_pause_client_ =
      std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
      std::string(as2_names::actions::behaviors::trajectorygenerator) +
      "/_behavior/pause",
      node_ptr_);

    traj_gen_resume_client_ =
      std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
      std::string(as2_names::actions::behaviors::trajectorygenerator) +
      "/_behavior/resume",
      node_ptr_);

    modify_pub_ =
      node_ptr_->create_publisher<as2_msgs::msg::PoseStampedWithIDArray>(
      as2_names::topics::motion_reference::modify_waypoint,
      as2_names::topics::motion_reference::qos_waypoint);

    traj_gen_goal_options_.feedback_callback =
      std::bind(
      &Plugin::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);
    traj_gen_goal_options_.result_callback =
      std::bind(&Plugin::result_callback, this, std::placeholders::_1);

    // Reactive-modify configuration. The plugin keeps the target anchored in
    // "earth" and re-emits modify_waypoint messages only when the live target
    // in earth has drifted more than `modify_threshold_` from the last
    // published one, and at most at `modify_frequency_` Hz.
    // Defaults: threshold=0 (any change triggers modify), frequency=0 (no
    // rate-limit). Raise them to filter TF noise or cap the modify rate.
    modify_threshold_ = declareAndGetDouble(
      "follow_reference_plugin_trajectory.modify_threshold", 0.0);
    modify_frequency_ = declareAndGetDouble(
      "follow_reference_plugin_trajectory.modify_frequency", 0.0);
  }

  bool own_activate(as2_msgs::action::FollowReference::Goal & _goal) override
  {
    if (!traj_gen_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Trajectory generator action server not available");
      return false;
    }

    // Resolve the target in "earth" upfront. The trajectory generator then
    // works with a static-frame waypoint and the reactive-modify loop in
    // own_run() detects drift by re-evaluating the original target in earth.
    geometry_msgs::msg::PointStamped target_in_earth;
    if (!tryConvertTargetToEarth(_goal.target_pose, target_in_earth)) {
      return false;
    }

    // Reset internal state for a fresh activation.
    traj_gen_goal_accepted_ = false;
    traj_gen_result_received_ = false;
    traj_gen_result_ = false;
    cancel_requested_ = false;
    last_target_in_earth_ = target_in_earth;
    last_modify_time_ = node_ptr_->now();

    auto traj_generator_goal =
      followReferenceGoalToTrajectoryGoal(_goal, target_in_earth);

    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "FollowReference[trajectory] target (earth): %f, %f, %f",
      traj_generator_goal.path[0].pose.pose.position.x,
      traj_generator_goal.path[0].pose.pose.position.y,
      traj_generator_goal.path[0].pose.pose.position.z);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "FollowReference[trajectory] yaw mode: %d, max_speed: %.3f",
      traj_generator_goal.yaw.mode,
      traj_generator_goal.max_speed);

    traj_gen_goal_handle_future_ = traj_gen_client_->async_send_goal(
      traj_generator_goal, traj_gen_goal_options_);
    if (!traj_gen_goal_handle_future_.valid()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "FollowReference[trajectory]: request could not be sent");
      return false;
    }
    return true;
  }

  bool own_modify(as2_msgs::action::FollowReference::Goal & _goal) override
  {
    // Convert to earth before publishing so the trajectory generator always
    // sees a static-frame waypoint. The reactive-modify loop in own_run()
    // keeps the cache (last_target_in_earth_) coherent so frame-drift
    // updates are not duplicated.
    geometry_msgs::msg::PointStamped target_in_earth;
    if (!tryConvertTargetToEarth(_goal.target_pose, target_in_earth)) {
      return false;
    }
    publishModifyInEarth(target_in_earth);

    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "FollowReference[trajectory] modify target (earth): %f, %f, %f",
      target_in_earth.point.x, target_in_earth.point.y,
      target_in_earth.point.z);
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(
      node_ptr_->get_logger(), "FollowReference[trajectory] cancel");
    cancel_requested_ = true;
    if (traj_gen_goal_handle_future_.valid()) {
      // Same cancel pattern as the other trajectory plugins
      // (go_to / follow_path / takeoff / land): resolve the goal handle
      // future and request a per-goal async cancel.
      traj_gen_client_->async_cancel_goal(traj_gen_goal_handle_future_.get());
    }
    return true;
  }

  bool own_pause(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference[trajectory] paused");
    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;
    auto out = traj_gen_pause_client_->sendRequest(req, resp, 3);
    if (out && resp.success) {return true;}
    return false;
  }

  bool own_resume(const std::shared_ptr<std::string> & message) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference[trajectory] resumed");
    std_srvs::srv::Trigger::Request req;
    std_srvs::srv::Trigger::Response resp;
    auto out = traj_gen_resume_client_->sendRequest(req, resp, 3);
    if (out && resp.success) {return true;}
    return false;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus & state) override
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "FollowReference[trajectory] end");
    sendHover();
    traj_gen_goal_accepted_ = false;
    traj_gen_result_received_ = false;
    traj_gen_result_ = false;
    cancel_requested_ = false;
    return;
  }

  as2_behavior::ExecutionStatus own_run() override
  {
    // External cancel requested: report SUCCESS so the behavior server
    // emits an ABORTED state machine transition rather than FAILURE.
    if (cancel_requested_) {
      result_.follow_reference_success = true;
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!traj_gen_goal_accepted_) {
      if (traj_gen_goal_handle_future_.valid() &&
        traj_gen_goal_handle_future_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        auto goal_handle = traj_gen_goal_handle_future_.get();
        if (goal_handle) {
          RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Trajectory generator goal accepted");
          traj_gen_goal_accepted_ = true;
        } else {
          RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "Trajectory generator goal was rejected");
          result_.follow_reference_success = false;
          return as2_behavior::ExecutionStatus::FAILURE;
        }
      } else {
        auto & clk = *node_ptr_->get_clock();
        RCLCPP_INFO_THROTTLE(
          node_ptr_->get_logger(), clk, 5000,
          "Waiting for trajectory generator goal to be accepted");
        return as2_behavior::ExecutionStatus::RUNNING;
      }
    }

    if (traj_gen_result_received_) {
      // In follow_reference_mode the trajectory generator should not finish
      // by itself. Receiving a result while we have not cancelled means
      // something failed downstream.
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Trajectory generator returned a result while follow_reference was "
        "expected to keep running (success=%d). Reporting failure.",
        traj_gen_result_);
      result_.follow_reference_success = traj_gen_result_;
      return traj_gen_result_ ?
             as2_behavior::ExecutionStatus::SUCCESS :
             as2_behavior::ExecutionStatus::FAILURE;
    }

    // Reactive modify: re-evaluate the live target in earth. If it has
    // drifted more than modify_threshold_ from the last published one and
    // the rate-limit allows it, publish a fresh modify_waypoint message.
    // For static frames the converted point stays constant, the diff stays
    // below threshold, and nothing extra is published.
    reactiveModifyTick();

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void feedback_callback(
    GoalHandleTrajectoryGenerator::SharedPtr,
    const std::shared_ptr<const TrajectoryGeneratorAction::Feedback> feedback)
  {
    traj_gen_feedback_ = *feedback;
    return;
  }

  void result_callback(
    const GoalHandleTrajectoryGenerator::WrappedResult & result)
  {
    traj_gen_result_received_ = true;
    if (result.result) {
      traj_gen_result_ = result.result->trajectory_generator_success;
    } else {
      traj_gen_result_ = false;
    }
    return;
  }

private:
  std::shared_ptr<rclcpp_action::Client<TrajectoryGeneratorAction>>
  traj_gen_client_ = nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr
    traj_gen_pause_client_ = nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr
    traj_gen_resume_client_ = nullptr;
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr
    modify_pub_ = nullptr;
  rclcpp_action::Client<TrajectoryGeneratorAction>::SendGoalOptions
    traj_gen_goal_options_;
  std::shared_future<GoalHandleTrajectoryGenerator::SharedPtr>
  traj_gen_goal_handle_future_;
  TrajectoryGeneratorAction::Feedback traj_gen_feedback_;

  bool traj_gen_goal_accepted_ = false;
  bool traj_gen_result_received_ = false;
  bool traj_gen_result_ = false;
  bool cancel_requested_ = false;

  // Reactive-modify configuration and state. Defaults: 0 / 0 = trigger
  // modify on every detected change, no rate-limit.
  double modify_threshold_ = 0.0;
  double modify_frequency_ = 0.0;
  geometry_msgs::msg::PointStamped last_target_in_earth_;
  rclcpp::Time last_modify_time_;

  double declareAndGetDouble(
    const std::string & param_name, double default_value)
  {
    if (!node_ptr_->has_parameter(param_name)) {
      node_ptr_->declare_parameter<double>(param_name, default_value);
    }
    return node_ptr_->get_parameter(param_name).as_double();
  }

  bool tryConvertTargetToEarth(
    const geometry_msgs::msg::PointStamped & target,
    geometry_msgs::msg::PointStamped & out)
  {
    // Refresh the stamp so the TF lookup uses the most recent transform
    geometry_msgs::msg::PointStamped stamped = target;
    stamped.header.stamp = node_ptr_->now();
    try {
      out = tf_handler_->convert(stamped, "earth");
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "FollowReference[trajectory]: could not transform target from '%s' "
        "to 'earth': %s",
        target.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  void publishModifyInEarth(
    const geometry_msgs::msg::PointStamped & target_in_earth)
  {
    as2_msgs::msg::PoseStampedWithIDArray msg;
    msg.poses.resize(1);
    auto & wp = msg.poses.front();
    wp.id = kFollowReferenceWaypointId;
    wp.pose.header = target_in_earth.header;
    wp.pose.pose.position.x = target_in_earth.point.x;
    wp.pose.pose.position.y = target_in_earth.point.y;
    wp.pose.pose.position.z = target_in_earth.point.z;
    wp.pose.pose.orientation.w = 1.0;  // Identity; the trajectory generator
                                       // uses YawMode, not pose orientation.
    modify_pub_->publish(msg);

    last_target_in_earth_ = target_in_earth;
    last_modify_time_ = node_ptr_->now();
  }

  void reactiveModifyTick()
  {
    // Rate-limit gate.
    if (modify_frequency_ > 0.0) {
      const double min_period = 1.0 / modify_frequency_;
      if ((node_ptr_->now() - last_modify_time_).seconds() < min_period) {
        return;
      }
    }

    // Refresh the stamp so the TF lookup uses the latest transform
    geometry_msgs::msg::PointStamped target = goal_.target_pose;
    target.header.stamp = node_ptr_->now();
    geometry_msgs::msg::PointStamped current_in_earth;
    try {
      current_in_earth = tf_handler_->convert(target, "earth");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
        "FollowReference[trajectory]: TF lookup failed during reactive "
        "modify: %s",
        ex.what());
      return;
    }

    const double dx = current_in_earth.point.x - last_target_in_earth_.point.x;
    const double dy = current_in_earth.point.y - last_target_in_earth_.point.y;
    const double dz = current_in_earth.point.z - last_target_in_earth_.point.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > modify_threshold_) {
      publishModifyInEarth(current_in_earth);
    }
  }

  // The trajectory generator only accepts a single scalar max_speed. Use the
  // maximum of the three axis-aligned limits provided by FollowReference to
  // get the closest behavior to the requested envelope (the generator
  // distributes that single speed across the polynomial axes internally).
  TrajectoryGeneratorAction::Goal followReferenceGoalToTrajectoryGoal(
    const as2_msgs::action::FollowReference::Goal & _goal,
    const geometry_msgs::msg::PointStamped & target_in_earth)
  {
    TrajectoryGeneratorAction::Goal out;
    out.stamp = node_ptr_->now();
    out.yaw = _goal.yaw;
    out.max_speed = static_cast<float>(
      std::max({_goal.max_speed_x, _goal.max_speed_y, _goal.max_speed_z}));
    out.start_on_paused = false;
    out.follow_reference_mode = true;

    as2_msgs::msg::PoseStampedWithID wp;
    wp.id = kFollowReferenceWaypointId;
    wp.pose.header = target_in_earth.header;
    wp.pose.pose.position.x = target_in_earth.point.x;
    wp.pose.pose.position.y = target_in_earth.point.y;
    wp.pose.pose.position.z = target_in_earth.point.z;
    wp.pose.pose.orientation.w = 1.0;
    out.path.emplace_back(wp);
    return out;
  }
};  // Plugin class
}  // namespace follow_reference_plugin_trajectory

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  follow_reference_plugin_trajectory::Plugin,
  follow_reference_base::FollowReferenceBase)
