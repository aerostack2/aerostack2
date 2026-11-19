// Copyright 2026 Vishwa
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
//    * Neither the name of the copyright holder nor the names of its
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
* @file flight_governor.cpp
*
* ROS 2 node that governs motion references between the behaviors and the
* motion controller.
*
* @author Vishwa
*/

#include "as2_flight_governor/flight_governor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "as2_core/names/topics.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_flight_governor/policy_loader.hpp"

namespace as2_flight_governor
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

geometry_msgs::msg::Vector3 toMsg(const Vec3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

Vec3 fromMsg(const geometry_msgs::msg::Point & p) {return Vec3{p.x, p.y, p.z};}
Vec3 fromMsg(const geometry_msgs::msg::Vector3 & v) {return Vec3{v.x, v.y, v.z};}

geometry_msgs::msg::Point toPoint(const Vec3 & v)
{
  geometry_msgs::msg::Point out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

/// A controller speed limit of 0 or below means "no limit" on that axis.
double limitToCore(double value)
{
  if (std::isnan(value)) {return value;}  // The governor vetoes it.
  return (value <= 0.0 || std::isinf(value)) ? kInf : value;
}

void appendUnique(std::vector<std::string> & list, const std::vector<std::string> & items)
{
  for (const auto & item : items) {
    bool found = false;
    for (const auto & existing : list) {
      found = found || existing == item;
    }
    if (!found) {list.push_back(item);}
  }
}

}  // namespace

FlightGovernor::FlightGovernor(const rclcpp::NodeOptions & options)
: as2::Node("flight_governor", options)
{
  const std::string policies_file = this->getParameter<std::string>("policies_file", "");
  if (policies_file.empty()) {
    throw std::runtime_error("flight governor: parameter 'policies_file' is required");
  }

  GovernorConfig config;
  config.state_timeout = this->getParameter<double>("state_timeout", 0.5);
  config.latch_veto = this->getParameter<bool>("latch_veto", true);
  config.hold_speed = this->getParameter<double>("hold_speed", 0.5);
  if (!(config.state_timeout > 0.0) || !(config.hold_speed > 0.0)) {
    throw std::runtime_error(
            "flight governor: 'state_timeout' and 'hold_speed' must be positive");
  }
  veto_alert_ = this->getParameter<int>("veto_alert", as2_msgs::msg::AlertEvent::INFO_ALERT);
  if (veto_alert_ < as2_msgs::msg::AlertEvent::EMERGENCY_LAND ||
    veto_alert_ > as2_msgs::msg::AlertEvent::FORCE_LAND)
  {
    throw std::runtime_error("flight governor: 'veto_alert' is not an AlertEvent value");
  }
  policy_frame_ = this->getParameter<std::string>("policy_frame", "");
  if (policy_frame_.empty()) {
    policy_frame_ = this->getOdomFrameId();
  }

  governor_ = std::make_unique<Governor>(loadPoliciesFromFile(policies_file), config);
  RCLCPP_INFO(
    this->get_logger(), "Loaded %zu policies in frame '%s':",
    governor_->policies().size(), policy_frame_.c_str());
  for (const auto & policy : governor_->policies()) {
    RCLCPP_INFO(
      this->get_logger(), "  - %s (%s)", policy->name().c_str(), policy->type().c_str());
  }
  if (governor_->policies().empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No policies loaded: only stale or invalid state and references will be vetoed");
  }

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  namespace topics = as2_names::topics;
  const std::string out = kOutputPrefix;

  state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    topics::self_localization::pose, topics::self_localization::qos,
    std::bind(&FlightGovernor::stateCallback, this, std::placeholders::_1));
  controller_info_sub_ = this->create_subscription<as2_msgs::msg::ControllerInfo>(
    topics::controller::info, topics::controller::qos_info,
    std::bind(&FlightGovernor::controllerInfoCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    topics::motion_reference::pose, topics::motion_reference::qos,
    std::bind(&FlightGovernor::poseCallback, this, std::placeholders::_1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    topics::motion_reference::twist, topics::motion_reference::qos,
    std::bind(&FlightGovernor::twistCallback, this, std::placeholders::_1));
  trajectory_sub_ = this->create_subscription<as2_msgs::msg::TrajectorySetpoints>(
    topics::motion_reference::trajectory, topics::motion_reference::qos,
    std::bind(&FlightGovernor::trajectoryCallback, this, std::placeholders::_1));
  thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    topics::motion_reference::thrust, topics::motion_reference::qos,
    std::bind(&FlightGovernor::thrustCallback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    out + topics::motion_reference::pose, topics::motion_reference::qos);
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    out + topics::motion_reference::twist, topics::motion_reference::qos);
  trajectory_pub_ = this->create_publisher<as2_msgs::msg::TrajectorySetpoints>(
    out + topics::motion_reference::trajectory, topics::motion_reference::qos);
  thrust_pub_ = this->create_publisher<as2_msgs::msg::Thrust>(
    out + topics::motion_reference::thrust, topics::motion_reference::qos);

  verdict_pub_ = this->create_publisher<as2_msgs::msg::GovernorVerdict>(
    out + "verdict", rclcpp::QoS(10));
  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
    topics::global::alert_event, topics::global::qos);

  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
    out + "reset",
    std::bind(
      &FlightGovernor::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

  wiring_timer_ = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&FlightGovernor::checkWiring, this));
}

// ---------------------------------------------------------------------------
// Inputs that are not references

void FlightGovernor::stateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose = *msg;
  if (!toPolicyFrame(pose)) {
    return;  // The state goes stale, and the governor vetoes.
  }
  last_pose_ = pose;
  governor_->updateState(State{fromMsg(pose.pose.position), stampSeconds(msg->header.stamp)});
}

void FlightGovernor::controllerInfoCallback(
  const as2_msgs::msg::ControllerInfo::SharedPtr msg)
{
  input_mode_ = msg->input_control_mode;
}

void FlightGovernor::resetCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const bool was_latched = governor_->reset();
  was_vetoing_ = false;
  response->success = true;
  response->message = was_latched ? "latched veto cleared" : "no veto was latched";
  RCLCPP_INFO(this->get_logger(), "Reset: %s", response->message.c_str());
}

// ---------------------------------------------------------------------------
// References, dispatched on the controller's input control mode

void FlightGovernor::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  using as2_msgs::msg::ControlMode;
  if (!input_mode_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No controller info yet: holding back pose references");
    return;
  }
  switch (input_mode_->control_mode) {
    case ControlMode::POSITION:
      governPositionPose(*msg);
      break;
    case ControlMode::SPEED:
    case ControlMode::TRAJECTORY:
    case ControlMode::HOVER:
      // The pose only carries a yaw angle in these modes: nothing to govern.
      pose_pub_->publish(*msg);
      break;
    default:
      passThrough(*msg, pose_pub_, "pose", "control mode is not governed");
  }
}

void FlightGovernor::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  using as2_msgs::msg::ControlMode;
  if (!input_mode_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No controller info yet: holding back twist references");
    return;
  }
  switch (input_mode_->control_mode) {
    case ControlMode::POSITION:
      governSpeedLimit(*msg);
      break;
    case ControlMode::SPEED:
      governVelocity(*msg);
      break;
    case ControlMode::HOVER:
      twist_pub_->publish(*msg);
      break;
    default:
      passThrough(*msg, twist_pub_, "twist", "control mode is not governed");
  }
}

void FlightGovernor::thrustCallback(const as2_msgs::msg::Thrust::SharedPtr msg)
{
  passThrough(*msg, thrust_pub_, "thrust", "thrust references are not governed");
}

void FlightGovernor::trajectoryCallback(
  const as2_msgs::msg::TrajectorySetpoints::SharedPtr msg)
{
  using as2_msgs::msg::ControlMode;
  if (!input_mode_ || input_mode_->control_mode != ControlMode::TRAJECTORY) {
    if (!input_mode_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No controller info yet: holding back trajectory references");
      return;
    }
    passThrough(*msg, trajectory_pub_, "trajectory", "control mode is not governed");
    return;
  }
  if (msg->setpoints.empty()) {
    trajectory_pub_->publish(*msg);
    return;
  }

  as2_msgs::msg::TrajectorySetpoints in = *msg;
  const bool tf_ok = in.header.frame_id == policy_frame_ ||
    tf_handler_->tryConvert(in, policy_frame_);
  const double now = nowSeconds();

  Decision combined;
  auto out_points = in.setpoints;
  for (std::size_t i = 0; i < in.setpoints.size(); ++i) {
    Command cmd;
    cmd.kind = Command::Kind::TRAJECTORY;
    cmd.position = fromMsg(in.setpoints[i].position);
    cmd.velocity = fromMsg(in.setpoints[i].twist);
    const Decision d = tf_ok ? governor_->evaluate(cmd, now) : governor_->veto(
      cmd, "cannot transform trajectory from '" + msg->header.frame_id + "' to '" +
      policy_frame_ + "'");
    if (i == 0) {
      combined.requested = d.requested;
      combined.allowed = d.allowed;
    }
    if (d.verdict == Verdict::VETO) {
      combined = d;
      break;
    }
    if (d.verdict == Verdict::CLAMP) {
      combined.verdict = Verdict::CLAMP;
      appendUnique(combined.policies, d.policies);
      if (combined.reason.empty()) {
        combined.reason = d.reason + " (setpoint " + std::to_string(i) + ")";
      }
      out_points[i].position = toMsg(d.allowed.position);
      out_points[i].twist = toMsg(d.allowed.velocity);
      // The original feed-forward acceleration no longer matches the clamped
      // velocity, so it is dropped.
      out_points[i].acceleration = geometry_msgs::msg::Vector3();
    }
  }

  bool sent = true;
  if (combined.verdict == Verdict::ALLOW) {
    trajectory_pub_->publish(*msg);
  } else if (combined.verdict == Verdict::CLAMP) {
    in.setpoints = out_points;
    trajectory_pub_->publish(in);
  } else if (combined.allowed_valid) {
    as2_msgs::msg::TrajectorySetpoints hold;
    hold.header.stamp = msg->header.stamp;
    hold.header.frame_id = policy_frame_;
    as2_msgs::msg::TrajectoryPoint point;
    point.id = "flight_governor_hold";
    point.position = toMsg(combined.allowed.position);
    point.yaw_angle = last_pose_ ?
      as2::frame::getYawFromQuaternion(last_pose_->pose.orientation) :
      msg->setpoints.front().yaw_angle;
    hold.setpoints.push_back(point);
    trajectory_pub_->publish(hold);
  } else {
    sent = false;
  }
  std_msgs::msg::Header header = msg->header;
  header.frame_id = policy_frame_;
  report(combined, header, "trajectory", true, sent);
}

void FlightGovernor::governPositionPose(const geometry_msgs::msg::PoseStamped & msg)
{
  geometry_msgs::msg::PoseStamped in = msg;
  const bool tf_ok = toPolicyFrame(in);

  Command cmd;
  cmd.kind = Command::Kind::POSE;
  cmd.position = fromMsg(in.pose.position);
  const Decision d = tf_ok ? governor_->evaluate(cmd, nowSeconds()) : governor_->veto(
    cmd, "cannot transform pose from '" + msg.header.frame_id + "' to '" + policy_frame_ + "'");

  bool sent = true;
  if (d.verdict == Verdict::ALLOW) {
    pose_pub_->publish(msg);  // Untouched: ALLOW is an exact pass-through.
  } else if (d.allowed_valid) {
    geometry_msgs::msg::PoseStamped out = in;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = policy_frame_;
    out.pose.position = toPoint(d.allowed.position);
    if (d.verdict == Verdict::VETO && last_pose_) {
      out.pose.orientation = last_pose_->pose.orientation;
    }
    pose_pub_->publish(out);
  } else {
    sent = false;
  }
  std_msgs::msg::Header header = msg.header;
  header.frame_id = policy_frame_;
  report(d, header, "pose", true, sent);
}

void FlightGovernor::governSpeedLimit(const geometry_msgs::msg::TwistStamped & msg)
{
  // Speed limits are magnitudes, so they are not transformed between frames.
  Command cmd;
  cmd.kind = Command::Kind::SPEED_LIMIT;
  cmd.velocity = Vec3{
    limitToCore(msg.twist.linear.x), limitToCore(msg.twist.linear.y),
    limitToCore(msg.twist.linear.z)};
  const Decision d = governor_->evaluate(cmd, nowSeconds());

  if (d.verdict == Verdict::ALLOW) {
    twist_pub_->publish(msg);
  } else {
    geometry_msgs::msg::TwistStamped out = msg;
    const Vec3 & v = d.allowed.velocity;
    const bool all_unlimited = std::isinf(v.x) && std::isinf(v.y) && std::isinf(v.z);
    auto toController = [all_unlimited](double value) {
        if (all_unlimited) {return 0.0;}
        if (std::isinf(value)) {return kUnlimitedSpeed;}
        return std::max(value, kMinSpeedLimit);
      };
    out.twist.linear.x = toController(v.x);
    out.twist.linear.y = toController(v.y);
    out.twist.linear.z = toController(v.z);
    if (d.verdict == Verdict::VETO) {
      out.twist.angular = geometry_msgs::msg::Vector3();
    }
    twist_pub_->publish(out);
  }
  report(d, msg.header, "speed_limit", true, true);
}

void FlightGovernor::governVelocity(const geometry_msgs::msg::TwistStamped & msg)
{
  geometry_msgs::msg::TwistStamped in = msg;
  const bool tf_ok = toPolicyFrame(in);

  Command cmd;
  cmd.kind = Command::Kind::TWIST;
  cmd.velocity = fromMsg(in.twist.linear);
  const Decision d = tf_ok ? governor_->evaluate(cmd, nowSeconds()) : governor_->veto(
    cmd, "cannot transform twist from '" + msg.header.frame_id + "' to '" + policy_frame_ + "'");

  if (d.verdict == Verdict::ALLOW) {
    twist_pub_->publish(msg);
  } else if (d.verdict == Verdict::CLAMP) {
    geometry_msgs::msg::TwistStamped out = in;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = policy_frame_;
    out.twist.linear = toMsg(d.allowed.velocity);
    twist_pub_->publish(out);
  } else {
    // A zero twist is zero in every frame, so the original header is kept.
    geometry_msgs::msg::TwistStamped out;
    out.header = msg.header;
    twist_pub_->publish(out);
  }
  std_msgs::msg::Header header = msg.header;
  header.frame_id = policy_frame_;
  report(d, header, "twist", true, true);
}

template<typename MsgT, typename PubT>
void FlightGovernor::passThrough(
  const MsgT & msg, const PubT & pub, const std::string & type, const std::string & why)
{
  pub->publish(msg);
  if (governor_->isLatched()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Veto is latched but %s references are passing through: %s", type.c_str(), why.c_str());
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "%s references pass through ungoverned: %s", type.c_str(), why.c_str());
  }
  Decision d;
  d.latched = governor_->isLatched();
  d.reason = why;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = policy_frame_;
  report(d, header, type, false, true);
}

// ---------------------------------------------------------------------------
// Reporting and helpers

void FlightGovernor::report(
  const Decision & decision, const std_msgs::msg::Header & header,
  const std::string & type, bool governed, bool sent)
{
  as2_msgs::msg::GovernorVerdict msg;
  msg.header = header;
  msg.verdict = static_cast<std::uint8_t>(decision.verdict);
  msg.latched = decision.latched;
  msg.governed = governed;
  msg.sent = sent;
  msg.command_type = type;
  msg.policies = decision.policies;
  msg.reason = decision.reason;
  msg.requested_position = toMsg(decision.requested.position);
  msg.requested_velocity = toMsg(decision.requested.velocity);
  msg.allowed_position = toMsg(decision.allowed.position);
  msg.allowed_velocity = toMsg(decision.allowed.velocity);
  verdict_pub_->publish(msg);

  if (!governed) {
    return;
  }
  if (decision.verdict == Verdict::VETO) {
    if (!was_vetoing_) {
      RCLCPP_ERROR(this->get_logger(), "VETO: %s", decision.reason.c_str());
      as2_msgs::msg::AlertEvent alert;
      alert.alert = static_cast<std::int8_t>(veto_alert_);
      alert.description = "flight_governor veto: " + decision.reason;
      alert_pub_->publish(alert);
    }
    if (!sent) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No safe %s hold exists yet (no vehicle position received): nothing sent",
        type.c_str());
    }
    was_vetoing_ = true;
  } else {
    was_vetoing_ = false;
    if (decision.verdict == Verdict::CLAMP) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "CLAMP: %s", decision.reason.c_str());
    }
  }
}

void FlightGovernor::checkWiring()
{
  // A governor nobody listens to governs nothing. Make that loud.
  const std::string out = kOutputPrefix;
  namespace mr = as2_names::topics::motion_reference;
  const std::size_t listeners =
    this->count_subscribers(out + mr::pose) + this->count_subscribers(out + mr::twist) +
    this->count_subscribers(out + mr::trajectory) + this->count_subscribers(out + mr::thrust);
  if (listeners == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Nothing subscribes to %smotion_reference/*: references are NOT being governed. "
      "Launch as2_motion_controller with use_flight_governor:=true.", kOutputPrefix);
  }
}

double FlightGovernor::nowSeconds() {return this->now().seconds();}

double FlightGovernor::stampSeconds(const builtin_interfaces::msg::Time & stamp) const
{
  return rclcpp::Time(stamp, this->get_clock()->get_clock_type()).seconds();
}

bool FlightGovernor::toPolicyFrame(geometry_msgs::msg::PoseStamped & msg)
{
  return msg.header.frame_id == policy_frame_ || tf_handler_->tryConvert(msg, policy_frame_);
}

bool FlightGovernor::toPolicyFrame(geometry_msgs::msg::TwistStamped & msg)
{
  return msg.header.frame_id == policy_frame_ || tf_handler_->tryConvert(msg, policy_frame_);
}

}  // namespace as2_flight_governor
