// Copyright 2026 Universidad Politécnica de Madrid
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
//    of its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

/*!*******************************************************************************************
 *  @file       geometric_controller.cpp
 *  @brief      Geometric controller plugin for the Aerostack framework.
 *  @authors    Rafael Pérez Seguí
 ********************************************************************************************/

#include "geometric_controller.hpp"

#include <algorithm>

#include "pid_controller/pid.hpp"
#include "geometric_controllers/attitude_geometric_controller.hpp"
#include "geometric_controllers/rates_geometric_controller.hpp"

namespace geometric_controller
{

void Plugin::ownInitialize()
{
  position_controller_.set_proportional_saturation_flag(true);
  velocity_controller_.set_proportional_saturation_flag(true);
  trajectory_controller_.set_proportional_saturation_flag(true);

  // TrajectorySetpoints encodes pose and twist in the same frame; align both
  // input frames here. Output stays in base_link.
  setDesiredTwistFrameId(getDesiredPoseFrameId());

  // Plugin-specific debug publishers, namespaced under <plugin>.debug.*.
  // The standard state/reference debug topics are owned by ControllerHandler.
  debug_desired_velocity_pub_ =
    createDebugPublisher<geometry_msgs::msg::TwistStamped>("debug.desired_velocity_topic");
  debug_acceleration_pub_ =
    createDebugPublisher<geometry_msgs::msg::AccelStamped>("debug.acceleration_topic");
  debug_velocity_norm_pub_ =
    createDebugPublisher<geometry_msgs::msg::Vector3Stamped>("debug.velocity_norm_topic");
}

std::vector<std::string> Plugin::requiredParameters() const
{
  std::vector<std::string> required;
  for (const std::string & block : {"position", "velocity", "trajectory"}) {
    for (const std::string & gain : {"kp", "ki", "kd", "antiwindup_cte"}) {
      required.push_back(block + "." + gain);
    }
  }
  // The trajectory loop closes the velocity error directly, so it has no
  // derivative filter, and only the velocity loop saturates its output.
  required.push_back("position.alpha");
  required.push_back("velocity.alpha");
  required.push_back("velocity.saturation_upper");
  required.push_back("velocity.saturation_lower");
  for (const std::string & gain : {"kp", "ki", "kd", "antiwindup_cte", "alpha"}) {
    required.push_back("yaw." + gain);
  }
  required.push_back("geometric.mass");
  required.push_back("geometric.rotation_kp");
  required.push_back("geometric.yaw_speed_limit");
  return required;
}

void Plugin::updateParameter(const std::string & name, const rclcpp::Parameter & param)
{
  using as2_motion_controller_param_utils::readVector3;

  const auto dot = name.find('.');
  const std::string block = name.substr(0, dot);
  const std::string subname = dot == std::string::npos ? std::string() : name.substr(dot + 1);

  bool known = false;
  if (block == "position") {
    known = updatePidParameter(position_controller_, subname, param, true, false);
  } else if (block == "velocity") {
    known = updatePidParameter(velocity_controller_, subname, param, true, true);
  } else if (block == "trajectory") {
    known = updatePidParameter(trajectory_controller_, subname, param, false, false);
  } else if (block == "yaw") {
    known = true;
    if (subname == "kp") {
      yaw_controller_.set_kp(param.as_double());
    } else if (subname == "ki") {
      yaw_controller_.set_ki(param.as_double());
    } else if (subname == "kd") {
      yaw_controller_.set_kd(param.as_double());
    } else if (subname == "antiwindup_cte") {
      yaw_controller_.set_anti_windup(param.as_double());
    } else if (subname == "alpha") {
      yaw_controller_.set_alpha(param.as_double());
    } else {
      known = false;
    }
  } else if (block == "geometric") {
    known = true;
    if (subname == "mass") {
      geometric_controller_.update_vehicle_mass(param.as_double());
    } else if (subname == "rotation_kp") {
      geometric_controller_.update_kp_rotation(readVector3(param));
    } else if (subname == "yaw_speed_limit") {
      yaw_speed_limit_ = param.as_double();
    } else {
      known = false;
    }
  }

  if (!known) {
    RCLCPP_ERROR(getNodePtr()->get_logger(), "Unknown parameter '%s'", name.c_str());
  }
}

void Plugin::reset()
{
  ControllerBase::reset();
  position_controller_.reset_controller();
  yaw_controller_.reset_controller();
  velocity_controller_.reset_controller();
  trajectory_controller_.reset_controller();
}

as2_msgs::msg::ControlMode Plugin::hoverMode() const
{
  as2_msgs::msg::ControlMode mode;
  mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
  mode.control_mode = as2_msgs::msg::ControlMode::UNSET;

  if (!position_controller_.get_gains_kp().isZero()) {
    mode.control_mode = as2_msgs::msg::ControlMode::POSITION;
  } else if (!velocity_controller_.get_gains_kp().isZero()) {
    // Without a position loop the hold only drives the speed to zero, so the
    // drift is not corrected.
    mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  }
  return mode;
}

bool Plugin::onSetMode(
  const as2_msgs::msg::ControlMode & mode_in,
  const as2_msgs::msg::ControlMode & mode_out)
{
  refreshAvailability();

  if (!isControlModeAvailable(mode_in, mode_out)) {
    RCLCPP_WARN(
      getNodePtr()->get_logger(),
      "Control mode [%s] rejected: its control loop is disabled by zero gains",
      as2::control_mode::controlModeToString(mode_in).c_str());
    return false;
  }

  ref_velocity_.setZero();
  ref_acceleration_.setZero();
  return true;
}

void Plugin::onUpdateState(
  const geometry_msgs::msg::PoseStamped & pose_msg,
  const geometry_msgs::msg::TwistStamped & twist_msg)
{
  current_position_ = Eigen::Vector3d(
    pose_msg.pose.position.x,
    pose_msg.pose.position.y,
    pose_msg.pose.position.z);
  current_velocity_ = Eigen::Vector3d(
    twist_msg.twist.linear.x,
    twist_msg.twist.linear.y,
    twist_msg.twist.linear.z);
  current_orientation_ = Eigen::Quaterniond(
    pose_msg.pose.orientation.w,
    pose_msg.pose.orientation.x,
    pose_msg.pose.orientation.y,
    pose_msg.pose.orientation.z);
  current_orientation_.normalize();
}

void Plugin::onUpdateReference(const geometry_msgs::msg::PoseStamped & ref)
{
  if (getControlModeIn().control_mode != as2_msgs::msg::ControlMode::POSITION &&
    getControlModeIn().control_mode != as2_msgs::msg::ControlMode::SPEED)
  {
    return;
  }

  ref_position_ = Eigen::Vector3d(
    ref.pose.position.x,
    ref.pose.position.y,
    ref.pose.position.z);
  ref_yaw_ = as2::frame::getYawFromQuaternion(ref.pose.orientation);
}

void Plugin::onUpdateReference(const geometry_msgs::msg::TwistStamped & ref)
{
  if (getControlModeIn().control_mode != as2_msgs::msg::ControlMode::POSITION &&
    getControlModeIn().control_mode != as2_msgs::msg::ControlMode::SPEED)
  {
    return;
  }

  ref_velocity_ = Eigen::Vector3d(
    ref.twist.linear.x,
    ref.twist.linear.y,
    ref.twist.linear.z);

  if (getControlModeIn().control_mode == as2_msgs::msg::ControlMode::SPEED) {
    desired_velocity_ = ref_velocity_;
  }

  if (getControlModeIn().yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    ref_yaw_speed_ = ref.twist.angular.z;
  }
}

void Plugin::onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & ref)
{
  if (getControlModeIn().control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }
  if (ref.setpoints.empty()) {
    return;
  }

  const auto & sp = ref.setpoints[0];
  ref_position_ = Eigen::Vector3d(sp.position.x, sp.position.y, sp.position.z);
  ref_velocity_ = Eigen::Vector3d(sp.twist.x, sp.twist.y, sp.twist.z);
  ref_acceleration_ = Eigen::Vector3d(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
  ref_yaw_ = sp.yaw_angle;
}

bool Plugin::computeOutput(
  double dt,
  geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::TwistStamped & twist,
  as2_msgs::msg::Thrust & thrust)
{
  (void)pose;  // This plugin does not write the pose output.

  const bool speed_exit = getControlModeOut().control_mode == as2_msgs::msg::ControlMode::SPEED;

  if (getControlModeIn().control_mode == as2_msgs::msg::ControlMode::TRAJECTORY && speed_exit) {
    // The SPEED output cannot express an acceleration, so the trajectory is
    // tracked by the position loop with the reference velocity fed forward.
    desired_velocity_ = position_controller_.position_to_linear_velocity(
      current_position_, ref_position_, dt) + ref_velocity_;
  } else if (getControlModeIn().control_mode == as2_msgs::msg::ControlMode::TRAJECTORY) {
    // Trajectory tracking closes position and velocity errors in a single loop,
    // with the reference acceleration as feed-forward term.
    desired_velocity_ = ref_velocity_;
    desired_acceleration_ = trajectory_controller_.trajectory_to_linear_acceleration(
      current_position_, current_velocity_, ref_position_, ref_velocity_, ref_acceleration_, dt);
  } else {
    // 1. Position -> linear velocity
    if (getControlModeIn().control_mode == as2_msgs::msg::ControlMode::SPEED) {
      desired_velocity_ = ref_velocity_;
    } else {
      desired_velocity_ = position_controller_.position_to_linear_velocity(
        current_position_, ref_position_, dt);

      // Saturate by norm using the (overloaded) reference x-component as max
      // velocity, preserving the legacy contract of the position+velocity API.
      if (ref_velocity_.norm() > 0.001) {
        const double max_velocity = ref_velocity_.x();
        if (desired_velocity_.norm() > max_velocity) {
          desired_velocity_ = desired_velocity_.normalized() * max_velocity;
        }
      }
    }

    // 2. Linear velocity -> linear acceleration
    if (!speed_exit) {
      desired_acceleration_ = velocity_controller_.linear_velocity_to_linear_acceleration(
        current_velocity_, desired_velocity_, dt);
    }
  }

  const auto stamp = getNodePtr()->now();

  if (speed_exit) {
    twist.header.stamp = stamp;
    twist.header.frame_id = getDesiredPoseFrameId();
    twist.twist.linear.x = desired_velocity_.x();
    twist.twist.linear.y = desired_velocity_.y();
    twist.twist.linear.z = desired_velocity_.z();
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = getYawSpeedCommand(dt);

    publishPluginDebug(stamp);
    return true;
  }

  // 3. Yaw reference
  double yaw_reference = ref_yaw_;
  if (getControlModeIn().yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    yaw_reference = yawSpeedToYawAngle(
      getLimitedYawSpeed(), current_orientation_, geometric_controller_.get_kp_rotation().z());
  }

  // 4. Acceleration + yaw -> thrust + body rates
  auto [desired_thrust, desired_rates] = geometric_controller_.acceleration_to_rates(
    desired_acceleration_, yaw_reference, current_orientation_);

  thrust.header.stamp = stamp;
  thrust.header.frame_id = getNodePtr()->getBaseFrameId();
  thrust.thrust = desired_thrust;

  twist.header.stamp = stamp;
  twist.header.frame_id = getNodePtr()->getBaseFrameId();
  twist.twist.linear.x = desired_velocity_.x();
  twist.twist.linear.y = desired_velocity_.y();
  twist.twist.linear.z = desired_velocity_.z();
  twist.twist.angular.x = desired_rates.x();
  twist.twist.angular.y = desired_rates.y();
  twist.twist.angular.z = desired_rates.z();

  publishPluginDebug(stamp);
  return true;
}

// ===== Internal helpers =====================================================

bool Plugin::updatePidParameter(
  pid_controller::PID<double, 3> & pid_handler,
  const std::string & name,
  const rclcpp::Parameter & param,
  const bool with_alpha,
  const bool with_saturation)
{
  using as2_motion_controller_param_utils::readVector3;

  if (name == "kp") {
    pid_handler.set_gains_kp(readVector3(param));
  } else if (name == "ki") {
    pid_handler.set_gains_ki(readVector3(param));
  } else if (name == "kd") {
    pid_handler.set_gains_kd(readVector3(param));
  } else if (name == "antiwindup_cte") {
    pid_handler.set_anti_windup(Eigen::Vector3d::Constant(param.as_double()));
  } else if (with_alpha && name == "alpha") {
    pid_handler.set_alpha(Eigen::Vector3d::Constant(param.as_double()));
  } else if (with_saturation && (name == "saturation_upper" || name == "saturation_lower")) {
    // The two limits are set together, so the one that did not change is
    // taken from the loop itself.
    const auto current = pid_handler.get_params();
    const Eigen::Vector3d value = readVector3(param);
    const bool upper_changed = name == "saturation_upper";
    pid_handler.set_output_saturation(
      upper_changed ? value : current.upper_output_saturation,
      upper_changed ? current.lower_output_saturation : value, true);
  } else {
    return false;
  }
  return true;
}

void Plugin::setLoopEnabled(bool & _flag, const bool _enabled, const char * _reason)
{
  _flag = _enabled;
  if (!_flag) {
    RCLCPP_WARN(getNodePtr()->get_logger(), "%s", _reason);
  }
}

void Plugin::refreshAvailability()
{
  // A loop with no proportional action produces no actuation at all.
  setLoopEnabled(
    position_enabled_, !position_controller_.get_gains_kp().isZero(),
    "Position loop disabled: zero proportional gains");
  setLoopEnabled(
    velocity_enabled_, !velocity_controller_.get_gains_kp().isZero(),
    "Velocity loop disabled: zero proportional gains");
  setLoopEnabled(
    trajectory_enabled_, !trajectory_controller_.get_gains_kp().isZero(),
    "Trajectory loop disabled: zero proportional gains");
  setLoopEnabled(
    yaw_enabled_, yaw_controller_.get_kp() > 0.0,
    "Yaw loop disabled: zero proportional gain");
  setLoopEnabled(
    geometric_enabled_,
    geometric_controller_.get_vehicle_mass() > 0.0 &&
    (geometric_controller_.get_kp_rotation().array() > 0.0).all(),
    "Geometric loop disabled: non-positive vehicle mass or rotation gains");
}

double Plugin::getLimitedYawSpeed() const
{
  if (yaw_speed_limit_ <= 0.0) {
    return ref_yaw_speed_;
  }
  return std::clamp(ref_yaw_speed_, -yaw_speed_limit_, yaw_speed_limit_);
}

double Plugin::getYawSpeedCommand(const double dt)
{
  if (getControlModeIn().yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    return getLimitedYawSpeed();
  }

  const double yaw_error = as2::frame::angleMinError(
    ref_yaw_, as2::frame::getYawFromQuaternion(current_orientation_));
  return yaw_controller_.compute_control(dt, yaw_error);
}

double Plugin::yawSpeedToYawAngle(
  const double yaw_speed,
  const Eigen::Quaterniond & current_orientation,
  const double kp_yaw)
{
  const double current_yaw = as2::frame::getYawFromQuaternion(current_orientation);
  if (kp_yaw <= 0.0) {
    return current_yaw;
  }
  return current_yaw + yaw_speed / kp_yaw;
}

bool Plugin::isControlModeAvailable(
  const as2_msgs::msg::ControlMode & mode_in,
  const as2_msgs::msg::ControlMode & mode_out) const
{
  if (mode_in.control_mode == as2_msgs::msg::ControlMode::UNSET) {
    // Bypass: the plugin does not control the vehicle.
    return true;
  }

  const bool speed_exit = mode_out.control_mode == as2_msgs::msg::ControlMode::SPEED;
  if (speed_exit) {
    // The SPEED output carries the yaw as a rate and stops before the attitude
    // stage, so it needs the yaw loop instead of the geometric one.
    if (!yaw_enabled_) {
      return false;
    }
  } else if (!geometric_enabled_ || !velocity_enabled_) {
    return false;
  }

  switch (mode_in.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION:
      return position_enabled_;
    case as2_msgs::msg::ControlMode::SPEED:
      return true;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      return speed_exit ? position_enabled_ : trajectory_enabled_;
    default:
      return false;
  }
}

void Plugin::publishPluginDebug(const rclcpp::Time & stamp)
{
  if (debug_desired_velocity_pub_) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = getDesiredPoseFrameId();
    msg.twist.linear.x = desired_velocity_.x();
    msg.twist.linear.y = desired_velocity_.y();
    msg.twist.linear.z = desired_velocity_.z();
    debug_desired_velocity_pub_->publish(msg);
  }
  if (debug_acceleration_pub_) {
    geometry_msgs::msg::AccelStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = getDesiredPoseFrameId();
    msg.accel.linear.x = desired_acceleration_.x();
    msg.accel.linear.y = desired_acceleration_.y();
    msg.accel.linear.z = desired_acceleration_.z();
    debug_acceleration_pub_->publish(msg);
  }
  if (debug_velocity_norm_pub_) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = getDesiredPoseFrameId();
    msg.vector.x = ref_velocity_.x();          // Configured speed limit.
    msg.vector.y = current_velocity_.norm();   // Actual speed.
    msg.vector.z = desired_velocity_.norm();   // Commanded speed.
    debug_velocity_norm_pub_->publish(msg);
  }
}

}  // namespace geometric_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  geometric_controller::Plugin,
  as2_motion_controller_plugin_base::ControllerBase)
