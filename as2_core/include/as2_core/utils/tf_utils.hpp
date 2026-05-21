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

/*!*******************************************************************************************
 *  \file       tf_utils.hpp
 *  \brief      Tranform utilities library header file.
 *  \authors    David Perez Saura
 *              Miguel Fernandez Cortizas
 *              Rafael Perez Segui
 ********************************************************************************/

#ifndef AS2_CORE__UTILS__TF_UTILS_HPP_
#define AS2_CORE__UTILS__TF_UTILS_HPP_


#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/create_timer_ros.h>

#include <string>
#include <utility>
#include <memory>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <as2_msgs/msg/trajectory_point.hpp>
#include <as2_msgs/msg/trajectory_setpoints.hpp>

#include "as2_core/custom/tf2_geometry_msgs.hpp"
#include "as2_core/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace as2
{
namespace tf
{

using namespace std::chrono_literals; // NOLINT

/**
 * @brief Prefix a TF frame name with a namespace, following aerostack2 conventions.
 *
 * Rules applied:
 *  - If `_frame_name` is empty, throws `std::runtime_error`.
 *  - If `_frame_name` starts with '/', it is treated as absolute and returned
 *    without the leading '/' (no namespace prefix is added).
 *  - If `_frame_name` already starts with `<_namespace>/`, it is returned as is.
 *  - If `_namespace` is empty, `_frame_name` is returned unchanged and a warning
 *    is logged (potential frame conflict).
 *  - Otherwise, returns "<namespace>/<frame_name>", stripping any leading '/'
 *    in `_namespace`.
 *
 * @param _namespace  Robot/node namespace. May be empty or start with '/'.
 * @param _frame_name Frame name, possibly absolute (leading '/') or already prefixed.
 * @return Fully-qualified TF frame id.
 * @throw std::runtime_error if `_frame_name` is empty.
 */
std::string generateTfName(const std::string & _namespace, const std::string & _frame_name);

/**
 * @brief Convenience overload that uses `node->get_namespace()` as namespace.
 *
 * @param node        ROS 2 node whose namespace will be used as prefix.
 * @param _frame_name Frame name (see `generateTfName(namespace, frame_name)` for rules).
 * @return Fully-qualified TF frame id.
 * @throw std::runtime_error if `_frame_name` is empty.
 */
std::string generateTfName(rclcpp::Node * node, std::string _frame_name);
/**
 * @brief Build a TransformStamped from a translation and Euler-angle rotation.
 *
 * The rotation is built with `tf2::Quaternion::setRPY(roll, pitch, yaw)`, i.e.
 * the standard ROS convention (intrinsic rotations applied in roll-pitch-yaw
 * order around the body axes).
 *
 * @note The returned message has its `header.stamp` left default-initialized;
 *       the caller is responsible for stamping it before broadcasting.
 *
 * @param _frame_id       Parent frame id (`header.frame_id`).
 * @param _child_frame_id Child frame id (`child_frame_id`).
 * @param _translation_x  Translation along X, in meters.
 * @param _translation_y  Translation along Y, in meters.
 * @param _translation_z  Translation along Z, in meters.
 * @param _roll           Roll angle, in radians.
 * @param _pitch          Pitch angle, in radians.
 * @param _yaw            Yaw angle, in radians.
 * @return `TransformStamped` describing parent → child.
 */
geometry_msgs::msg::TransformStamped getTransformation(
  const std::string & _frame_id, const std::string & _child_frame_id, double _translation_x,
  double _translation_y, double _translation_z, double _roll, double _pitch, double _yaw);

/**
 * @brief Helper that wraps `tf2_ros::Buffer` and `tf2_ros::TransformListener`
 *        for use inside an `as2::Node`.
 *
 * On construction, the handler declares (if not already declared) and reads the
 * ROS parameter `tf_timeout_threshold` (in seconds, default 0.05) on the owning
 * node. This timeout is used by all overloads that do not take an explicit
 * timeout argument.
 *
 * The convert/getter overloads that perform a `lookupTransform` use `"earth"`
 * as the fixed frame, which is the inertial root used in aerostack2.
 */
class TfHandler
{
private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  as2::Node * node_;
  std::chrono::nanoseconds tf_timeout_threshold_ = std::chrono::nanoseconds::zero();

public:
  /**
   * @brief Construct a new TfHandler bound to the given `as2::Node`.
   *
   * @param _node Owning node used for clock, parameters and logging.
   */
  explicit TfHandler(as2::Node * _node);

  /**
   * @brief Set the TF lookup timeout threshold.
   *
   * @param tf_timeout_threshold Timeout, in seconds.
   */
  void setTfTimeoutThreshold(double tf_timeout_threshold);

  /**
   * @brief Set the TF lookup timeout threshold.
   *
   * @param tf_timeout_threshold Timeout, as `std::chrono::nanoseconds`.
   */
  void setTfTimeoutThreshold(const std::chrono::nanoseconds & tf_timeout_threshold);

  /**
   * @brief Get the TF lookup timeout threshold.
   *
   * @return Timeout, in seconds.
   */
  double getTfTimeoutThreshold() const;

  /**
   * @brief Get the underlying TF buffer.
   *
   * @return Shared pointer to the `tf2_ros::Buffer` owned by this handler.
   */
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const;

  /**
   * @brief Convert a stamped message from one frame to another.
   *
   * Uses `lookupTransform(target, now, source, input.header.stamp, "earth", timeout)`
   * when `timeout > 0`, and the latest available transform (`tf2::TimePointZero`)
   * otherwise.
   *
   * @param input        Input stamped message; must expose `header.frame_id`
   *                     and `header.stamp`.
   * @param target_frame Target frame id.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Message expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  template<typename T>
  T convert(
    const T & input, const std::string & target_frame,
    const std::chrono::nanoseconds timeout)
  {
    T output;
    if (timeout != std::chrono::nanoseconds::zero()) {
      tf2::doTransform(
        input, output,
        tf_buffer_->lookupTransform(
          target_frame, node_->get_clock()->now(), input.header.frame_id, input.header.stamp,
          "earth", timeout));
    } else {
      tf2::doTransform(
        input, output,
        tf_buffer_->lookupTransform(
          target_frame, tf2::TimePointZero, input.header.frame_id, tf2::TimePointZero, "earth",
          timeout));
    }

    output.header.stamp = input.header.stamp;
    output.header.frame_id = target_frame;
    return output;
  }

  /**
   * @brief Convert a stamped message from one frame to another, using the
   *        configured `tf_timeout_threshold_`.
   *
   * Uses `lookupTransform(target, now, source, input.header.stamp, "earth", timeout)`
   * when `timeout > 0`, and the latest available transform (`tf2::TimePointZero`)
   * otherwise.
   *
   * @param input        Input stamped message; must expose `header.frame_id`
   *                     and `header.stamp`.
   * @param target_frame Target frame id.
   * @return Message expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  template<typename T>
  T convert(
    const T & input, const std::string & target_frame)
  {
    return convert<T>(input, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief Convert a `TwistStamped` to a target frame.
   *
   * Only the linear component is transformed (as a `Vector3Stamped`). The
   * angular component is copied through unchanged: this is correct when both
   * the source and target frames are inertial (or share the same orientation),
   * which is the typical aerostack2 case (e.g. `earth ↔ <ns>/odom`).
   *
   * @param _twist       Input twist with valid `header.frame_id` and `stamp`.
   * @param target_frame Target frame id.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Twist expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::TwistStamped convert(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & target_frame,
    const std::chrono::nanoseconds timeout);

  /**
   * @brief Convert a `TwistStamped` to a target frame, using the configured
   *        `tf_timeout_threshold_`.
   *
   * Only the linear component is transformed (as a `Vector3Stamped`). The
   * angular component is copied through unchanged: this is correct when both
   * the source and target frames are inertial (or share the same orientation),
   * which is the typical aerostack2 case (e.g. `earth ↔ <ns>/odom`).
   *
   * @param twist        Input twist with valid `header.frame_id` and `stamp`.
   * @param target_frame Target frame id.
   * @return Twist expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::TwistStamped convert(
    const geometry_msgs::msg::TwistStamped & twist, const std::string & target_frame)
  {
    return convert(twist, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief Convert a `nav_msgs::msg::Path` to a target frame.
   *
   * Each pose in `_path.poses` is transformed individually; the resulting path
   * has `header.frame_id = target_frame` and keeps the original `header.stamp`.
   *
   * @param _path        Input path. Each pose must carry a valid `frame_id`.
   * @param target_frame Target frame id.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Path with all poses expressed in `target_frame`.
   * @throw tf2::TransformException if any per-pose lookup fails.
   */
  nav_msgs::msg::Path convert(
    const nav_msgs::msg::Path & _path, const std::string & target_frame,
    const std::chrono::nanoseconds timeout);

  /**
   * @brief Convert a `nav_msgs::msg::Path` to a target frame, using the
   *        configured `tf_timeout_threshold_`.
   *
   * Each pose in `path.poses` is transformed individually; the resulting path
   * has `header.frame_id = target_frame` and keeps the original `header.stamp`.
   *
   * @param path         Input path. Each pose must carry a valid `frame_id`.
   * @param target_frame Target frame id.
   * @return Path with all poses expressed in `target_frame`.
   * @throw tf2::TransformException if any per-pose lookup fails.
   */
  nav_msgs::msg::Path convert(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame)
  {
    return convert(path, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief Convert an `as2_msgs::msg::TrajectorySetpoints` to a target frame.
   *
   * The trajectory `header.frame_id` is the frame in which both the position
   * and the linear quantities (twist, acceleration) of every `TrajectoryPoint`
   * are expressed. The transform is therefore applied as follows:
   *  - position: full transform (rotation + translation).
   *  - twist and acceleration: rotation only (they are vector quantities, not
   *    points, so the translation does not apply).
   *  - yaw_angle: the yaw component of the transform is added.
   *
   * The `yaw_angle` handling is geometrically meaningful only when the
   * transform between source and target frames does not introduce significant
   * roll or pitch — which holds for transforms between inertial frames such as
   * `earth ↔ <ns>/odom` in aerostack2 systems.
   *
   * @param traj         Input trajectory with valid `header.frame_id` and `stamp`.
   * @param target_frame Target frame id.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Trajectory expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  as2_msgs::msg::TrajectorySetpoints convert(
    const as2_msgs::msg::TrajectorySetpoints & traj, const std::string & target_frame,
    const std::chrono::nanoseconds timeout);

  /**
   * @brief Convert an `as2_msgs::msg::TrajectorySetpoints` to a target frame,
   *        using the configured `tf_timeout_threshold_`.
   *
   * The trajectory `header.frame_id` is the frame in which both the position
   * and the linear quantities (twist, acceleration) of every `TrajectoryPoint`
   * are expressed. The transform is therefore applied as follows:
   *  - position: full transform (rotation + translation).
   *  - twist and acceleration: rotation only (they are vector quantities, not
   *    points, so the translation does not apply).
   *  - yaw_angle: the yaw component of the transform is added.
   *
   * The `yaw_angle` handling is geometrically meaningful only when the
   * transform between source and target frames does not introduce significant
   * roll or pitch — which holds for transforms between inertial frames such as
   * `earth ↔ <ns>/odom` in aerostack2 systems.
   *
   * @param traj         Input trajectory with valid `header.frame_id` and `stamp`.
   * @param target_frame Target frame id.
   * @return Trajectory expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  as2_msgs::msg::TrajectorySetpoints convert(
    const as2_msgs::msg::TrajectorySetpoints & traj, const std::string & target_frame)
  {
    return convert(traj, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief Obtain a `PoseStamped` from the TF buffer.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Pose of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief Obtain a `PoseStamped` from the TF buffer.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `rclcpp::Time`.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Pose of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief Obtain a `PoseStamped` from the TF buffer using the configured
   *        `tf_timeout_threshold_`.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`. Defaults to
   *                     `tf2::TimePointZero` (latest).
   * @return Pose of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief Obtain a `PoseStamped` from the TF buffer using the configured
   *        `tf_timeout_threshold_`.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `rclcpp::Time`.
   * @return Pose of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time);

  /**
   * @brief Obtain a `QuaternionStamped` (orientation only) from the TF buffer.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Orientation of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief Obtain a `QuaternionStamped` (orientation only) from the TF buffer.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `rclcpp::Time`.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return Orientation of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief Obtain a `QuaternionStamped` from the TF buffer using the configured
   *        `tf_timeout_threshold_`.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`. Defaults to
   *                     `tf2::TimePointZero` (latest).
   * @return Orientation of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief Obtain a `QuaternionStamped` from the TF buffer using the configured
   *        `tf_timeout_threshold_`.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `rclcpp::Time`.
   * @return Orientation of `source_frame` expressed in `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  /**
   * @brief Obtain a `TransformStamped` from the TF buffer.
   *
   * @note Unlike the other getters in this class, this overload uses the simple
   *       two-frame `lookupTransform(target, source, time)` without a fixed
   *       frame and without an explicit timeout. For a timeout-bearing variant
   *       see the 4-argument overload below.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`. Defaults to
   *                     `tf2::TimePointZero` (latest).
   * @return Transform from `source_frame` to `target_frame`.
   * @throw tf2::TransformException if the lookup fails.
   */
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief Obtain a `TransformStamped` from the TF buffer, blocking up to
   *        `timeout` for the transform to become available.
   *
   * Uses the timeout-bearing two-frame
   * `lookupTransform(target, source, time, timeout)` overload of `tf2_ros::Buffer`,
   * without a fixed frame.
   *
   * @param target_frame Target frame id.
   * @param source_frame Source frame id.
   * @param time         Lookup time, as `tf2::TimePoint`.
   * @param timeout      Lookup timeout. Zero returns immediately if the
   *                     transform is not yet available.
   * @return Transform from `source_frame` to `target_frame`.
   * @throw tf2::TransformException if the lookup fails or the timeout expires.
   */
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief Try to convert `input` in place to a target frame.
   *
   * On failure, the `tf2::TransformException` is caught internally, a warning
   * is logged via the node logger, and `false` is returned. `input` is left
   * unmodified in that case.
   *
   * @param input        Variable to convert (modified in place on success).
   * @param target_frame Target frame id.
   * @param timeout      Lookup timeout. Zero uses `tf2::TimePointZero` (latest).
   * @return `true` if the conversion was successful, `false` otherwise.
   */
  template<typename T>
  bool tryConvert(
    T & input, const std::string & target_frame,
    const std::chrono::nanoseconds timeout)
  {
    try {
      input = convert(input, target_frame, timeout);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Could not get transform: %s", ex.what());
      return false;
    }
  }

  /**
   * @brief Try to convert `input` in place to a target frame, using the
   *        configured `tf_timeout_threshold_`.
   *
   * On failure, the `tf2::TransformException` is caught internally, a warning
   * is logged via the node logger, and `false` is returned. `input` is left
   * unmodified in that case.
   *
   * @param input        Variable to convert (modified in place on success).
   * @param target_frame Target frame id.
   * @return `true` if the conversion was successful, `false` otherwise.
   */
  template<typename T>
  bool tryConvert(T & input, const std::string & target_frame)
  {
    return tryConvert(input, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief Build a (pose, twist) pair expressed in the requested frames.
   *
   * Steps:
   *  1. Transform the input twist to `_twist_target_frame` (linear part is
   *     rotated; angular part is kept — see `convert(TwistStamped, ...)`).
   *  2. Look up the pose of `_pose_source_frame` in `_pose_target_frame`,
   *     stamped at the converted twist's `header.stamp`.
   *
   * @param _twist               Input twist with valid `header.frame_id` and `stamp`.
   * @param _twist_target_frame  Frame in which the output twist is expressed.
   * @param _pose_target_frame   Target frame for the pose lookup.
   * @param _pose_source_frame   Source frame for the pose lookup.
   * @param timeout              Timeout used for both TF lookups.
   * @return `std::pair{ pose, twist }` with the pose in `_pose_target_frame`
   *         and the twist in `_twist_target_frame`.
   * @throw tf2::TransformException if any of the two lookups fails.
   */
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> getState(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & _twist_target_frame,
    const std::string & _pose_target_frame, const std::string & _pose_source_frame,
    const std::chrono::nanoseconds timeout);

  /**
   * @brief Build a (pose, twist) pair expressed in the requested frames, using
   *        the configured `tf_timeout_threshold_`.
   *
   * Steps:
   *  1. Transform the input twist to `_twist_target_frame` (linear part is
   *     rotated; angular part is kept — see `convert(TwistStamped, ...)`).
   *  2. Look up the pose of `_pose_source_frame` in `_pose_target_frame`,
   *     stamped at the converted twist's `header.stamp`.
   *
   * @param _twist               Input twist with valid `header.frame_id` and `stamp`.
   * @param _twist_target_frame  Frame in which the output twist is expressed.
   * @param _pose_target_frame   Target frame for the pose lookup.
   * @param _pose_source_frame   Source frame for the pose lookup.
   * @return `std::pair{ pose, twist }` with the pose in `_pose_target_frame`
   *         and the twist in `_twist_target_frame`.
   * @throw tf2::TransformException if any of the two lookups fails.
   */
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> getState(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & _twist_target_frame,
    const std::string & _pose_target_frame, const std::string & _pose_source_frame);
};  // class TfHandler

}  // namespace tf
}  // namespace as2
#endif  // AS2_CORE__UTILS__TF_UTILS_HPP_
