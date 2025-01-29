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
 * @brief Add namespace to the name of the Transform frame id
 *
 * @param _namespace
 * @param _frame_name
 * @return std::string namespace/frame_id
 */
std::string generateTfName(const std::string & _namespace, const std::string & _frame_name);
std::string generateTfName(rclcpp::Node * node, std::string _frame_name);
/**
 * @brief Generate a Transform message from translation and rotation in Euler angles
 *
 * @param _frame_id
 * @param _child_frame_id
 * @param _translation_x
 * @param _translation_y
 * @param _translation_z
 * @param _roll
 * @param _pitch
 * @param _yaw
 * @return geometry_msgs::msg::TransformStamped
 */
geometry_msgs::msg::TransformStamped getTransformation(
  const std::string & _frame_id, const std::string & _child_frame_id, double _translation_x,
  double _translation_y, double _translation_z, double _roll, double _pitch, double _yaw);

/**
 * @brief  TfHandler class to handle the tf2_ros::Buffer and tf2_ros::TransformListener with ease
 * inside a as2::Node class
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
   * @brief Construct a new Tf Handler object
   * @param _node an as2::Node object
   */
  explicit TfHandler(as2::Node * _node);

  /**
   * @brief Set the tf timeout threshold
   * @param tf_timeout_threshold double in seconds
   */
  void setTfTimeoutThreshold(double tf_timeout_threshold);

  /**
   * @brief Set the tf timeout threshold
   * @param tf_timeout_threshold std::chrono::nanoseconds
   */
  void setTfTimeoutThreshold(const std::chrono::nanoseconds & tf_timeout_threshold);

  /**
   * @brief Get the tf timeout threshold
   * @return double
   */
  double getTfTimeoutThreshold() const;

  /**
   * @brief Get the tf buffer object
   * @return std::shared_ptr<tf2_ros::Buffer>
   */
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const;

  /**
   * @brief convert a from one frame to another
   * @param input variable to convert
   * @param target_frame the target frame
   * @param timeout the timeout for the transform
   * @return variable in the target frame
   * @throw tf2::TransformException if the transform is not available
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
   * @brief convert a from one frame to another
   * @param input variable to convert
   * @param target_frame the target frame
   * @return variable in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  template<typename T>
  T convert(
    const T & input, const std::string & target_frame)
  {
    return convert<T>(input, target_frame, tf_timeout_threshold_);
  }

  geometry_msgs::msg::TwistStamped convert(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & target_frame,
    const std::chrono::nanoseconds timeout)
  {
    geometry_msgs::msg::TwistStamped twist_out;
    geometry_msgs::msg::Vector3Stamped vector_out;

    vector_out.header = _twist.header;
    vector_out.vector = _twist.twist.linear;
    // transform linear speed
    vector_out = convert(vector_out, target_frame, timeout);
    twist_out.header = vector_out.header;
    twist_out.twist.linear = vector_out.vector;
    twist_out.twist.angular = _twist.twist.angular;
    return twist_out;
  }

  geometry_msgs::msg::TwistStamped convert(
    const geometry_msgs::msg::TwistStamped & twist, const std::string & target_frame)
  {
    return convert(twist, target_frame, tf_timeout_threshold_);
  }

  nav_msgs::msg::Path convert(
    const nav_msgs::msg::Path & _path, const std::string & target_frame,
    const std::chrono::nanoseconds timeout)
  {
    nav_msgs::msg::Path path_out;

    for (auto & pose : _path.poses) {
      geometry_msgs::msg::PoseStamped pose_out;
      if (timeout != std::chrono::nanoseconds::zero()) {
        tf2::doTransform(
          pose, pose_out,
          tf_buffer_->lookupTransform(
            target_frame, node_->get_clock()->now(), pose.header.frame_id, pose.header.stamp,
            "earth",
            timeout));
      } else {
        tf2::doTransform(
          pose, pose_out,
          tf_buffer_->lookupTransform(
            target_frame, tf2::TimePointZero, pose.header.frame_id, tf2::TimePointZero, "earth",
            timeout));
      }

      path_out.poses.push_back(pose_out);
    }
    path_out.header.frame_id = target_frame;
    path_out.header.stamp = _path.header.stamp;
    return path_out;
  }

  nav_msgs::msg::Path convert(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame)
  {
    return convert(path, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief obtain a PoseStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in TimePoint
   * @param timeout the timeout for the transform
   * @return geometry_msgs::msg::PoseStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief obtain a PoseStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in Ros Time
   * @param timeout the timeout for the transform
   * @return geometry_msgs::msg::PoseStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in TimePoint
   * @return geometry_msgs::msg::TransformStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in Ros Time
   * @return geometry_msgs::msg::TransformStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in TimePoint
   * @param timeout the timeout for the transform
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in Ros Time
   * @param timeout the timeout for the transform
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time, const std::chrono::nanoseconds timeout);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in TimePoint
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in Ros Time
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero);

  /**
   * @brief convert input to desired frame, checking if frames are
   * valid
   * @param input a variable to get converted
   * @param target_frame the target frame
   * @param timeout the timeout for the transform
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
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
   * @brief convert to desired frame, checking if frames are
   * valid
   * @param input a variable to get converted
   * @param target_frame the target frame
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  template<typename T>
  bool tryConvert(T & input, const std::string & target_frame)
  {
    return tryConvert(input, target_frame, tf_timeout_threshold_);
  }

  /**
   * @brief convert a geometry_msgs::msg::QuaternionStamped to desired frame, checking if frames are
   * valid
   * @param _quaternion a geometry_msgs::msg::QuaternionStamped to get converted
   * @param _target_frame the target frame
   * @param _timeout the timeout for the transform
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> getState(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & _twist_target_frame,
    const std::string & _pose_target_frame, const std::string & _pose_source_frame,
    const std::chrono::nanoseconds timeout);

  /**
   * @brief convert a geometry_msgs::msg::QuaternionStamped to desired frame, checking if frames are
   * valid
   * @param _quaternion a geometry_msgs::msg::QuaternionStamped to get converted
   * @param _target_frame the target frame
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> getState(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & _twist_target_frame,
    const std::string & _pose_target_frame, const std::string & _pose_source_frame);
};  // namespace tf

}  // namespace tf
}  // namespace as2
#endif  // AS2_CORE__UTILS__TF_UTILS_HPP_
