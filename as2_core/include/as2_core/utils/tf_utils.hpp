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

#define TF_TIMEOUT 50ms

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

public:
  /**
   * @brief Construct a new Tf Handler object
   * @param _node an as2::Node object
   */
  explicit TfHandler(as2::Node * _node)
  : node_(_node)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /**
   * @brief Get the tf buffer object
   * @return std::shared_ptr<tf2_ros::Buffer>
   */
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const {return tf_buffer_;}

  /**
   * @brief convert a geometry_msgs::msg::PointStamped from one frame to another
   * @param _point a geometry_msgs::msg::PointStamped
   * @param _target_frame the target frame
   * @return geometry_msgs::msg::PointStamped in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PointStamped convert(
    const geometry_msgs::msg::PointStamped & _point, const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::PoseStamped from one frame to another
   * @param _pose a geometry_msgs::msg::PoseStamped
   * @param _target_frame the target frame
   * @return geometry_msgs::msg::PoseStamped in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped convert(
    const geometry_msgs::msg::PoseStamped & _pose, const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::TwistStamped from one frame to another (only the linear
   * part will be converted, the angular part will be maintained)
   * @param _twist a geometry_msgs::msg::TwistStamped
   * @param _target_frame the target frame
   * @return geometry_msgs::msg::TwistStamped in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::TwistStamped convert(
    const geometry_msgs::msg::TwistStamped & _twist, const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::Vector3Stamped from one frame to another
   * @param _vector a geometry_msgs::msg::Vector3Stamped
   * @param _target_frame the target frame
   * @return geometry_msgs::msg::Vector3Stamped in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::Vector3Stamped convert(
    const geometry_msgs::msg::Vector3Stamped & _vector, const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a nav_msgs::msg::Path from one frame to another
   * @param _path a nav_msgs::msg::Path
   * @param _target_frame the target frame
   * @return nav_msgs::msg::Path in the target frame
   * @throw tf2::TransformException if the transform is not available
   */
  nav_msgs::msg::Path convert(
    const nav_msgs::msg::Path & _path, const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::QuaternionStamped from one frame to another
   * @param _quaternion a geometry_msgs::msg::QuaternionStamped
   * @param _target_frame the target frame
   * @return geometry_msgs::msg::QuaternionStamped in the target frame
   * @throw tf2::TransformException if the transform is not available
  */
  geometry_msgs::msg::QuaternionStamped convert(
    const geometry_msgs::msg::QuaternionStamped & _quaternion,
    const std::string & target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief obtain a PoseStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform in Ros Time
   * @return geometry_msgs::msg::PoseStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::PoseStamped getPoseStamped(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

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
    const tf2::TimePoint & time = tf2::TimePointZero,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time = tf2::TimePointZero,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief obtain a TransformStamped from the TF_buffer
   * @param target_frame the target frame
   * @param source_frame the source frame
   * @param time the time of the transform
   * @return geometry_msgs::msg::QuaternionStamped
   * @throw tf2::TransformException if the transform is not available
   */
  geometry_msgs::msg::QuaternionStamped getQuaternionStamped(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

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
   * @brief convert a geometry_msgs::msg::PointStamped to desired frame, checking if frames are
   * valid
   * @param point a geometry_msgs::msg::PointStamped to get converted
   * @param _target_frame the target frame
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  bool tryConvert(
    geometry_msgs::msg::PointStamped & _point, const std::string & _target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::PoseStamped to desired frame, checking if frames are
   * valid
   * @param point a geometry_msgs::msg::PoseStamped to get converted
   * @param _target_frame the target frame
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  bool tryConvert(
    geometry_msgs::msg::PoseStamped & _pose, const std::string & _target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief convert a geometry_msgs::msg::TwistStamped to desired frame, checking if frames are
   * valid
   * @param point a geometry_msgs::msg::TwistStamped to get converted
   * @param _target_frame the target frame
   * @return bool true if the conversion was successful
   * @throw tf2::TransformException if the transform is not available
   */
  bool tryConvert(
    geometry_msgs::msg::TwistStamped & _twist, const std::string & _target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

  /**
   * @brief Get the pose and twist of the UAV at the given twist timestamp, in the given frames
   *
   * @param _twist
   * @param _twist_target_frame
   * @param _pose_target_frame
   * @param _pose_source_frame
   * @param _timeout
   * @return std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped>
   */
  bool tryConvert(
    geometry_msgs::msg::QuaternionStamped & _quaternion, const std::string & _target_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);

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
    const std::string & _pose_target_frame, const std::string & _pose_source_frame,
    const std::chrono::nanoseconds timeout = TF_TIMEOUT);
};  // namespace tf

}  // namespace tf
}  // namespace as2
#endif  // AS2_CORE__UTILS__TF_UTILS_HPP_
