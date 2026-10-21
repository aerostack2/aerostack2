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
* @file ros_conversions.hpp
*
* Messages into the filter's own types and back, and frame ids into the frames it knows
*
* @authors Rodrigo Da Silva Gómez
*/

#ifndef SIMPLE_EKF__ROS_CONVERSIONS_HPP_
#define SIMPLE_EKF__ROS_CONVERSIONS_HPP_

#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <simple_ekf_core/types.hpp>

namespace simple_ekf
{

/**
 * @brief A message stamp in nanoseconds, built from its two integer fields so that no
 *        precision is lost on the way.
 */
inline simple_ekf_core::Nanoseconds toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<simple_ekf_core::Nanoseconds>(stamp.sec) * 1000000000LL +
         static_cast<simple_ekf_core::Nanoseconds>(stamp.nanosec);
}

/**
 * @brief A frame id without the leading slash some publishers still put on it.
 */
inline std::string bareFrame(const std::string & frame_id)
{
  return (!frame_id.empty() && frame_id[0] == '/') ? frame_id.substr(1) : frame_id;
}

/**
 * @brief The state estimator's own frame ids, as its node names them: "earth", "drone0/map",
 *        "drone0/odom", "drone0/base_link".
 */
struct FrameIds
{
  std::string earth;
  std::string map;
  std::string odom;
  std::string base;
};

/**
 * @brief Which of the filter's frames a frame id names, by exact name.
 *
 * @return Nothing if the id is none of the state estimator's own frame ids.
 */
inline std::optional<simple_ekf_core::SourceFrame> matchFrameId(
  const std::string & frame_id, const FrameIds & frames)
{
  const std::string frame = bareFrame(frame_id);
  if (frame == frames.earth) {
    return simple_ekf_core::SourceFrame::EARTH;
  }
  if (frame == frames.map) {
    return simple_ekf_core::SourceFrame::MAP;
  }
  if (frame == frames.odom) {
    return simple_ekf_core::SourceFrame::ODOM;
  }
  if (frame == frames.base) {
    return simple_ekf_core::SourceFrame::BASE;
  }
  return std::nullopt;
}

/**
 * @brief The state estimator's frame id for one of the filter's frames.
 */
inline const std::string & frameIdOf(simple_ekf_core::SourceFrame frame, const FrameIds & frames)
{
  switch (frame) {
    case simple_ekf_core::SourceFrame::EARTH:
      return frames.earth;
    case simple_ekf_core::SourceFrame::ODOM:
      return frames.odom;
    case simple_ekf_core::SourceFrame::BASE:
      return frames.base;
    default:
      return frames.map;
  }
}

/**
 * @brief A guess of which of the filter's frames a pose's frame id names, from the words in it.
 *
 * Only for an id that is none of the state estimator's own (see @ref matchFrameId), as every
 * id was resolved before: by substring, in this order, and an id with none of the words is
 * taken as the map frame.
 */
inline simple_ekf_core::SourceFrame guessPoseSourceFrame(const std::string & frame_id)
{
  const std::string frame = bareFrame(frame_id);
  if (frame.find("earth") != std::string::npos) {
    return simple_ekf_core::SourceFrame::EARTH;
  }
  if (frame.find("map") != std::string::npos) {
    return simple_ekf_core::SourceFrame::MAP;
  }
  if (frame.find("odom") != std::string::npos) {
    return simple_ekf_core::SourceFrame::ODOM;
  }
  if (frame.find("base") != std::string::npos) {
    return simple_ekf_core::SourceFrame::BASE;
  }
  return simple_ekf_core::SourceFrame::MAP;
}

/**
 * @brief A guess of which of the filter's frames a twist's frame id names, from the words in it.
 *
 * As @ref guessPoseSourceFrame, except that an id with none of the words is the vehicle's,
 * where a twist normally is.
 */
inline simple_ekf_core::SourceFrame guessTwistSourceFrame(const std::string & frame_id)
{
  const std::string frame = bareFrame(frame_id);
  if (frame.find("earth") != std::string::npos) {
    return simple_ekf_core::SourceFrame::EARTH;
  }
  if (frame.find("odom") != std::string::npos) {
    return simple_ekf_core::SourceFrame::ODOM;
  }
  if (frame.find("map") != std::string::npos) {
    return simple_ekf_core::SourceFrame::MAP;
  }
  return simple_ekf_core::SourceFrame::BASE;
}

inline simple_ekf_core::Vector3 pointToVector3(const geometry_msgs::msg::Point & point)
{
  return simple_ekf_core::Vector3(point.x, point.y, point.z);
}

inline simple_ekf_core::Rigid poseMsgToRigid(const geometry_msgs::msg::Pose & pose)
{
  return simple_ekf_core::Rigid(
    simple_ekf_core::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    pointToVector3(pose.position));
}

inline geometry_msgs::msg::Pose rigidToPoseMsg(const simple_ekf_core::Rigid & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();

  const simple_ekf_core::Quaternion rotation = transform.getRotation();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

/**
 * @param frame The filter's frame the pose is in, resolved from its frame id by the caller
 */
inline simple_ekf_core::PoseSample toPoseSample(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg, simple_ekf_core::SourceFrame frame)
{
  simple_ekf_core::PoseSample sample;
  sample.stamp = toNanoseconds(msg.header.stamp);
  sample.frame = frame;
  sample.pose = poseMsgToRigid(msg.pose.pose);
  sample.covariance = msg.pose.covariance;
  return sample;
}

/**
 * @param frame The filter's frame the velocity is in, which for a topic configured as body
 *        frame is the vehicle's whatever the message header says
 */
inline simple_ekf_core::TwistSample toTwistSample(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg, simple_ekf_core::SourceFrame frame)
{
  simple_ekf_core::TwistSample sample;
  sample.stamp = toNanoseconds(msg.header.stamp);
  sample.frame = frame;
  sample.linear = {msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};
  sample.covariance = msg.twist.covariance;
  return sample;
}

inline simple_ekf_core::ImuSample toImuSample(const sensor_msgs::msg::Imu & msg)
{
  simple_ekf_core::ImuSample sample;
  sample.stamp = toNanoseconds(msg.header.stamp);
  sample.linear_acceleration = {
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
  sample.angular_velocity = {
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
  return sample;
}

/**
 * @brief The filter's twist as a message. Its covariance is left at zero, as the filter
 *        does not estimate one.
 */
inline geometry_msgs::msg::TwistWithCovariance twistToMsg(
  const simple_ekf_core::TwistInBase & twist)
{
  geometry_msgs::msg::TwistWithCovariance twist_msg;
  twist_msg.twist.linear.x = twist.linear.x();
  twist_msg.twist.linear.y = twist.linear.y();
  twist_msg.twist.linear.z = twist.linear.z();
  twist_msg.twist.angular.x = twist.angular.x();
  twist_msg.twist.angular.y = twist.angular.y();
  twist_msg.twist.angular.z = twist.angular.z();
  return twist_msg;
}

}  // namespace simple_ekf

#endif  // SIMPLE_EKF__ROS_CONVERSIONS_HPP_
