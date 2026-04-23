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

/*!******************************************************************************
 *  \file       frame_utils.hpp
 *  \brief      frame utils implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#ifndef FRAME_UTILS_HPP_
#define FRAME_UTILS_HPP_

#include <OgreMatrix4.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>

namespace as2_camera_overlay
{
template<typename T>
T getOrDeclare(
  rclcpp::Node * node, const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_parameter_value().get<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

inline std::string getOrDeclareStr(
  rclcpp::Node * node, const std::string & name,
  const char * default_value)
{
  return getOrDeclare<std::string>(node, name, std::string(default_value));
}

struct Intrinsics
{
  unsigned int width{0};
  unsigned int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double tx{0.0};
  double ty{0.0};
  bool valid() const {return width > 0 && height > 0 && fx > 0.0 && fy > 0.0;}
  bool operator==(const Intrinsics & o) const
  {
    return width == o.width && height == o.height && fx == o.fx && fy == o.fy &&
           cx == o.cx && cy == o.cy && tx == o.tx && ty == o.ty;
  }
  bool operator!=(const Intrinsics & o) const {return !(*this == o);}
};

Intrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo & info);
Ogre::Matrix4 buildProjectionMatrix(
  const Intrinsics & k, float near_plane,
  float far_plane, float zoom_factor);
void applyStereoBaseline(
  Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation,
  const Intrinsics & k);
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point & p);
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 & v);
Ogre::Quaternion toOgreQuaternion(const geometry_msgs::msg::Quaternion & q);
void poseToOgre(
  const geometry_msgs::msg::Pose & pose, Ogre::Vector3 & position,
  Ogre::Quaternion & orientation);
void transformToOgre(
  const geometry_msgs::msg::Transform & transform,
  Ogre::Vector3 & position, Ogre::Quaternion & orientation);
Ogre::Quaternion visionToOgreRotation(const Ogre::Quaternion & q);
bool lookupTransformOgre(
  const tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp, Ogre::Vector3 & position,
  Ogre::Quaternion & orientation,
  std::string * error = nullptr);
}  // namespace as2_camera_overlay

#endif  // FRAME_UTILS_HPP_
