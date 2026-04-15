#ifndef AS2_CAMERA_OVERLAY__FRAME_HELPERS_HPP_
#define AS2_CAMERA_OVERLAY__FRAME_HELPERS_HPP_

#include <memory>
#include <string>

#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace as2_camera_overlay
{

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point & p);
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 & v);
Ogre::Quaternion toOgreQuaternion(const geometry_msgs::msg::Quaternion & q);

void poseToOgre(
  const geometry_msgs::msg::Pose & pose,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation);

void transformToOgre(
  const geometry_msgs::msg::Transform & transform,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation);

Ogre::Quaternion visionToOgreRotation(const Ogre::Quaternion & q);

bool lookupTransformOgre(
  const tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation,
  std::string * error = nullptr);

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__FRAME_HELPERS_HPP_
