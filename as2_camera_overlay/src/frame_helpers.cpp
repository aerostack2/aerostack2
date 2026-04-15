#include "frame_helpers.hpp"

#include <string>

#include <tf2/exceptions.h>

namespace as2_camera_overlay
{

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point & p)
{
  return Ogre::Vector3(
    static_cast<float>(p.x),
    static_cast<float>(p.y),
    static_cast<float>(p.z));
}

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 & v)
{
  return Ogre::Vector3(
    static_cast<float>(v.x),
    static_cast<float>(v.y),
    static_cast<float>(v.z));
}

Ogre::Quaternion toOgreQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return Ogre::Quaternion(
    static_cast<float>(q.w),
    static_cast<float>(q.x),
    static_cast<float>(q.y),
    static_cast<float>(q.z));
}

void poseToOgre(
  const geometry_msgs::msg::Pose & pose,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  position = toOgreVector(pose.position);
  orientation = toOgreQuaternion(pose.orientation);
}

void transformToOgre(
  const geometry_msgs::msg::Transform & transform,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  position = toOgreVector(transform.translation);
  orientation = toOgreQuaternion(transform.rotation);
}

Ogre::Quaternion visionToOgreRotation(const Ogre::Quaternion & q)
{
  return q * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
}

bool lookupTransformOgre(
  const tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation,
  std::string * error)
{
  try {
    const auto t = buffer.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
    (void)stamp;
    transformToOgre(t.transform, position, orientation);
    return true;
  } catch (const tf2::TransformException & ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

}  // namespace as2_camera_overlay
