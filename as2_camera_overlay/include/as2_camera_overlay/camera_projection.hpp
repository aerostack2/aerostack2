#ifndef AS2_CAMERA_OVERLAY__CAMERA_PROJECTION_HPP_
#define AS2_CAMERA_OVERLAY__CAMERA_PROJECTION_HPP_

#include <OgreMatrix4.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <sensor_msgs/msg/camera_info.hpp>

namespace as2_camera_overlay
{

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
};

Intrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo & info);

Ogre::Matrix4 buildProjectionMatrix(
  const Intrinsics & k,
  float near_plane,
  float far_plane,
  float zoom_factor);

void applyStereoBaseline(
  Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation,
  const Intrinsics & k);

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__CAMERA_PROJECTION_HPP_
