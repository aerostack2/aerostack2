#include "camera_projection.hpp"

namespace as2_camera_overlay
{

Intrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo & info)
{
  Intrinsics k;
  k.width = info.width;
  k.height = info.height;
  const auto & p = info.p;
  k.fx = p[0];
  k.fy = p[5];
  k.cx = p[2];
  k.cy = p[6];
  const double tx_num = p[3];
  const double ty_num = p[7];
  k.tx = (k.fx != 0.0) ? (-tx_num / k.fx) : 0.0;
  k.ty = (k.fy != 0.0) ? (-ty_num / k.fy) : 0.0;
  return k;
}

Ogre::Matrix4 buildProjectionMatrix(
  const Intrinsics & k,
  float near_plane,
  float far_plane,
  float zoom_factor)
{
  Ogre::Matrix4 m = Ogre::Matrix4::ZERO;
  const float fx = static_cast<float>(k.fx);
  const float fy = static_cast<float>(k.fy);
  const float cx = static_cast<float>(k.cx);
  const float cy = static_cast<float>(k.cy);
  const float w = static_cast<float>(k.width);
  const float h = static_cast<float>(k.height);

  m[0][0] = (2.0f * fx / w) * zoom_factor;
  m[1][1] = (2.0f * fy / h) * zoom_factor;

  m[0][2] = 2.0f * (0.5f - cx / w) * zoom_factor;
  m[1][2] = 2.0f * (cy / h - 0.5f) * zoom_factor;

  m[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  m[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);

  m[3][2] = -1.0f;
  return m;
}

void applyStereoBaseline(
  Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation,
  const Intrinsics & k)
{
  const Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  const Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + right * static_cast<float>(k.tx) + down * static_cast<float>(k.ty);
}

}  // namespace as2_camera_overlay
