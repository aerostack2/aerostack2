#include "frame_utils.hpp"

#include <string>
#include <tf2/exceptions.h>

namespace as2_camera_overlay {

// --- Camera Projection (The Lens Math) ---

Intrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo &info) {
  Intrinsics k;
  k.width = info.width;
  k.height = info.height;
  const auto &p =
      info.p; // The 'P' matrix contains the focal lengths and center.
  k.fx = p[0];
  k.fy = p[5];
  k.cx = p[2];
  k.cy = p[6];

  // Handle stereo baseline (offsets between lenses in a multi-camera setup).
  const double tx_num = p[3];
  const double ty_num = p[7];
  k.tx = (k.fx != 0.0) ? (-tx_num / k.fx) : 0.0;
  k.ty = (k.fy != 0.0) ? (-ty_num / k.fy) : 0.0;
  return k;
}

/**
 * This function builds a 4x4 "Projection Matrix."
 * In 3D graphics, this matrix is responsible for the 'perspective' — it makes
 * distant objects look smaller and defines the Field of View.
 * We configure it to match the drone camera's real-world focal length.
 */
Ogre::Matrix4 buildProjectionMatrix(const Intrinsics &k, float near_plane,
                                    float far_plane, float zoom_factor) {
  Ogre::Matrix4 m = Ogre::Matrix4::ZERO;
  const float fx = static_cast<float>(k.fx);
  const float fy = static_cast<float>(k.fy);
  const float cx = static_cast<float>(k.cx);
  const float cy = static_cast<float>(k.cy);
  const float w = static_cast<float>(k.width);
  const float h = static_cast<float>(k.height);

  // Math to map 3D units to 2D pixels based on focal length.
  m[0][0] = (2.0f * fx / w) * zoom_factor;
  m[1][1] = (2.0f * fy / h) * zoom_factor;

  // Math to handle the image center offset (cx, cy).
  m[0][2] = 2.0f * (0.5f - cx / w) * zoom_factor;
  m[1][2] = 2.0f * (cy / h - 0.5f) * zoom_factor;

  // Setup the 'depth' range (near and far clipping planes).
  m[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  m[2][3] = -2.0f * far_plane * near_plane / (far_plane - near_plane);

  m[3][2] = -1.0f; // Required for perspective math.
  return m;
}

void applyStereoBaseline(Ogre::Vector3 &position,
                         const Ogre::Quaternion &orientation,
                         const Intrinsics &k) {
  const Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  const Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + right * static_cast<float>(k.tx) +
             down * static_cast<float>(k.ty);
}

// --- Frame Helpers (Translators) ---

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point &p) {
  return Ogre::Vector3(static_cast<float>(p.x), static_cast<float>(p.y),
                       static_cast<float>(p.z));
}

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 &v) {
  return Ogre::Vector3(static_cast<float>(v.x), static_cast<float>(v.y),
                       static_cast<float>(v.z));
}

Ogre::Quaternion toOgreQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return Ogre::Quaternion(static_cast<float>(q.w), static_cast<float>(q.x),
                          static_cast<float>(q.y), static_cast<float>(q.z));
}

void poseToOgre(const geometry_msgs::msg::Pose &pose, Ogre::Vector3 &position,
                Ogre::Quaternion &orientation) {
  position = toOgreVector(pose.position);
  orientation = toOgreQuaternion(pose.orientation);
}

void transformToOgre(const geometry_msgs::msg::Transform &transform,
                     Ogre::Vector3 &position, Ogre::Quaternion &orientation) {
  position = toOgreVector(transform.translation);
  orientation = toOgreQuaternion(transform.rotation);
}

/**
 * Physically rotates the coordinate system 180 degrees around X.
 * This aligns "Camera Space" (Y is down) with "Ogre Space" (Y is up).
 */
Ogre::Quaternion visionToOgreRotation(const Ogre::Quaternion &q) {
  return q * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
}

/**
 * The master lookup function. Connects to the TF library to find the 3D
 * position of the camera in the world.
 */
bool lookupTransformOgre(const tf2_ros::Buffer &buffer,
                         const std::string &target_frame,
                         const std::string &source_frame,
                         const rclcpp::Time &stamp, Ogre::Vector3 &position,
                         Ogre::Quaternion &orientation, std::string *error) {
  try {
    // Ask for the position of the drone's camera.
    const auto t =
        buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    (void)stamp;
    // Translate the result from ROS format to Ogre format.
    transformToOgre(t.transform, position, orientation);
    return true;
  } catch (const tf2::TransformException &ex) {
    if (error != nullptr)
      *error = ex.what();
    return false;
  }
}

} // namespace as2_camera_overlay
