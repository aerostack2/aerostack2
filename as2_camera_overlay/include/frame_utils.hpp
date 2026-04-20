#ifndef AS2_CAMERA_OVERLAY__FRAME_UTILS_HPP_
#define AS2_CAMERA_OVERLAY__FRAME_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreMatrix4.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>

namespace as2_camera_overlay {

// --- Parameter Helpers (Steering the AI) ---

/**
 * @brief Grabs a ROS parameter or declares it with a default if it's missing.
 */
template <typename T>
T getOrDeclare(rclcpp::Node *node, const std::string &name,
               const T &default_value) {
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_parameter_value().get<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

/**
 * @brief Helper for string parameters.
 */
inline std::string getOrDeclareStr(rclcpp::Node *node, const std::string &name,
                                   const char *default_value) {
  return getOrDeclare<std::string>(node, name, std::string(default_value));
}

// --- Camera Projection (The Lens Math) ---

/**
 * @brief Parsed camera lens data.
 */
struct Intrinsics {
  unsigned int width{0};  ///< Video width in pixels.
  unsigned int height{0}; ///< Video height in pixels.
  double fx{0.0};         ///< Horizontal focal length.
  double fy{0.0};         ///< Vertical focal length.
  double cx{0.0};         ///< Lens center (X).
  double cy{0.0};         ///< Lens center (Y).
  double tx{0.0};         ///< Horizontal stereo offset.
  double ty{0.0};         ///< Vertical stereo offset.

  bool valid() const { return width > 0 && height > 0 && fx > 0.0 && fy > 0.0; }

  bool operator==(const Intrinsics &o) const {
    return width == o.width && height == o.height && fx == o.fx && fy == o.fy &&
           cx == o.cx && cy == o.cy && tx == o.tx && ty == o.ty;
  }
  bool operator!=(const Intrinsics &o) const { return !(*this == o); }
};

/**
 * @brief Extracts focal lengths and optical centers from a ROS message.
 */
Intrinsics intrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo &info);

/**
 * @brief The "Secret Hack". Builds a 4x4 math matrix that warps 3D graphics
 * to exactly match how the real physical camera lens warps light.
 */
Ogre::Matrix4 buildProjectionMatrix(const Intrinsics &k, float near_plane,
                                    float far_plane, float zoom_factor);

/**
 * @brief Adjusts the camera position for stereo-offset (if using a dual-lens
 * camera).
 */
void applyStereoBaseline(Ogre::Vector3 &position,
                         const Ogre::Quaternion &orientation,
                         const Intrinsics &k);

// --- Frame Helpers (Format Converters) ---

/**
 * @brief Converts ROS Points to Ogre Vectors.
 */
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point &p);
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 &v);

/**
 * @brief Converts ROS Quaternions (rotations) to Ogre Quaternions.
 */
Ogre::Quaternion toOgreQuaternion(const geometry_msgs::msg::Quaternion &q);

/**
 * @brief Translates a full ROS Pose (Pos+Rot) to Ogre types.
 */
void poseToOgre(const geometry_msgs::msg::Pose &pose, Ogre::Vector3 &position,
                Ogre::Quaternion &orientation);

/**
 * @brief Translates a ROS Transform to Ogre types.
 */
void transformToOgre(const geometry_msgs::msg::Transform &transform,
                     Ogre::Vector3 &position, Ogre::Quaternion &orientation);

/**
 * @brief Fixes the coordinate system.
 * Computer Vision uses: Z forward, X right, Y down.
 * Ogre3D Graphics uses: Z back, X right, Y up.
 */
Ogre::Quaternion visionToOgreRotation(const Ogre::Quaternion &q);

/**
 * @brief The bridge to the TF library. Finds 3D objects in the world.
 */
bool lookupTransformOgre(const tf2_ros::Buffer &buffer,
                         const std::string &target_frame,
                         const std::string &source_frame,
                         const rclcpp::Time &stamp, Ogre::Vector3 &position,
                         Ogre::Quaternion &orientation,
                         std::string *error = nullptr);

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__FRAME_UTILS_HPP_
