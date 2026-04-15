#ifndef AS2_CAMERA_OVERLAY__OVERLAY_DISPLAY_BASE_HPP_
#define AS2_CAMERA_OVERLAY__OVERLAY_DISPLAY_BASE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace Ogre {
class SceneManager;
class SceneNode;
} // namespace Ogre

namespace as2_camera_overlay {

struct DisplayContext {
  Ogre::SceneManager *scene_manager{nullptr};
  Ogre::SceneNode *root_node{nullptr};
  rclcpp::Node *node{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::string fixed_frame;
  std::string display_name;
  std::string param_namespace;
};

class OverlayDisplayBase {
public:
  using SharedPtr = std::shared_ptr<OverlayDisplayBase>;

  virtual ~OverlayDisplayBase() = default;

  virtual void onInitialize(const DisplayContext &context) = 0;

  virtual void update(const rclcpp::Time &stamp,
                      const std::string &fixed_frame) = 0;

  virtual void setEnabled(bool enabled);

  bool isEnabled() const { return enabled_; }

  const std::string &name() const { return name_; }

protected:
  std::string name_;
  bool enabled_{true};
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__OVERLAY_DISPLAY_BASE_HPP_
