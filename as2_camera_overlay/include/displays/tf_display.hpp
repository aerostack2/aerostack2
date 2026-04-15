#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__TF_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__TF_DISPLAY_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "overlay_display_base.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}
namespace rviz_rendering
{
class Axes;
}

namespace as2_camera_overlay
{

class TFDisplay : public OverlayDisplayBase
{
public:
  TFDisplay();
  ~TFDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  struct FrameEntry
  {
    std::string frame_id;
    std::unique_ptr<rviz_rendering::Axes> axes;
  };

  std::vector<FrameEntry> frames_;
  Ogre::SceneNode * node_{nullptr};
  Ogre::SceneManager * scene_manager_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.TFDisplay")};
  float axis_length_{0.5f};
  float axis_radius_{0.05f};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__TF_DISPLAY_HPP_
