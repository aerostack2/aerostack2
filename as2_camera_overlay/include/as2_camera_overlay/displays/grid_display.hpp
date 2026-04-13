#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__GRID_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__GRID_DISPLAY_HPP_

#include <memory>
#include <string>

#include "as2_camera_overlay/overlay_display_base.hpp"

namespace Ogre
{
class SceneNode;
}
namespace rviz_rendering
{
class Grid;
}

namespace as2_camera_overlay
{

class GridDisplay : public OverlayDisplayBase
{
public:
  GridDisplay();
  ~GridDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  std::unique_ptr<rviz_rendering::Grid> grid_;
  Ogre::SceneNode * node_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string reference_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.GridDisplay")};
  std::string plane_{"XY"};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__GRID_DISPLAY_HPP_
