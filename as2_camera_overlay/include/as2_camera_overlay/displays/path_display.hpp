#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__PATH_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__PATH_DISPLAY_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include "as2_camera_overlay/overlay_display_base.hpp"

namespace Ogre
{
class SceneNode;
class SceneManager;
}
namespace rviz_rendering
{
class BillboardLine;
}

namespace as2_camera_overlay
{

class PathDisplay : public OverlayDisplayBase
{
public:
  PathDisplay();
  ~PathDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  void topicCallback(nav_msgs::msg::Path::ConstSharedPtr msg);

  Ogre::SceneNode * node_{nullptr};
  Ogre::SceneManager * scene_manager_{nullptr};
  std::unique_ptr<rviz_rendering::BillboardLine> line_;
  float line_width_{0.05f};
  std::vector<double> color_v_{0.1, 1.0, 0.1, 1.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;

  std::mutex mutex_;
  nav_msgs::msg::Path::ConstSharedPtr last_msg_;
  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.PathDisplay")};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__PATH_DISPLAY_HPP_
