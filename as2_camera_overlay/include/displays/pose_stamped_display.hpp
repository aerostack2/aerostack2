#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__POSE_STAMPED_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__POSE_STAMPED_DISPLAY_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "overlay_display_base.hpp"

namespace Ogre
{
class SceneNode;
}
namespace rviz_rendering
{
class Arrow;
class Axes;
}

namespace as2_camera_overlay
{

class PoseStampedDisplay : public OverlayDisplayBase
{
public:
  PoseStampedDisplay();
  ~PoseStampedDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  void topicCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  Ogre::SceneNode * node_{nullptr};
  std::unique_ptr<rviz_rendering::Arrow> arrow_;
  std::unique_ptr<rviz_rendering::Axes> axes_;
  std::string shape_{"arrow"};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_msg_;
  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.PoseStampedDisplay")};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__POSE_STAMPED_DISPLAY_HPP_
