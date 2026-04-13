#ifndef AS2_CAMERA_OVERLAY__DISPLAYS__POSE_ARRAY_DISPLAY_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAYS__POSE_ARRAY_DISPLAY_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>

#include "as2_camera_overlay/overlay_display_base.hpp"

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

class PoseArrayDisplay : public OverlayDisplayBase
{
public:
  PoseArrayDisplay();
  ~PoseArrayDisplay() override;

  void onInitialize(const DisplayContext & context) override;
  void update(const rclcpp::Time & stamp, const std::string & fixed_frame) override;
  void setEnabled(bool enabled) override;

private:
  void topicCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void rebuildShapes(size_t n);

  Ogre::SceneNode * node_{nullptr};
  Ogre::SceneManager * scene_manager_{nullptr};
  std::string shape_{"arrow"};
  float length_{0.3f};
  std::vector<double> color_v_{1.0, 0.5, 0.0, 1.0};

  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows_;
  std::vector<std::unique_ptr<rviz_rendering::Axes>> axes_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr last_msg_;
  rclcpp::Logger logger_{rclcpp::get_logger("as2_camera_overlay.PoseArrayDisplay")};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAYS__POSE_ARRAY_DISPLAY_HPP_
