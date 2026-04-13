#ifndef AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_
#define AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_loader.hpp>

#include "as2_camera_overlay/camera_projection.hpp"
#include "as2_camera_overlay/overlay_display_base.hpp"
#include "as2_camera_overlay/overlay_renderer.hpp"

namespace as2_camera_overlay
{

class OverlayNode : public rclcpp::Node
{
public:
  explicit OverlayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OverlayNode() override;

private:
  enum class BackgroundMode
  {
    Camera,
    Black,
    Color,
  };

  void declareParameters();
  void initRenderer();
  void loadDisplays();
  void startAttached();
  void startSynthetic();

  void cameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
  void syntheticTick();

  void renderAndPublish(
    const std_msgs::msg::Header & header,
    const Intrinsics & intrinsics,
    const std::string & camera_frame_id,
    const sensor_msgs::msg::Image * background_image);

  bool lookupCameraPose(
    const std::string & camera_frame,
    const rclcpp::Time & stamp,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation);

  std::string fixed_frame_;
  BackgroundMode background_mode_{BackgroundMode::Camera};
  float near_plane_{0.01f};
  float far_plane_{1000.0f};

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;

  double synthetic_rate_hz_{20.0};
  Intrinsics synthetic_intrinsics_{};
  std::string synthetic_frame_id_;
  Ogre::ColourValue synthetic_bg_color_{0.0f, 0.0f, 0.0f, 1.0f};

  std::vector<std::string> enabled_displays_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<OverlayRenderer> renderer_;
  std::unique_ptr<pluginlib::ClassLoader<OverlayDisplayBase>> display_loader_;
  std::vector<std::shared_ptr<OverlayDisplayBase>> displays_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::TimerBase::SharedPtr synthetic_timer_;

  std::mutex render_mutex_;
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_
