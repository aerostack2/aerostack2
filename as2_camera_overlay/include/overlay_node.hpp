#ifndef AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_
#define AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "camera_projection.hpp"
#include "display_loader.hpp"
#include "headless_display_context.hpp"
#include "overlay_renderer.hpp"

namespace as2_camera_overlay {

class OverlayNode : public rclcpp::Node {
public:
  explicit OverlayNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~OverlayNode() override;

private:
  void declareParameters();
  void initRenderer();
  void loadDisplays();
  void startAttached();

  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info);

  void renderAndPublish(const std_msgs::msg::Header &header,
                        const Intrinsics &intrinsics,
                        const std::string &camera_frame_id,
                        const sensor_msgs::msg::Image *background_image);

  bool lookupCameraPose(const std::string &camera_frame,
                        const rclcpp::Time &stamp, Ogre::Vector3 &position,
                        Ogre::Quaternion &orientation);

  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);

  std::string fixed_frame_;
  float near_plane_{0.01f};
  float far_plane_{1000.0f};
  float zoom_factor_{1.0f};
  float render_scale_{1.0f};
  double max_render_fps_{0.0};
  std::chrono::steady_clock::time_point last_render_time_{};

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<OverlayRenderer> renderer_;
  std::unique_ptr<HeadlessDisplayContext> display_context_;
  std::unique_ptr<DisplayLoader> display_loader_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  std::mutex camera_info_mutex_;
  std::mutex render_mutex_;
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_
