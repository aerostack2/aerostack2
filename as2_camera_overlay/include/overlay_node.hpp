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

#include "display_loader.hpp"
#include "frame_utils.hpp"
#include "headless_rviz.hpp"
#include "overlay_renderer.hpp"

namespace as2_camera_overlay {

/**
 * @brief The main ROS 2 node for the camera overlay system.
 *
 * OverlayNode acts as the project manager (The Coordinator). It:
 * 1. Loads configuration parameters (zoom, topics, render scale).
 * 2. Manages the 3D graphics engine (OverlayRenderer).
 * 3. Uses DisplayLoader to load RViz plugins for drawing grids/markers.
 * 4. Subscribes to raw camera images and camera info.
 * 5. Uses the TF system to find the drone's position in the world.
 * 6. Coordinates the 3D render-to-image process and publishes the augmented
 * video.
 */
class OverlayNode : public rclcpp::Node {
public:
  /**
   * @brief Initializes the node, sets up parameters, and starts the Ogre
   * engine.
   */
  explicit OverlayNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Cleanly shuts down the node.
   */
  ~OverlayNode() override;

private:
  /**
   * @brief Reads user settings from the ROS parameter server (e.g. from
   * defaults.yaml).
   */
  void declareParameters();

  /**
   * @brief Spawns and configures the Ogre3D graphics engine.
   */
  void initRenderer();

  /**
   * @brief Loads the RViz plugins (like MarkerArray, Grid) specified in the
   * config.
   */
  void loadDisplays();

  /**
   * @brief Sets up the ROS subscribers (Image/CameraInfo) and the result
   * publisher.
   */
  void startAttached();

  /**
   * @brief Entry point for every incoming camera frame.
   *
   * @param image The raw camera frame message.
   * @param info The camera's lens parameters (focal length, etc.).
   */
  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info);

  /**
   * @brief The core processing pipeline:
   * 1. Match the virtual 3D camera to the physical drone camera via TF.
   * 2. Apply background image and draw 3D markers.
   * 3. Publish the resulting augmented image.
   *
   * @param header ROS header containing timestamp (crucial for TF lookups).
   * @param intrinsics Simple struct containing parsed camera lens data.
   * @param camera_frame_id The TF frame name for the camera (e.g.,
   * "drone/camera").
   * @param background_image The raw image buffer to use as the 3D scene's
   * background.
   */
  void renderAndPublish(const std_msgs::msg::Header &header,
                        const Intrinsics &intrinsics,
                        const std::string &camera_frame_id,
                        const sensor_msgs::msg::Image *background_image);

  /**
   * @brief Calculates where the drone camera was in the world at a specific
   * time.
   *
   * @param camera_frame The frame of the camera to locate.
   * @param stamp The exact time the photo was taken.
   * @param position Output 3D vector for the graphics engine.
   * @param orientation Output rotation quaternion for the graphics engine.
   * @return true if TF was able to find the drone's position.
   */
  bool lookupCameraPose(const std::string &camera_frame,
                        const rclcpp::Time &stamp, Ogre::Vector3 &position,
                        Ogre::Quaternion &orientation);

  /**
   * @brief Handles updates to parameters (like zoom or scale) while the node is
   * running.
   */
  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);

  // --- Configuration ---
  std::string fixed_frame_;    ///< The global world frame (e.g. "earth").
  float near_plane_{0.01f};    ///< Minimum distance Ogre will draw.
  float far_plane_{1000.0f};   ///< Maximum distance Ogre will draw.
  float zoom_factor_{1.0f};    ///< Artificial zoom for the projection matrix.
  float render_scale_{1.0f};   ///< Resolution scale (0.5 = 25% size).
  double max_render_fps_{0.0}; ///< Limit frame rate. 0 = unlimited.
  std::chrono::steady_clock::time_point last_render_time_{};

  // --- ROS Topics ---
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;

  // --- Spatial Awareness (TF) ---
  std::shared_ptr<tf2_ros::Buffer>
      tf_buffer_; ///< Database of drone positions over time.
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; ///< Feeds data into the tf_buffer.

  // --- Major Component Instances ---
  std::unique_ptr<OverlayRenderer>
      renderer_; ///< The Ogre3D painter (The Artist).
  std::unique_ptr<HeadlessDisplayContext>
      display_context_; ///< The "Fake RViz" shell for plugins.
  std::unique_ptr<DisplayLoader>
      display_loader_; ///< The dynamic plugin manager.

  // --- Communication Handles ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // --- Lifecycle/Callback Handles ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // --- Synchronization ---
  std::mutex camera_info_mutex_; ///< Ensures camera info isn't updated while
                                 ///< we're using it.
  std::mutex render_mutex_; ///< Ensures we only render one frame at a time.
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__OVERLAY_NODE_HPP_
