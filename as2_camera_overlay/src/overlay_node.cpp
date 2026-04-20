#include "overlay_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QCoreApplication>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <yaml-cpp/yaml.h>

#include "frame_utils.hpp"

namespace as2_camera_overlay {

/**
 * Constructor: Sets up the base ROS node and initializes the spatial (TF)
 * system.
 */
OverlayNode::OverlayNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("as2_camera_overlay", options) {
  // Load configuration like which topics to listen to.
  declareParameters();

  // Initialize TF (Transform). This system allows the drone to know where its
  // parts are relative to each other and the world.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Boot the Ogre3D graphics engine.
  initRenderer();

  // We use a timer that fires instantly (0ns) to finish initialization.
  // We do this because loadDisplays() needs shared_from_this(), which is only
  // valid AFTER this constructor finishes.
  init_timer_ = this->create_wall_timer(std::chrono::nanoseconds(0), [this]() {
    init_timer_->cancel(); // Stop timer so it only runs once.
    loadDisplays();        // Load the RViz plugins (grids, markers).
    startAttached();       // Subscribe to the camera topics.

    // Setup a listener for parameter changes.
    using namespace std::placeholders;
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&OverlayNode::onParameterChange, this, _1));
  });
}

OverlayNode::~OverlayNode() = default;

/**
 * Loads user settings from ROS. If a setting is missing, it uses a default
 * value.
 */
void OverlayNode::declareParameters() {
  fixed_frame_ = getOrDeclareStr(this, "fixed_frame", "earth");
  near_plane_ =
      static_cast<float>(getOrDeclare<double>(this, "near_plane", 0.01));
  far_plane_ =
      static_cast<float>(getOrDeclare<double>(this, "far_plane", 1000.0));
  zoom_factor_ =
      static_cast<float>(getOrDeclare<double>(this, "zoom_factor", 1.0));

  image_topic_ = getOrDeclareStr(this, "input.image_topic", "camera/image_raw");
  camera_info_topic_ =
      getOrDeclareStr(this, "input.camera_info_topic", "camera/camera_info");
  output_topic_ = getOrDeclareStr(this, "output.topic", "camera/image_overlay");

  // The 'render_scale' allows us to render at a lower resolution to save GPU.
  render_scale_ = std::clamp(
      static_cast<float>(getOrDeclare<double>(this, "render_scale", 1.0)), 0.1f,
      1.0f);
  max_render_fps_ =
      std::max(0.0, getOrDeclare<double>(this, "max_render_fps", 0.0));
}

/**
 * Callback for live parameter updates from the user.
 */
rcl_interfaces::msg::SetParametersResult OverlayNode::onParameterChange(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "zoom_factor") {
      // ... basic validation logic ...
      zoom_factor_ = static_cast<float>(param.as_double());
      RCLCPP_INFO(get_logger(), "Updated zoom_factor to %.4f", zoom_factor_);
    } else if (param.get_name() == "render_scale") {
      // ... basic validation logic ...
      render_scale_ =
          std::clamp(static_cast<float>(param.as_double()), 0.1f, 1.0f);
      RCLCPP_INFO(get_logger(), "Updated render_scale to %.2f", render_scale_);
    } else if (param.get_name() == "max_render_fps") {
      max_render_fps_ = std::max(0.0, param.as_double());
      RCLCPP_INFO(get_logger(), "Updated max_render_fps to %.1f",
                  max_render_fps_);
    }
  }
  return result;
}

/**
 * Starts the Ogre3D engine. This allows us to "paint" 3D objects in memory.
 */
void OverlayNode::initRenderer() {
  renderer_ = std::make_unique<OverlayRenderer>();
  renderer_->initialize();
  renderer_->setShowCameraBackground(true); // Allow real video behind 3D world.
}

/**
 * Reads the YAML config and loads the specified RViz plugins (Display types).
 * This is the "secret sauce" that lets us get Grids/Markers/Paths for free.
 */
void OverlayNode::loadDisplays() {
  // Create a 'Fake RViz' context. Standard RViz plugins expect to be running
  // inside a windowed application. This fake context tricks them into running
  // headlessly by providing 'dummy' versions of RViz interfaces.
  display_context_ = std::make_unique<HeadlessDisplayContext>(
      renderer_->sceneManager(), tf_buffer_, shared_from_this(), fixed_frame_);

  display_loader_ =
      std::make_unique<DisplayLoader>(display_context_.get(), get_logger());

  // Parse the YAML block that contains the list of displays to load.
  std::string displays_yaml_str = getOrDeclareStr(this, "displays_yaml", "[]");

  try {
    YAML::Node root = YAML::Load(displays_yaml_str);
    if (root.IsSequence()) {
      for (const auto &entry : root) {
        DisplayConfig cfg;
        cfg.class_id = entry["class"].as<std::string>("");
        cfg.name = entry["name"].as<std::string>("");
        if (cfg.class_id.empty()) {
          RCLCPP_WARN(get_logger(),
                      "Display entry missing 'class' key — skipping.");
          continue;
        }
        YAML::Node props = entry["properties"];
        // Tell the loader to actually instantiate and configure the plugin.
        display_loader_->loadDisplay(cfg, props);
      }
    }
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse displays_yaml: %s", e.what());
  }
}

/**
 * Starts the ROS communication line. Subscribes to input camera topics.
 */
void OverlayNode::startAttached() {
  // Setup the publisher for the final, merged (Augmented) video stream.
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      output_topic_, rclcpp::SensorDataQoS());

  // Subscribe to CameraInfo. This tells us the lens properties (focal length,
  // etc).
  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(camera_info_mutex_); // thread-safe save
        latest_camera_info_ = msg;
      });

  // Subscribe to the raw video frames.
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
        {
          std::lock_guard<std::mutex> lk(camera_info_mutex_);
          info = latest_camera_info_;
        }

        // If we don't have the lens data yet, we can't draw the 3D markers
        // correctly. We just pass the video through untouched.
        if (!info) {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                               "No CameraInfo received. Passthrough mode.");
          image_pub_->publish(*image);
          return;
        }

        // Process this frame.
        cameraCallback(image, info);
      });

  RCLCPP_INFO(get_logger(), "Subscribed to camera feeds. Running.");
}

/**
 * Validates the camera info and triggers the final rendering process.
 */
void OverlayNode::cameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
  Intrinsics k = intrinsicsFromCameraInfo(*info);

  // Consistency check: ensure the info message matches the actual image pixels.
  if (image->width > 0 && image->height > 0) {
    if (k.width != image->width || k.height != image->height) {
      k.width = image->width;
      k.height = image->height;
    }
  }

  // Abort if the lens math is invalid (e.g. focal length is zero).
  if (!k.valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                         "Invalid CameraInfo.");
    image_pub_->publish(*image);
    return;
  }

  // Hand off to the heavy lifter.
  renderAndPublish(image->header, k, image->header.frame_id, image.get());
}

/**
 * Asks the TF library: "Where was the camera in the world at this exact
 * microsecond?"
 */
bool OverlayNode::lookupCameraPose(const std::string &camera_frame,
                                   const rclcpp::Time & /*stamp*/,
                                   Ogre::Vector3 &position,
                                   Ogre::Quaternion &orientation) {
  // Clean up the frame string.
  const std::string frame = (!camera_frame.empty() && camera_frame[0] == '/')
                                ? camera_frame.substr(1)
                                : camera_frame;

  std::string err;
  // Look up the 3D position/rotation of the camera frame relative to the world
  // frame.
  if (!lookupTransformOgre(*tf_buffer_, fixed_frame_, frame, rclcpp::Time(0, 0),
                           position, orientation, &err)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                         "TF lookup failed.");
    return false;
  }
  // Convert Computer Vision coordinates to Ogre3D graphics coordinates.
  orientation = visionToOgreRotation(orientation);
  return true;
}

/**
 * THE CORE PIPELINE: Processes a single video frame.
 */
void OverlayNode::renderAndPublish(
    const std_msgs::msg::Header &header, const Intrinsics &intrinsics,
    const std::string &camera_frame_id,
    const sensor_msgs::msg::Image *background_image) {

  // 1. Rate Limiting: Skip frames if we are exceeding max_render_fps.
  if (max_render_fps_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - last_render_time_).count();
    if (elapsed < 1.0 / max_render_fps_)
      return;
    last_render_time_ = now;
  }

  // 2. Localization: Find the camera in 3D space.
  Ogre::Vector3 cam_pos;
  Ogre::Quaternion cam_rot;
  if (!lookupCameraPose(camera_frame_id, header.stamp, cam_pos, cam_rot))
    return;
  applyStereoBaseline(cam_pos, cam_rot, intrinsics);

  // 3. The Performance Hack: Scaling.
  // We can render the 3D graphics on a smaller version of the image to save
  // massive GPU power.
  const unsigned int orig_w = intrinsics.width;
  const unsigned int orig_h = intrinsics.height;
  const bool need_scale = (render_scale_ < 1.0f);

  Intrinsics scaled_k = intrinsics;
  if (need_scale) {
    // Shrink the target render size.
    scaled_k.width =
        std::max(1u, static_cast<unsigned int>(orig_w * render_scale_));
    scaled_k.height =
        std::max(1u, static_cast<unsigned int>(orig_h * render_scale_));
    const double sx =
        static_cast<double>(scaled_k.width) / static_cast<double>(orig_w);
    const double sy =
        static_cast<double>(scaled_k.height) / static_cast<double>(orig_h);
    // Scale the focal lengths so the 3D projection still lines up.
    scaled_k.fx *= sx;
    scaled_k.fy *= sy;
    scaled_k.cx *= sx;
    scaled_k.cy *= sy;
  }

  // Use OpenCV to shrink the background video frame if needed.
  cv::Mat scaled_bg;
  if (need_scale && background_image != nullptr) {
    auto cv_ptr = cv_bridge::toCvCopy(*background_image, "bgr8");
    cv::resize(cv_ptr->image, scaled_bg,
               cv::Size(scaled_k.width, scaled_k.height), 0, 0, cv::INTER_AREA);
  }

  cv::Mat rendered;
  {
    // 4. The 3D Rendering (Atomic Block).
    std::lock_guard<std::mutex> lk(render_mutex_);

    // Tell Ogre how big the photo should be.
    renderer_->ensureRenderTarget(scaled_k.width, scaled_k.height);
    // Setup the virtual camera lens properties.
    renderer_->setIntrinsics(scaled_k, near_plane_, far_plane_, zoom_factor_);
    // Move virtual camera to physical camera's 3D location.
    renderer_->setCameraPose(cam_pos, cam_rot);

    // Paste the camera image behind the 3D world.
    if (background_image != nullptr) {
      if (need_scale)
        renderer_->updateBackgroundImage(scaled_bg);
      else
        renderer_->updateBackgroundImage(*background_image);
    }

    // Process internal plugin events (required for markers to update).
    QCoreApplication::processEvents();

    // Command all RViz plugins to refresh their 3D models.
    const auto wall_now = std::chrono::steady_clock::now();
    const float wall_dt_s =
        std::chrono::duration<float>(wall_now - last_render_time_).count();
    display_loader_->updateAll(wall_dt_s, wall_dt_s);

    rendered = renderer_->renderAndRead();
  }

  if (rendered.empty())
    return;

  // 5. Cleanup & Publish.
  // Stretch the result back to original size.
  if (need_scale) {
    cv::resize(rendered, rendered, cv::Size(orig_w, orig_h), 0, 0,
               cv::INTER_LINEAR);
  }

  // Pack the finished pixels into a ROS Image message and send it to the
  // drone's network.
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header = header;
  msg->height = static_cast<uint32_t>(rendered.rows);
  msg->width = static_cast<uint32_t>(rendered.cols);
  msg->encoding = "bgra8";
  msg->step = static_cast<uint32_t>(rendered.step);
  msg->data.assign(rendered.data,
                   rendered.data + (rendered.step * rendered.rows));
  image_pub_->publish(std::move(msg));
}

} // namespace as2_camera_overlay
