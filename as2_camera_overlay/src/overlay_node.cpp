#include "overlay_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>

#include "frame_helpers.hpp"
#include "param_helpers.hpp"

namespace as2_camera_overlay
{

OverlayNode::OverlayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("as2_camera_overlay", options)
{
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  initRenderer();
  loadDisplays();
  startAttached();

  using namespace std::placeholders;
  parameter_callback_handle_ =
    this->add_on_set_parameters_callback(std::bind(&OverlayNode::onParameterChange, this, _1));
}

OverlayNode::~OverlayNode() = default;

void OverlayNode::declareParameters()
{
  fixed_frame_ = getOrDeclareStr(this, "fixed_frame", "earth");
  near_plane_ = static_cast<float>(getOrDeclare<double>(this, "near_plane", 0.01));
  far_plane_ = static_cast<float>(getOrDeclare<double>(this, "far_plane", 1000.0));
  zoom_factor_ = static_cast<float>(getOrDeclare<double>(this, "zoom_factor", 1.0));

  image_topic_ = getOrDeclareStr(this, "input.image_topic", "camera/image_raw");
  camera_info_topic_ = getOrDeclareStr(this, "input.camera_info_topic", "camera/camera_info");
  output_topic_ = getOrDeclareStr(this, "output.topic", "camera/image_overlay");

  enabled_displays_ = getOrDeclare<std::vector<std::string>>(
    this, "enabled_displays",
    std::vector<std::string>{
    "as2_camera_overlay/GridDisplay",
    "as2_camera_overlay/MarkerArrayDisplay",
  });

  render_scale_ = std::clamp(
    static_cast<float>(getOrDeclare<double>(this, "render_scale", 1.0)),
    0.1f, 1.0f);
  max_render_fps_ = std::max(0.0, getOrDeclare<double>(this, "max_render_fps", 0.0));
}

rcl_interfaces::msg::SetParametersResult OverlayNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "zoom_factor") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "zoom_factor must be a floating-point value";
        return result;
      }
      const double value = param.as_double();
      if (value <= 0.0) {
        result.successful = false;
        result.reason = "zoom_factor must be > 0";
        return result;
      }
      zoom_factor_ = static_cast<float>(value);
      RCLCPP_INFO(get_logger(), "Updated zoom_factor to %.4f", zoom_factor_);
    } else if (param.get_name() == "render_scale") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "render_scale must be a floating-point value";
        return result;
      }
      const float value = std::clamp(static_cast<float>(param.as_double()), 0.1f, 1.0f);
      render_scale_ = value;
      RCLCPP_INFO(get_logger(), "Updated render_scale to %.2f", render_scale_);
    } else if (param.get_name() == "max_render_fps") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "max_render_fps must be a floating-point value";
        return result;
      }
      max_render_fps_ = std::max(0.0, param.as_double());
      RCLCPP_INFO(get_logger(), "Updated max_render_fps to %.1f", max_render_fps_);
    }
  }
  return result;
}

void OverlayNode::initRenderer()
{
  renderer_ = std::make_unique<OverlayRenderer>();
  renderer_->initialize();
  renderer_->setShowCameraBackground(true);
}

void OverlayNode::loadDisplays()
{
  display_loader_ = std::make_unique<pluginlib::ClassLoader<OverlayDisplayBase>>(
    "as2_camera_overlay", "as2_camera_overlay::OverlayDisplayBase");

  for (const auto & class_id : enabled_displays_) {
    try {
      auto display = display_loader_->createSharedInstance(class_id);

      DisplayContext ctx;
      ctx.scene_manager = renderer_->sceneManager();
      ctx.root_node = renderer_->rootNode();
      ctx.node = this;
      ctx.tf_buffer = tf_buffer_;
      ctx.fixed_frame = fixed_frame_;
      ctx.display_name = class_id;

      std::string short_name = class_id;
      const auto slash = short_name.find_last_of('/');
      if (slash != std::string::npos) {
        short_name = short_name.substr(slash + 1);
      }
      ctx.param_namespace = "displays." + short_name;

      display->onInitialize(ctx);
      displays_.push_back(std::move(display));
      RCLCPP_INFO(get_logger(), "Loaded display plugin '%s'", class_id.c_str());
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load display '%s': %s", class_id.c_str(), e.what());
    }
  }
}

void OverlayNode::startAttached()
{
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_topic_, rclcpp::SensorDataQoS());

  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lk(camera_info_mutex_);
      latest_camera_info_ = msg;
    });

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
      sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
      {
        std::lock_guard<std::mutex> lk(camera_info_mutex_);
        info = latest_camera_info_;
      }

      if (!info) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 5000,
          "No CameraInfo received on '%s'. Publishing image passthrough on '%s'.",
          camera_info_topic_.c_str(), output_topic_.c_str());
        image_pub_->publish(*image);
        return;
      }

      cameraCallback(image, info);
    });

  RCLCPP_INFO(
    get_logger(), "Subscribed to image '%s' and latest info '%s'",
    image_topic_.c_str(), camera_info_topic_.c_str());
}

void OverlayNode::cameraCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  Intrinsics k = intrinsicsFromCameraInfo(*info);

  if (image->width > 0 && image->height > 0) {
    if (k.width != image->width || k.height != image->height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 5000,
        "CameraInfo size (%ux%u) differs from image size (%ux%u). "
        "Using image dimensions for render target.",
        k.width, k.height, image->width, image->height);
    }
    k.width = image->width;
    k.height = image->height;
  }

  if (!k.valid()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Invalid CameraInfo (width=%u height=%u fx=%.2f fy=%.2f). "
      "Publishing image passthrough.",
      k.width, k.height, k.fx, k.fy);
    image_pub_->publish(*image);
    return;
  }
  renderAndPublish(image->header, k, image->header.frame_id, image.get());
}

bool OverlayNode::lookupCameraPose(
  const std::string & camera_frame,
  const rclcpp::Time & /*stamp*/,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  // tf2 rejects frame IDs with a leading '/'
  const std::string frame =
    (!camera_frame.empty() && camera_frame[0] == '/') ? camera_frame.substr(1) : camera_frame;

  std::string err;
  if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame_, frame, rclcpp::Time(0, 0),
      position, orientation, &err))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "TF lookup %s -> %s failed: %s",
      fixed_frame_.c_str(), frame.c_str(), err.c_str());
    return false;
  }
  orientation = visionToOgreRotation(orientation);
  return true;
}

void OverlayNode::renderAndPublish(
  const std_msgs::msg::Header & header,
  const Intrinsics & intrinsics,
  const std::string & camera_frame_id,
  const sensor_msgs::msg::Image * background_image)
{
  // Frame throttling
  if (max_render_fps_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_render_time_).count();
    if (elapsed < 1.0 / max_render_fps_) {
      return;
    }
    last_render_time_ = now;
  }

  Ogre::Vector3 cam_pos;
  Ogre::Quaternion cam_rot;
  if (!lookupCameraPose(camera_frame_id, header.stamp, cam_pos, cam_rot)) {
    return;
  }
  applyStereoBaseline(cam_pos, cam_rot, intrinsics);

  // Compute scaled dimensions
  const unsigned int orig_w = intrinsics.width;
  const unsigned int orig_h = intrinsics.height;
  const bool need_scale = (render_scale_ < 1.0f);

  Intrinsics scaled_k = intrinsics;
  if (need_scale) {
    scaled_k.width = std::max(1u, static_cast<unsigned int>(orig_w * render_scale_));
    scaled_k.height = std::max(1u, static_cast<unsigned int>(orig_h * render_scale_));
    const double sx = static_cast<double>(scaled_k.width) / static_cast<double>(orig_w);
    const double sy = static_cast<double>(scaled_k.height) / static_cast<double>(orig_h);
    scaled_k.fx *= sx;
    scaled_k.fy *= sy;
    scaled_k.cx *= sx;
    scaled_k.cy *= sy;
  }

  // Pre-scale background image outside the render lock
  cv::Mat scaled_bg;
  if (need_scale && background_image != nullptr) {
    auto cv_ptr = cv_bridge::toCvCopy(*background_image, "bgr8");
    cv::resize(
      cv_ptr->image, scaled_bg,
      cv::Size(scaled_k.width, scaled_k.height), 0, 0, cv::INTER_AREA);
  }

  cv::Mat rendered;
  {
    std::lock_guard<std::mutex> lk(render_mutex_);

    renderer_->ensureRenderTarget(scaled_k.width, scaled_k.height);
    renderer_->setIntrinsics(scaled_k, near_plane_, far_plane_, zoom_factor_);
    renderer_->setCameraPose(cam_pos, cam_rot);

    if (background_image != nullptr) {
      if (need_scale) {
        renderer_->updateBackgroundImage(scaled_bg);
      } else {
        renderer_->updateBackgroundImage(*background_image);
      }
    }

    for (auto & display : displays_) {
      display->update(header.stamp, fixed_frame_);
    }

    rendered = renderer_->renderAndRead();
  }

  if (rendered.empty()) {
    return;
  }

  // Upscale back to original resolution
  if (need_scale) {
    cv::resize(rendered, rendered, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);
  }

  // Publish directly without cv_bridge copy
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header = header;
  msg->height = static_cast<uint32_t>(rendered.rows);
  msg->width = static_cast<uint32_t>(rendered.cols);
  msg->encoding = "bgra8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(rendered.step);
  msg->data.assign(rendered.data, rendered.data + (rendered.step * rendered.rows));
  image_pub_->publish(std::move(msg));
}

}  // namespace as2_camera_overlay
