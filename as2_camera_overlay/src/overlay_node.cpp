#include "as2_camera_overlay/overlay_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <rclcpp/qos.hpp>

#include "as2_camera_overlay/frame_helpers.hpp"
#include "as2_camera_overlay/param_helpers.hpp"

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

  if (background_mode_ == BackgroundMode::Camera) {
    startAttached();
  } else {
    startSynthetic();
  }
}

OverlayNode::~OverlayNode() = default;

void OverlayNode::declareParameters()
{
  fixed_frame_ = getOrDeclareStr(this, "fixed_frame", "earth");

  const std::string bg = getOrDeclareStr(this, "background_mode", "camera");
  if (bg == "camera") {
    background_mode_ = BackgroundMode::Camera;
  } else if (bg == "black") {
    background_mode_ = BackgroundMode::Black;
  } else if (bg == "color") {
    background_mode_ = BackgroundMode::Color;
  } else {
    RCLCPP_WARN(
      get_logger(), "Unknown background_mode '%s', defaulting to 'camera'", bg.c_str());
    background_mode_ = BackgroundMode::Camera;
  }

  near_plane_ = static_cast<float>(getOrDeclare<double>(this, "near_plane", 0.01));
  far_plane_ = static_cast<float>(getOrDeclare<double>(this, "far_plane", 1000.0));

  image_topic_ = getOrDeclareStr(this, "input.image_topic", "camera/image_raw");
  camera_info_topic_ = getOrDeclareStr(this, "input.camera_info_topic", "camera/camera_info");
  output_topic_ = getOrDeclareStr(this, "output.topic", "camera/image_overlay");

  synthetic_rate_hz_ = getOrDeclare<double>(this, "synthetic.rate_hz", 20.0);
  const int width = getOrDeclare<int>(this, "synthetic.width", 1280);
  const int height = getOrDeclare<int>(this, "synthetic.height", 720);
  const double fx = getOrDeclare<double>(this, "synthetic.fx", 640.0);
  const double fy = getOrDeclare<double>(this, "synthetic.fy", 640.0);
  const double cx = getOrDeclare<double>(this, "synthetic.cx", width / 2.0);
  const double cy = getOrDeclare<double>(this, "synthetic.cy", height / 2.0);
  synthetic_intrinsics_.width = static_cast<unsigned int>(width);
  synthetic_intrinsics_.height = static_cast<unsigned int>(height);
  synthetic_intrinsics_.fx = fx;
  synthetic_intrinsics_.fy = fy;
  synthetic_intrinsics_.cx = cx;
  synthetic_intrinsics_.cy = cy;

  synthetic_frame_id_ = getOrDeclareStr(this, "synthetic.frame_id", "drone0/camera");

  const auto bg_color = getOrDeclare<std::vector<double>>(
    this, "synthetic.background_color", std::vector<double>{0.0, 0.0, 0.0, 1.0});
  synthetic_bg_color_ = Ogre::ColourValue(
    static_cast<float>(bg_color.size() > 0 ? bg_color[0] : 0.0),
    static_cast<float>(bg_color.size() > 1 ? bg_color[1] : 0.0),
    static_cast<float>(bg_color.size() > 2 ? bg_color[2] : 0.0),
    static_cast<float>(bg_color.size() > 3 ? bg_color[3] : 1.0));

  enabled_displays_ = getOrDeclare<std::vector<std::string>>(
    this, "enabled_displays",
    std::vector<std::string>{
    "as2_camera_overlay/GridDisplay",
    "as2_camera_overlay/MarkerArrayDisplay",
  });
}

void OverlayNode::initRenderer()
{
  renderer_ = std::make_unique<OverlayRenderer>();
  renderer_->initialize();
  renderer_->setBackgroundColor(synthetic_bg_color_);
  renderer_->setShowCameraBackground(background_mode_ == BackgroundMode::Camera);

  if (background_mode_ != BackgroundMode::Camera) {
    renderer_->ensureRenderTarget(
      synthetic_intrinsics_.width, synthetic_intrinsics_.height);
    renderer_->setIntrinsics(synthetic_intrinsics_, near_plane_, far_plane_);
  }
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

  image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, image_topic_);
  info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
    this, camera_info_topic_);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(20), *image_sub_, *info_sub_);
  sync_->registerCallback(
    std::bind(
      &OverlayNode::cameraCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(), "Attached mode: subscribed to image '%s' and info '%s'",
    image_topic_.c_str(), camera_info_topic_.c_str());
}

void OverlayNode::startSynthetic()
{
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_topic_, rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration<double>(1.0 / std::max(synthetic_rate_hz_, 1.0));
  synthetic_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {this->syntheticTick();});

  RCLCPP_INFO(
    get_logger(), "Synthetic mode: %ux%u @ %.1f Hz, frame_id='%s'",
    synthetic_intrinsics_.width, synthetic_intrinsics_.height,
    synthetic_rate_hz_, synthetic_frame_id_.c_str());
}

void OverlayNode::cameraCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  const Intrinsics k = intrinsicsFromCameraInfo(*info);
  if (!k.valid()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Invalid CameraInfo (width=%u height=%u fx=%.2f fy=%.2f)",
      k.width, k.height, k.fx, k.fy);
    return;
  }
  renderAndPublish(image->header, k, image->header.frame_id, image.get());
}

void OverlayNode::syntheticTick()
{
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = synthetic_frame_id_;
  renderAndPublish(header, synthetic_intrinsics_, synthetic_frame_id_, nullptr);
}

bool OverlayNode::lookupCameraPose(
  const std::string & camera_frame,
  const rclcpp::Time & /*stamp*/,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  std::string err;
  if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame_, camera_frame, rclcpp::Time(0, 0),
      position, orientation, &err))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "TF lookup %s -> %s failed: %s",
      fixed_frame_.c_str(), camera_frame.c_str(), err.c_str());
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
  std::lock_guard<std::mutex> lk(render_mutex_);

  renderer_->ensureRenderTarget(intrinsics.width, intrinsics.height);
  renderer_->setIntrinsics(intrinsics, near_plane_, far_plane_);

  Ogre::Vector3 cam_pos;
  Ogre::Quaternion cam_rot;
  if (!lookupCameraPose(camera_frame_id, header.stamp, cam_pos, cam_rot)) {
    return;
  }
  applyStereoBaseline(cam_pos, cam_rot, intrinsics);
  renderer_->setCameraPose(cam_pos, cam_rot);

  if (background_image != nullptr) {
    renderer_->updateBackgroundImage(*background_image);
  }

  for (auto & display : displays_) {
    display->update(header.stamp, fixed_frame_);
  }

  cv::Mat rendered = renderer_->renderAndRead();
  if (rendered.empty()) {
    return;
  }

  cv_bridge::CvImage cv_img;
  cv_img.header = header;
  cv_img.encoding = "bgr8";
  cv_img.image = rendered;
  image_pub_->publish(*cv_img.toImageMsg());
}

}  // namespace as2_camera_overlay
