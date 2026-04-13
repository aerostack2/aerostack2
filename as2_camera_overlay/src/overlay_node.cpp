#include "as2_camera_overlay/overlay_node.hpp"

#include <algorithm>
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
  startAttached();
}

OverlayNode::~OverlayNode() = default;

void OverlayNode::declareParameters()
{
  fixed_frame_ = getOrDeclareStr(this, "fixed_frame", "earth");
  near_plane_ = static_cast<float>(getOrDeclare<double>(this, "near_plane", 0.01));
  far_plane_ = static_cast<float>(getOrDeclare<double>(this, "far_plane", 1000.0));
  zoom_factor_ = static_cast<float>(getOrDeclare<double>(this, "zoom_factor", 0.01));

  image_topic_ = getOrDeclareStr(this, "input.image_topic", "camera/image_raw");
  camera_info_topic_ = getOrDeclareStr(this, "input.camera_info_topic", "camera/camera_info");
  output_topic_ = getOrDeclareStr(this, "output.topic", "camera/image_overlay");

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

  image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
    this, camera_info_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(20), *image_sub_, *info_sub_);
  sync_->registerCallback(
    std::bind(
      &OverlayNode::cameraCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(), "Subscribed to image '%s' and info '%s'",
    image_topic_.c_str(), camera_info_topic_.c_str());
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
  std::lock_guard<std::mutex> lk(render_mutex_);

  renderer_->ensureRenderTarget(intrinsics.width, intrinsics.height);
  renderer_->setIntrinsics(intrinsics, near_plane_, far_plane_, zoom_factor_);

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
