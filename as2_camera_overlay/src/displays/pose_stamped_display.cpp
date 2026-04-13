#include "as2_camera_overlay/displays/pose_stamped_display.hpp"

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreColourValue.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/axes.hpp>

#include "as2_camera_overlay/frame_helpers.hpp"
#include "as2_camera_overlay/param_helpers.hpp"

namespace as2_camera_overlay
{

PoseStampedDisplay::PoseStampedDisplay() = default;
PoseStampedDisplay::~PoseStampedDisplay() = default;

void PoseStampedDisplay::onInitialize(const DisplayContext & context)
{
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  logger_ = context.node->get_logger().get_child("PoseStampedDisplay");

  const std::string ns = context.param_namespace + ".";
  auto * node = context.node;

  const std::string topic = getOrDeclareStr(node, ns + "topic", "pose");
  shape_ = getOrDeclareStr(node, ns + "shape", "arrow");
  const double length = getOrDeclare<double>(node, ns + "length", 0.5);
  const std::vector<double> color_v = getOrDeclare<std::vector<double>>(
    node, ns + "color_rgba", std::vector<double>{1.0, 0.1, 0.1, 1.0});

  node_ = context.root_node->createChildSceneNode();

  if (shape_ == "axes") {
    axes_ = std::make_unique<rviz_rendering::Axes>(
      context.scene_manager, node_, static_cast<float>(length), 0.05f);
  } else {
    arrow_ = std::make_unique<rviz_rendering::Arrow>(
      context.scene_manager, node_,
      static_cast<float>(length * 0.7), 0.05f,
      static_cast<float>(length * 0.3), 0.1f);
    arrow_->setColor(
      static_cast<float>(color_v.size() > 0 ? color_v[0] : 1.0),
      static_cast<float>(color_v.size() > 1 ? color_v[1] : 0.1),
      static_cast<float>(color_v.size() > 2 ? color_v[2] : 0.1),
      static_cast<float>(color_v.size() > 3 ? color_v[3] : 1.0));
  }

  node_->setVisible(false);

  auto cb = [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      this->topicCallback(msg);
    };
  sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(topic, 10, cb);
}

void PoseStampedDisplay::topicCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  last_msg_ = std::move(msg);
}

void PoseStampedDisplay::update(
  const rclcpp::Time &, const std::string & fixed_frame)
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    msg = last_msg_;
  }
  if (!msg) {
    return;
  }

  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  std::string err;
  if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame, msg->header.frame_id, rclcpp::Time(0, 0),
      frame_pos, frame_rot, &err))
  {
    node_->setVisible(false);
    return;
  }

  Ogre::Vector3 local_pos;
  Ogre::Quaternion local_rot;
  poseToOgre(msg->pose, local_pos, local_rot);

  const Ogre::Vector3 world_pos = frame_pos + frame_rot * local_pos;
  const Ogre::Quaternion world_rot = frame_rot * local_rot;

  node_->setVisible(isEnabled());
  node_->setPosition(world_pos);
  node_->setOrientation(world_rot);

  if (arrow_) {
    arrow_->setOrientation(
      world_rot * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
    arrow_->setPosition(world_pos);
  }
}

void PoseStampedDisplay::setEnabled(bool enabled)
{
  OverlayDisplayBase::setEnabled(enabled);
  if (node_ != nullptr) {
    node_->setVisible(enabled && last_msg_ != nullptr);
  }
}

}  // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(
  as2_camera_overlay::PoseStampedDisplay, as2_camera_overlay::OverlayDisplayBase)
