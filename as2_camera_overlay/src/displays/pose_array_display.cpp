#include "displays/pose_array_display.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/axes.hpp>

#include "frame_helpers.hpp"
#include "param_helpers.hpp"

namespace as2_camera_overlay
{

PoseArrayDisplay::PoseArrayDisplay() = default;
PoseArrayDisplay::~PoseArrayDisplay() = default;

void PoseArrayDisplay::onInitialize(const DisplayContext & context)
{
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  scene_manager_ = context.scene_manager;
  logger_ = context.node->get_logger().get_child("PoseArrayDisplay");

  const std::string ns = context.param_namespace + ".";
  auto * node = context.node;

  const std::string topic = getOrDeclareStr(node, ns + "topic", "pose_array");
  shape_ = getOrDeclareStr(node, ns + "shape", "arrow");
  length_ = static_cast<float>(getOrDeclare<double>(node, ns + "length", 0.3));
  color_v_ = getOrDeclare<std::vector<double>>(
    node, ns + "color_rgba", std::vector<double>{1.0, 0.5, 0.0, 1.0});

  node_ = context.root_node->createChildSceneNode();
  node_->setVisible(false);

  auto cb = [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
      this->topicCallback(msg);
    };
  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(topic, 10, cb);
}

void PoseArrayDisplay::topicCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  last_msg_ = std::move(msg);
}

void PoseArrayDisplay::rebuildShapes(size_t n)
{
  if (shape_ == "axes") {
    while (axes_.size() < n) {
      axes_.emplace_back(
        std::make_unique<rviz_rendering::Axes>(scene_manager_, node_, length_, length_ * 0.1f));
    }
    while (axes_.size() > n) {
      axes_.pop_back();
    }
  } else {
    while (arrows_.size() < n) {
      auto arrow = std::make_unique<rviz_rendering::Arrow>(
        scene_manager_, node_,
        length_ * 0.7f, 0.03f, length_ * 0.3f, 0.07f);
      arrow->setColor(
        static_cast<float>(color_v_.size() > 0 ? color_v_[0] : 1.0),
        static_cast<float>(color_v_.size() > 1 ? color_v_[1] : 0.5),
        static_cast<float>(color_v_.size() > 2 ? color_v_[2] : 0.0),
        static_cast<float>(color_v_.size() > 3 ? color_v_[3] : 1.0));
      arrows_.emplace_back(std::move(arrow));
    }
    while (arrows_.size() > n) {
      arrows_.pop_back();
    }
  }
}

void PoseArrayDisplay::update(
  const rclcpp::Time & /*stamp*/, const std::string & fixed_frame)
{
  geometry_msgs::msg::PoseArray::ConstSharedPtr msg;
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

  rebuildShapes(msg->poses.size());
  node_->setVisible(isEnabled());

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    Ogre::Vector3 local_pos;
    Ogre::Quaternion local_rot;
    poseToOgre(msg->poses[i], local_pos, local_rot);
    const Ogre::Vector3 world_pos = frame_pos + frame_rot * local_pos;
    const Ogre::Quaternion world_rot = frame_rot * local_rot;
    if (shape_ == "axes") {
      axes_[i]->setPosition(world_pos);
      axes_[i]->setOrientation(world_rot);
    } else {
      arrows_[i]->setPosition(world_pos);
      arrows_[i]->setOrientation(
        world_rot * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
    }
  }
}

void PoseArrayDisplay::setEnabled(bool enabled)
{
  OverlayDisplayBase::setEnabled(enabled);
  if (node_ != nullptr) {
    node_->setVisible(enabled);
  }
}

}  // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(
  as2_camera_overlay::PoseArrayDisplay, as2_camera_overlay::OverlayDisplayBase)
