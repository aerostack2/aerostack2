#include "displays/path_display.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <OgreColourValue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include "frame_helpers.hpp"
#include "param_helpers.hpp"

namespace as2_camera_overlay
{

PathDisplay::PathDisplay() = default;
PathDisplay::~PathDisplay() = default;

void PathDisplay::onInitialize(const DisplayContext & context)
{
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  scene_manager_ = context.scene_manager;
  logger_ = context.node->get_logger().get_child("PathDisplay");

  const std::string ns = context.param_namespace + ".";
  auto * node = context.node;

  const std::string topic = getOrDeclareStr(node, ns + "topic", "path");
  line_width_ = static_cast<float>(getOrDeclare<double>(node, ns + "line_width", 0.05));
  color_v_ = getOrDeclare<std::vector<double>>(
    node, ns + "color_rgba", std::vector<double>{0.1, 1.0, 0.1, 1.0});

  node_ = context.root_node->createChildSceneNode();
  line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, node_);
  line_->setLineWidth(line_width_);
  line_->setColor(
    static_cast<float>(color_v_.size() > 0 ? color_v_[0] : 0.1),
    static_cast<float>(color_v_.size() > 1 ? color_v_[1] : 1.0),
    static_cast<float>(color_v_.size() > 2 ? color_v_[2] : 0.1),
    static_cast<float>(color_v_.size() > 3 ? color_v_[3] : 1.0));

  auto cb = [this](nav_msgs::msg::Path::ConstSharedPtr msg) {this->topicCallback(msg);};
  sub_ = node->create_subscription<nav_msgs::msg::Path>(topic, 10, cb);
}

void PathDisplay::topicCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  last_msg_ = std::move(msg);
}

void PathDisplay::update(const rclcpp::Time & /*stamp*/, const std::string & fixed_frame)
{
  nav_msgs::msg::Path::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    msg = last_msg_;
  }
  if (!msg || msg->poses.empty()) {
    return;
  }
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  std::string err;
  if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame, msg->header.frame_id, rclcpp::Time(0, 0),
      frame_pos, frame_rot, &err))
  {
    return;
  }

  line_->clear();
  line_->setMaxPointsPerLine(static_cast<uint32_t>(msg->poses.size()));
  line_->setNumLines(1);
  line_->setLineWidth(line_width_);
  line_->setColor(
    static_cast<float>(color_v_.size() > 0 ? color_v_[0] : 0.1),
    static_cast<float>(color_v_.size() > 1 ? color_v_[1] : 1.0),
    static_cast<float>(color_v_.size() > 2 ? color_v_[2] : 0.1),
    static_cast<float>(color_v_.size() > 3 ? color_v_[3] : 1.0));

  for (const auto & ps : msg->poses) {
    const Ogre::Vector3 local = toOgreVector(ps.pose.position);
    const Ogre::Vector3 world = frame_pos + frame_rot * local;
    line_->addPoint(world);
  }
  node_->setVisible(isEnabled());
}

void PathDisplay::setEnabled(bool enabled)
{
  OverlayDisplayBase::setEnabled(enabled);
  if (node_ != nullptr) {
    node_->setVisible(enabled);
  }
}

}  // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(
  as2_camera_overlay::PathDisplay, as2_camera_overlay::OverlayDisplayBase)
