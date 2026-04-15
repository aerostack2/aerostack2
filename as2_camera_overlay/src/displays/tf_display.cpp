#include "displays/tf_display.hpp"

#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/objects/axes.hpp>

#include "frame_helpers.hpp"
#include "param_helpers.hpp"

namespace as2_camera_overlay {

TFDisplay::TFDisplay() = default;
TFDisplay::~TFDisplay() = default;

void TFDisplay::onInitialize(const DisplayContext &context) {
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  scene_manager_ = context.scene_manager;
  logger_ = context.node->get_logger().get_child("TFDisplay");

  const std::string ns = context.param_namespace + ".";
  auto *node = context.node;

  const auto frame_ids = getOrDeclare<std::vector<std::string>>(
      node, ns + "frames", std::vector<std::string>{});
  axis_length_ =
      static_cast<float>(getOrDeclare<double>(node, ns + "length", 0.5));
  axis_radius_ =
      static_cast<float>(getOrDeclare<double>(node, ns + "radius", 0.05));

  node_ = context.root_node->createChildSceneNode();
  frames_.reserve(frame_ids.size());
  for (const auto &frame : frame_ids) {
    FrameEntry entry;
    entry.frame_id = frame;
    entry.axes = std::make_unique<rviz_rendering::Axes>(
        scene_manager_, node_, axis_length_, axis_radius_);
    frames_.push_back(std::move(entry));
  }
}

void TFDisplay::update(const rclcpp::Time &, const std::string &fixed_frame) {
  if (!tf_buffer_ || frames_.empty()) {
    return;
  }
  for (auto &entry : frames_) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    std::string error;
    if (!lookupTransformOgre(*tf_buffer_, fixed_frame, entry.frame_id,
                             rclcpp::Time(0, 0), position, orientation,
                             &error)) {
      entry.axes->getSceneNode()->setVisible(false);
      continue;
    }
    entry.axes->getSceneNode()->setVisible(isEnabled());
    entry.axes->setPosition(position);
    entry.axes->setOrientation(orientation);
  }
}

void TFDisplay::setEnabled(bool enabled) {
  OverlayDisplayBase::setEnabled(enabled);
  if (node_ != nullptr) {
    node_->setVisible(enabled);
  }
}

} // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(as2_camera_overlay::TFDisplay,
                       as2_camera_overlay::OverlayDisplayBase)
