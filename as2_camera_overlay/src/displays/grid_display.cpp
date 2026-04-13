#include "as2_camera_overlay/displays/grid_display.hpp"

#include <memory>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/objects/grid.hpp>

#include "as2_camera_overlay/frame_helpers.hpp"
#include "as2_camera_overlay/param_helpers.hpp"

namespace as2_camera_overlay
{

GridDisplay::GridDisplay() = default;
GridDisplay::~GridDisplay() = default;

void GridDisplay::onInitialize(const DisplayContext & context)
{
  name_ = context.display_name;
  tf_buffer_ = context.tf_buffer;
  logger_ = context.node->get_logger().get_child("GridDisplay");

  const std::string ns = context.param_namespace + ".";
  auto * node = context.node;

  const int cell_count = getOrDeclare<int>(node, ns + "cell_count", 25);
  const double cell_size = getOrDeclare<double>(node, ns + "cell_size", 1.0);
  const double line_width = getOrDeclare<double>(node, ns + "line_width", 0.03);
  reference_frame_ = getOrDeclareStr(node, ns + "reference_frame", "earth");
  plane_ = getOrDeclareStr(node, ns + "plane", "XY");
  const std::vector<double> color_v = getOrDeclare<std::vector<double>>(
    node, ns + "color_rgba", std::vector<double>{0.5, 0.5, 0.5, 0.5});

  Ogre::ColourValue color(
    static_cast<float>(color_v.size() > 0 ? color_v[0] : 0.5),
    static_cast<float>(color_v.size() > 1 ? color_v[1] : 0.5),
    static_cast<float>(color_v.size() > 2 ? color_v[2] : 0.5),
    static_cast<float>(color_v.size() > 3 ? color_v[3] : 0.5));

  node_ = context.root_node->createChildSceneNode();
  if (plane_ == "XY") {
    node_->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
  } else if (plane_ == "YZ") {
    node_->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z));
  }

  grid_ = std::make_unique<rviz_rendering::Grid>(
    context.scene_manager,
    node_,
    rviz_rendering::Grid::Lines,
    static_cast<uint32_t>(cell_count),
    static_cast<float>(cell_size),
    static_cast<float>(line_width),
    color);
  grid_->create();
}

void GridDisplay::update(const rclcpp::Time & /*stamp*/, const std::string & fixed_frame)
{
  if (node_ == nullptr || !tf_buffer_) {
    return;
  }
  Ogre::Vector3 position = Ogre::Vector3::ZERO;
  Ogre::Quaternion orientation = Ogre::Quaternion::IDENTITY;
  std::string error;
  if (reference_frame_.empty() || reference_frame_ == fixed_frame) {
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
  } else if (!lookupTransformOgre(
      *tf_buffer_, fixed_frame, reference_frame_, rclcpp::Time(0, 0),
      position, orientation, &error))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 5000,
      "GridDisplay: failed to look up %s -> %s: %s",
      fixed_frame.c_str(), reference_frame_.c_str(), error.c_str());
    return;
  }

  Ogre::Quaternion plane_rot = Ogre::Quaternion::IDENTITY;
  if (plane_ == "XY") {
    plane_rot = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
  } else if (plane_ == "YZ") {
    plane_rot = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z);
  }

  node_->setPosition(position);
  node_->setOrientation(orientation * plane_rot);
}

void GridDisplay::setEnabled(bool enabled)
{
  OverlayDisplayBase::setEnabled(enabled);
  if (node_ != nullptr) {
    node_->setVisible(enabled);
  }
}

}  // namespace as2_camera_overlay

PLUGINLIB_EXPORT_CLASS(as2_camera_overlay::GridDisplay, as2_camera_overlay::OverlayDisplayBase)
