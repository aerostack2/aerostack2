#include "display_loader.hpp"

#include <set>
#include <string>

#include <QColor>
#include <QString>
#include <QVariant>

#include <yaml-cpp/yaml.h>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

// X11 macro leak guard (Ogre pulls in X11)
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef Status
#undef Status
#endif
#ifdef Always
#undef Always
#endif

namespace as2_camera_overlay {

namespace {

// Display class IDs that require a RenderPanel / WindowManager — unsupported headless.
const std::set<std::string> kExcluded{
  "rviz_default_plugins/Camera",
  "rviz_default_plugins/Image",
  "rviz_default_plugins/ImageTransport",
  "rviz_default_plugins/DepthCloud",
  "rviz_default_plugins/InteractiveMarkers",
};

QVariant yamlToVariant(
  const YAML::Node & node,
  const std::string & prop_class)
{
  if (prop_class == "rviz_common::properties::BoolProperty") {
    return QVariant(node.as<bool>());
  }
  if (prop_class == "rviz_common::properties::IntProperty") {
    return QVariant(node.as<int>());
  }
  if (prop_class == "rviz_common::properties::FloatProperty") {
    return QVariant(node.as<double>());
  }
  if (prop_class == "rviz_common::properties::ColorProperty") {
    // Expect "R; G; B" (0-255 integers) as produced by rviz.
    auto str = node.as<std::string>();
    auto c = rviz_common::properties::parseColor(QString::fromStdString(str));
    return QVariant(c);
  }
  // String, RosTopicProperty, TfFrameProperty, EnumProperty, etc.
  return QVariant(QString::fromStdString(node.as<std::string>()));
}

}  // namespace

DisplayLoader::DisplayLoader(HeadlessDisplayContext * context, rclcpp::Logger logger)
: context_(context), logger_(std::move(logger)),
  loader_(std::make_unique<pluginlib::ClassLoader<rviz_common::Display>>(
      "rviz_common", "rviz_common::Display"))
{}

DisplayLoader::~DisplayLoader()
{
  // Disable and release displays before the loader is destroyed so the shared libs stay alive.
  for (auto & d : displays_) {
    try { d->setEnabled(false); } catch (...) {}
  }
  displays_.clear();
}

bool DisplayLoader::isExcluded(const std::string & class_id)
{
  return kExcluded.count(class_id) > 0;
}

bool DisplayLoader::loadDisplay(const DisplayConfig & cfg, const YAML::Node & props)
{
  if (isExcluded(cfg.class_id)) {
    RCLCPP_ERROR(
      logger_, "Display '%s' requires a RenderPanel and is not supported headless.",
      cfg.class_id.c_str());
    return false;
  }

  std::shared_ptr<rviz_common::Display> display;
  try {
    display = loader_->createSharedInstance(cfg.class_id);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(logger_, "Failed to load display '%s': %s", cfg.class_id.c_str(), e.what());
    return false;
  }

  display->setClassId(QString::fromStdString(cfg.class_id));
  display->setName(QString::fromStdString(cfg.name.empty() ? cfg.class_id : cfg.name));

  try {
    display->initialize(context_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Display '%s' threw during initialize(): %s", cfg.class_id.c_str(), e.what());
    return false;
  }

  if (props.IsMap()) {
    applyProperties(display.get(), props, logger_);
  }

  try {
    display->setEnabled(true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Display '%s' threw during setEnabled(true): %s",
      cfg.class_id.c_str(), e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(logger_, "Display '%s' threw unknown exception during setEnabled(true)",
      cfg.class_id.c_str());
    return false;
  }

  if (!display->isEnabled()) {
    RCLCPP_WARN(logger_, "Display '%s' is not enabled after setEnabled(true).",
      cfg.class_id.c_str());
  }

  displays_.push_back(std::move(display));
  RCLCPP_INFO(logger_, "Loaded display '%s' (%s)", cfg.class_id.c_str(),
    displays_.back()->getName().toStdString().c_str());
  return true;
}

void DisplayLoader::updateAll(float wall_dt, float ros_dt)
{
  for (auto & d : displays_) {
    if (!d->isEnabled()) { continue; }
    try {
      d->update(wall_dt, ros_dt);
    } catch (const std::exception & e) {
      RCLCPP_WARN_ONCE(logger_, "Display update threw: %s", e.what());
    }
  }
}

void DisplayLoader::applyProperties(
  rviz_common::properties::Property * prop,
  const YAML::Node & yaml_map,
  const rclcpp::Logger & logger)
{
  for (const auto & kv : yaml_map) {
    if (!kv.first.IsScalar()) { continue; }
    const std::string key = kv.first.as<std::string>();

    auto * child = prop->subProp(QString::fromStdString(key));
    if (child == prop) {
      // subProp() returns *this when the child is not found (FailureProperty pattern).
      RCLCPP_WARN(logger, "Property '%s' not found — skipping.", key.c_str());
      continue;
    }

    const YAML::Node & val = kv.second;

    if (val.IsMap()) {
      // Recurse into nested property groups (e.g. QosProfileProperty).
      applyProperties(child, val, logger);
      continue;
    }

    if (!val.IsScalar()) { continue; }

    const std::string prop_class = child->metaObject()->className();
    try {
      child->setValue(yamlToVariant(val, prop_class));
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Failed to set property '%s': %s", key.c_str(), e.what());
    }
  }
}

}  // namespace as2_camera_overlay
