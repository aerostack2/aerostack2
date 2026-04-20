#include "display_loader.hpp"

#include <set>
#include <string>

#include <QColor>
#include <QString>
#include <QVariant>

#include <yaml-cpp/yaml.h>

// RViz property headers: These are used to set settings on the plugins.
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

// X11 macro leak guard.
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif

namespace as2_camera_overlay {

namespace {

// List of RViz plugins that simply cannot run without a screen.
// For example, "Camera" display tries to open its own small window.
const std::set<std::string> kExcluded{
    "rviz_default_plugins/Camera",
    "rviz_default_plugins/Image",
    // Hay mass,
};

/**
 * Helper: Converts a YAML setting (from a text file) into a Qt Variant
 * (which RViz plugins use to store their settings).
 */
QVariant yamlToVariant(const YAML::Node &node, const std::string &prop_class) {
  if (prop_class == "rviz_common::properties::BoolProperty")
    return QVariant(node.as<bool>());
  if (prop_class == "rviz_common::properties::IntProperty")
    return QVariant(node.as<int>());
  if (prop_class == "rviz_common::properties::FloatProperty")
    return QVariant(node.as<double>());
  if (prop_class == "rviz_common::properties::ColorProperty") {
    // RViz colors are stored as "R; G; B" strings (0-255).
    auto str = node.as<std::string>();
    auto c = rviz_common::properties::parseColor(QString::fromStdString(str));
    return QVariant(c);
  }
  // Default to treating the setting as a plain string.
  return QVariant(QString::fromStdString(node.as<std::string>()));
}

} // namespace

DisplayLoader::DisplayLoader(HeadlessDisplayContext *context,
                             rclcpp::Logger logger)
    : context_(context), logger_(std::move(logger)),
      // Connect to the ROS plugin system to find RViz Display types.
      loader_(std::make_unique<pluginlib::ClassLoader<rviz_common::Display>>(
          "rviz_common", "rviz_common::Display")) {}

DisplayLoader::~DisplayLoader() {
  // Disable plugins before the loader dies to prevent crashes.
  for (auto &d : displays_) {
    try {
      d->setEnabled(false);
    } catch (...) {
    }
  }
  displays_.clear();
}

bool DisplayLoader::isExcluded(const std::string &class_id) {
  return kExcluded.count(class_id) > 0;
}

/**
 * Loads an RViz display tool dynamically.
 */
bool DisplayLoader::loadDisplay(const DisplayConfig &cfg,
                                const YAML::Node &props) {
  // 1. Safety check: make sure the plugin doesn't require a screen.
  if (isExcluded(cfg.class_id)) {
    RCLCPP_ERROR(logger_,
                 "Display '%s' requires a screen and is not supported.",
                 cfg.class_id.c_str());
    return false;
  }

  // 2. Instantiate the plugin using pluginlib.
  std::shared_ptr<rviz_common::Display> display;
  try {
    display = loader_->createSharedInstance(cfg.class_id);
  } catch (const pluginlib::PluginlibException &e) {
    RCLCPP_ERROR(logger_, "Failed to load plugin '%s': %s",
                 cfg.class_id.c_str(), e.what());
    return false;
  }

  display->setClassId(QString::fromStdString(cfg.class_id));
  display->setName(
      QString::fromStdString(cfg.name.empty() ? cfg.class_id : cfg.name));

  // 3. Initialize it with our "Fake RViz" environment.
  try {
    display->initialize(context_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Plugin initialization failed: %s", e.what());
    return false;
  }

  // 4. Apply settings from the YAML file (like color or topic name).
  if (props.IsMap()) {
    applyProperties(display.get(), props, logger_);
  }

  // 5. Start the plugin. It will begin listening to ROS topics.
  try {
    display->setEnabled(true);
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed to enable plugin '%s'.",
                 cfg.class_id.c_str());
    return false;
  }

  displays_.push_back(std::move(display));
  RCLCPP_INFO(logger_, "Loaded RViz plugin: %s", cfg.class_id.c_str());
  return true;
}

/**
 * Triggers all plugins to update their internal 3D math for the current frame.
 */
void DisplayLoader::updateAll(float wall_dt, float ros_dt) {
  for (auto &d : displays_) {
    if (!d->isEnabled())
      continue;
    try {
      d->update(wall_dt, ros_dt);
    } catch (...) {
    }
  }
}

/**
 * Recursively applies YAML configuration keys to the internal property tree
 * of an RViz plugin.
 */
void DisplayLoader::applyProperties(rviz_common::properties::Property *prop,
                                    const YAML::Node &yaml_map,
                                    const rclcpp::Logger &logger) {
  for (const auto &kv : yaml_map) {
    if (!kv.first.IsScalar())
      continue;
    const std::string key = kv.first.as<std::string>();

    // Find the specific setting on the plugin by name.
    auto *child = prop->subProp(QString::fromStdString(key));
    if (child == prop) {
      RCLCPP_WARN(logger, "Plugin property '%s' not found.", key.c_str());
      continue;
    }

    const YAML::Node &val = kv.second;
    if (val.IsMap()) {
      // If the setting is a group of settings, dive deeper.
      applyProperties(child, val, logger);
      continue;
    }

    if (!val.IsScalar())
      continue;

    // Convert the text from YAML into the correct type for the plugin (number,
    // color, etc).
    const std::string prop_class = child->metaObject()->className();
    try {
      child->setValue(yamlToVariant(val, prop_class));
    } catch (const std::exception &e) {
      RCLCPP_WARN(logger, "Failed to set property '%s': %s", key.c_str(),
                  e.what());
    }
  }
}

} // namespace as2_camera_overlay
