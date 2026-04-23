// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!******************************************************************************
 *  \file       display_loader.cpp
 *  \brief      display loader implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#include "display_loader.hpp"
#include <QColor>
#include <QString>
#include <QVariant>
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
#include <set>
#include <string>
#include <yaml-cpp/yaml.h>
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif
namespace as2_camera_overlay {
namespace {
const std::set<std::string> kExcluded{
    "rviz_default_plugins/Camera",
    "rviz_default_plugins/Image",
};
QVariant yamlToVariant(const YAML::Node &node, const std::string &prop_class) {
  if (prop_class == "rviz_common::properties::BoolProperty")
    return QVariant(node.as<bool>());
  if (prop_class == "rviz_common::properties::IntProperty")
    return QVariant(node.as<int>());
  if (prop_class == "rviz_common::properties::FloatProperty")
    return QVariant(node.as<double>());
  if (prop_class == "rviz_common::properties::ColorProperty") {
    auto str = node.as<std::string>();
    auto c = rviz_common::properties::parseColor(QString::fromStdString(str));
    return QVariant(c);
  }
  return QVariant(QString::fromStdString(node.as<std::string>()));
}
} // namespace
DisplayLoader::DisplayLoader(HeadlessDisplayContext *context,
                             rclcpp::Logger logger)
    : context_(context), logger_(std::move(logger)),
      loader_(std::make_unique<pluginlib::ClassLoader<rviz_common::Display>>(
          "rviz_common", "rviz_common::Display")) {}
DisplayLoader::~DisplayLoader() {
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
bool DisplayLoader::loadDisplay(const DisplayConfig &cfg,
                                const YAML::Node &props) {
  if (isExcluded(cfg.class_id)) {
    RCLCPP_ERROR(logger_,
                 "Display '%s' requires a screen and is not supported.",
                 cfg.class_id.c_str());
    return false;
  }
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
  try {
    display->initialize(context_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Plugin initialization failed: %s", e.what());
    return false;
  }
  if (props.IsMap()) {
    applyProperties(display.get(), props, logger_);
  }
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
void DisplayLoader::applyProperties(rviz_common::properties::Property *prop,
                                    const YAML::Node &yaml_map,
                                    const rclcpp::Logger &logger) {
  for (const auto &kv : yaml_map) {
    if (!kv.first.IsScalar())
      continue;
    const std::string key = kv.first.as<std::string>();
    auto *child = prop->subProp(QString::fromStdString(key));
    if (child == prop) {
      RCLCPP_WARN(logger, "Plugin property '%s' not found.", key.c_str());
      continue;
    }
    const YAML::Node &val = kv.second;
    if (val.IsMap()) {
      applyProperties(child, val, logger);
      continue;
    }
    if (!val.IsScalar())
      continue;
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
