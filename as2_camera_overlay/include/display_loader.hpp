#ifndef AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/display.hpp>
#include <rclcpp/rclcpp.hpp>

#include "headless_display_context.hpp"

namespace YAML { class Node; }

namespace as2_camera_overlay {

struct DisplayConfig
{
  std::string class_id;   // e.g. "rviz_default_plugins/Grid"
  std::string name;       // human-readable label
  // properties: arbitrary key→scalar, set via subProp(key)->setValue()
};

class DisplayLoader
{
public:
  explicit DisplayLoader(
    HeadlessDisplayContext * context,
    rclcpp::Logger logger);
  ~DisplayLoader();

  // Load a display by class ID, apply YAML properties map, and enable it.
  // Returns true on success.
  bool loadDisplay(const DisplayConfig & cfg, const YAML::Node & properties);

  // Call update(wall_dt_s, ros_dt_s) on every loaded display (seconds as float).
  void updateAll(float wall_dt, float ros_dt);

  size_t size() const { return displays_.size(); }

private:
  // Forbidden display class IDs that require a RenderPanel / WindowManager.
  static bool isExcluded(const std::string & class_id);

  // Recursively set properties from a YAML map node.
  static void applyProperties(
    rviz_common::properties::Property * prop,
    const YAML::Node & yaml_map,
    const rclcpp::Logger & logger);

  HeadlessDisplayContext * context_;
  rclcpp::Logger logger_;
  std::unique_ptr<pluginlib::ClassLoader<rviz_common::Display>> loader_;
  std::vector<std::shared_ptr<rviz_common::Display>> displays_;
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
