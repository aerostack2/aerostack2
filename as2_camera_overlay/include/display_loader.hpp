#ifndef AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
#define AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>

#include "headless_rviz.hpp"

// Forward declaration of YAML library classes.
namespace YAML {
class Node;
}

namespace as2_camera_overlay {

/**
 * @brief Simple container for display plugin configuration.
 */
struct DisplayConfig {
  std::string class_id; ///< The plugin ID (e.g. "rviz_default_plugins/Grid").
  std::string name;     ///< Human-readable label for logs.
};

/**
 * @brief The "Plugin Manager" class. Handles dynamic loading of RViz tools.
 *
 * Instead of writing our own code to draw 3D objects, this class uses ROS's
 * pluginlib system to load the exact same drawing code that RViz uses.
 */
class DisplayLoader {
public:
  /**
   * @brief Constructor. Sets up the plugin loader.
   * @param context The fake RViz environment required by plugins.
   * @param logger ROS logger for reporting errors.
   */
  explicit DisplayLoader(HeadlessDisplayContext *context,
                         rclcpp::Logger logger);

  /**
   * @brief Destructor. Cleanly unloads all plugins.
   */
  ~DisplayLoader();

  /**
   * @brief The core feature: Loads a plugin by ID and applies settings from a
   * YAML file.
   *
   * @param cfg The plugin ID and name.
   * @param properties YAML map containing settings like "Color", "Cell Size",
   * or "Topic".
   * @return true if the plugin was loaded and started successfully.
   */
  bool loadDisplay(const DisplayConfig &cfg, const YAML::Node &properties);

  /**
   * @brief Commands all loaded plugins to update their 3D models for the
   * current frame.
   *
   * @param wall_dt Time passed since last frame (real-world time).
   * @param ros_dt Time passed since last frame (robot/sim time).
   */
  void updateAll(float wall_dt, float ros_dt);

  /**
   * @brief Returns how many plugins are currently active.
   */
  size_t size() const { return displays_.size(); }

private:
  /**
   * @brief Filters out RViz plugins that require a physical window or screen.
   */
  static bool isExcluded(const std::string &class_id);

  /**
   * @brief Translates YAML configuration keys into RViz plugin properties.
   */
  static void applyProperties(rviz_common::properties::Property *prop,
                              const YAML::Node &yaml_map,
                              const rclcpp::Logger &logger);

  HeadlessDisplayContext *context_; ///< The fake RViz context.
  rclcpp::Logger logger_;           ///< ROS Logger.
  std::unique_ptr<pluginlib::ClassLoader<rviz_common::Display>>
      loader_; ///< The plugin library engine.
  std::vector<std::shared_ptr<rviz_common::Display>>
      displays_; ///< List of currently active plugins.
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__DISPLAY_LOADER_HPP_
