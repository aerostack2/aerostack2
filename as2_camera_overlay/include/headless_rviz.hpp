#ifndef AS2_CAMERA_OVERLAY__HEADLESS_RVIZ_HPP_
#define AS2_CAMERA_OVERLAY__HEADLESS_RVIZ_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/handler_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>

namespace Ogre {
class SceneManager;
}

namespace as2_camera_overlay {

/**
 * @brief The "Fake Node" wrapper.
 *
 * RViz plugins expect to talk to an interface called RosNodeAbstractionIface.
 * This class simply wraps our real ROS 2 node so the plugins can use it
 * to subscribe to topics without realizing they are running in a headless
 * environment.
 */
class HeadlessRosNode
    : public rviz_common::ros_integration::RosNodeAbstractionIface {
public:
  explicit HeadlessRosNode(rclcpp::Node::SharedPtr node);
  std::string get_node_name() const override;
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const override;
  rclcpp::Node::SharedPtr get_raw_node() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief The "Fake Frame Manager".
 *
 * Standard RViz plugins constantly ask for "Transforms" (TF) to know where
 * coordinate frames (like "base_link" or "map") are.
 * This class connects RViz's spatial requests to our drone's real TF buffer.
 */
class HeadlessFrameManager : public rviz_common::FrameManagerIface {
  Q_OBJECT
public:
  HeadlessFrameManager(
      std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Clock::SharedPtr clock,
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node);

  void setFixedFrame(const std::string &frame) override;
  void setPause(bool) override {}
  bool getPause() override { return false; }
  void setSyncMode(SyncMode) override {}
  SyncMode getSyncMode() override { return SyncOff; }
  void syncTime(rclcpp::Time) override {}
  rclcpp::Time getTime() override;

  bool getTransform(const std::string &frame, Ogre::Vector3 &position,
                    Ogre::Quaternion &orientation) override;
  bool getTransform(const std::string &frame, rclcpp::Time time,
                    Ogre::Vector3 &position,
                    Ogre::Quaternion &orientation) override;
  bool transform(const std::string &frame, rclcpp::Time time,
                 const geometry_msgs::msg::Pose &pose, Ogre::Vector3 &position,
                 Ogre::Quaternion &orientation) override;

  void update() override {}
  bool frameHasProblems(const std::string &frame, std::string &error) override;
  bool transformHasProblems(const std::string &frame,
                            std::string &error) override;
  bool transformHasProblems(const std::string &frame, rclcpp::Time time,
                            std::string &error) override;

  const std::string &getFixedFrame() override;
  rviz_common::transformation::TransformationLibraryConnector::WeakPtr
  getConnector() override;
  std::shared_ptr<rviz_common::transformation::FrameTransformer>
  getTransformer() override;
  std::vector<std::string> getAllFrameNames() override;

public Q_SLOTS:
  void setTransformerPlugin(
      std::shared_ptr<rviz_common::transformation::FrameTransformer>) override {
  }

private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  rclcpp::Clock::SharedPtr clock_;
  std::string fixed_frame_{"map"};
  std::shared_ptr<rviz_default_plugins::transformation::TFFrameTransformer>
      transformer_;
};

/**
 * @brief The "No-Op Mouse Manager".
 *
 * RViz allows you to click on 3D objects to see details. This requires a
 * complex mouse-event system. Since we have no mouse, this class provides
 * "Empty" implementations that do nothing safely to prevent the plugins from
 * crashing.
 */
class NoOpHandlerManager
    : public rviz_common::interaction::HandlerManagerIface {
public:
  void addHandler(
      rviz_common::interaction::CollObjectHandle h,
      rviz_common::interaction::SelectionHandlerWeakPtr handler) override;
  void removeHandler(rviz_common::interaction::CollObjectHandle h) override;
  rviz_common::interaction::SelectionHandlerPtr
  getHandler(rviz_common::interaction::CollObjectHandle h) override;
  std::unique_lock<std::recursive_mutex> lock() override;
  std::unique_lock<std::recursive_mutex> lock(std::defer_lock_t d) override;
  void
  addListener(rviz_common::interaction::HandlerManagerListener *) override {}
  void
  removeListener(rviz_common::interaction::HandlerManagerListener *) override {}
  rviz_common::interaction::CollObjectHandle createHandle() override {
    return ++next_handle_;
  }
  void enableInteraction(bool) override {}
  bool getInteractionEnabled() const override { return false; }
  rviz_common::interaction::HandlerRange handlers() override;

private:
  rviz_common::interaction::M_ObjectHandleToSelectionHandler handlers_;
  std::recursive_mutex mutex_;
  rviz_common::interaction::CollObjectHandle next_handle_{0};
};

/**
 * @brief The Master "Fake RViz" Context.
 *
 * This is the ultimate "glue" object. Every RViz plugin is handed a pointer
 * to a DisplayContext. This class provides that pointer, but when a plugin
 * asks for a window, we say "there is no window." When it asks for the 3D
 * scene, we give it our Ogre scene manager.
 */
class HeadlessDisplayContext : public rviz_common::DisplayContext {
  Q_OBJECT
public:
  HeadlessDisplayContext(Ogre::SceneManager *scene_manager,
                         std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                         rclcpp::Node::SharedPtr node,
                         const std::string &fixed_frame);
  ~HeadlessDisplayContext() override = default;

  void setFixedFrame(const std::string &frame);
  bool renderDirty() const { return render_dirty_.exchange(false); }

  // --- DisplayContext Interface (Satisfying RViz Requirements) ---
  Ogre::SceneManager *getSceneManager() const override;
  rviz_common::WindowManagerInterface *getWindowManager() const override {
    return nullptr;
  } // NO WINDOW
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override {
    return nullptr; // NO MOUSE SELECTION
  }
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override {
    return handler_manager_;
  }
  std::shared_ptr<rviz_common::interaction::ViewPickerIface>
  getViewPicker() const override {
    return nullptr;
  }
  rviz_common::FrameManagerIface *getFrameManager() const override;
  QString getFixedFrame() const override;
  uint64_t getFrameCount() const override;
  rviz_common::DisplayFactory *getDisplayFactory() const override {
    return nullptr;
  }
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const override;

  // Ignore GUI input events.
  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}

  rviz_common::ToolManager *getToolManager() const override { return nullptr; }
  rviz_common::ViewManager *getViewManager() const override { return nullptr; }
  rviz_common::transformation::TransformationManager *
  getTransformationManager() override {
    return nullptr;
  }
  rviz_common::DisplayGroup *getRootDisplayGroup() const override {
    return nullptr;
  }

  uint32_t getDefaultVisibilityBit() const override { return 0xFFFFFFFF; }
  rviz_common::BitAllocator *visibilityBits() override { return nullptr; }

  void setStatus(const QString &) override {}
  QString getHelpPath() const override { return QString(); }

  std::shared_ptr<rclcpp::Clock> getClock() override;

  void lockRender() override {}
  void unlockRender() override {}

public Q_SLOTS:
  // Called by plugins when they change something and want the screen to
  // refresh.
  void queueRender() override;

private:
  Ogre::SceneManager *scene_manager_;
  std::shared_ptr<NoOpHandlerManager> handler_manager_{
      std::make_shared<NoOpHandlerManager>()};
  std::shared_ptr<HeadlessRosNode> ros_node_;
  std::shared_ptr<HeadlessFrameManager> frame_manager_;
  std::shared_ptr<rclcpp::Clock> clock_;
  mutable std::atomic<bool> render_dirty_{false};
  mutable std::atomic<uint64_t> frame_count_{0};
};

} // namespace as2_camera_overlay

#endif // AS2_CAMERA_OVERLAY__HEADLESS_RVIZ_HPP_
