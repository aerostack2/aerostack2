#ifndef AS2_CAMERA_OVERLAY__HEADLESS_DISPLAY_CONTEXT_HPP_
#define AS2_CAMERA_OVERLAY__HEADLESS_DISPLAY_CONTEXT_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <unordered_map>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/interaction/handler_manager_iface.hpp>

#include "headless_frame_manager.hpp"
#include "headless_ros_node.hpp"

namespace Ogre {
class SceneManager;
}

namespace as2_camera_overlay {

// No-op HandlerManager so SelectionHandler::registerHandle() doesn't null-deref.
class NoOpHandlerManager : public rviz_common::interaction::HandlerManagerIface
{
public:
  void addHandler(
    rviz_common::interaction::CollObjectHandle h,
    rviz_common::interaction::SelectionHandlerWeakPtr handler) override
  {
    std::unique_lock<std::recursive_mutex> lk(mutex_);
    handlers_[h] = handler;
  }
  void removeHandler(rviz_common::interaction::CollObjectHandle h) override
  {
    std::unique_lock<std::recursive_mutex> lk(mutex_);
    handlers_.erase(h);
  }
  rviz_common::interaction::SelectionHandlerPtr
  getHandler(rviz_common::interaction::CollObjectHandle h) override
  {
    std::unique_lock<std::recursive_mutex> lk(mutex_);
    auto it = handlers_.find(h);
    return (it != handlers_.end()) ? it->second.lock() : nullptr;
  }
  std::unique_lock<std::recursive_mutex> lock() override
  { return std::unique_lock<std::recursive_mutex>(mutex_); }
  std::unique_lock<std::recursive_mutex> lock(std::defer_lock_t d) override
  { return std::unique_lock<std::recursive_mutex>(mutex_, d); }
  void addListener(rviz_common::interaction::HandlerManagerListener *) override {}
  void removeListener(rviz_common::interaction::HandlerManagerListener *) override {}
  rviz_common::interaction::CollObjectHandle createHandle() override { return ++next_handle_; }
  void enableInteraction(bool) override {}
  bool getInteractionEnabled() const override { return false; }
  rviz_common::interaction::HandlerRange handlers() override
  { return rviz_common::interaction::HandlerRange(handlers_); }
private:
  rviz_common::interaction::M_ObjectHandleToSelectionHandler handlers_;
  std::recursive_mutex mutex_;
  rviz_common::interaction::CollObjectHandle next_handle_{0};
};

// Minimal DisplayContext implementation for headless (no-GUI) use.
// GUI-only methods (WindowManager, SelectionManager, ToolManager, etc.) return nullptr.
class HeadlessDisplayContext : public rviz_common::DisplayContext
{
  Q_OBJECT

public:
  HeadlessDisplayContext(
    Ogre::SceneManager * scene_manager,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    const std::string & fixed_frame);

  ~HeadlessDisplayContext() override = default;

  void setFixedFrame(const std::string & frame);
  bool renderDirty() const { return render_dirty_.exchange(false); }

  // --- DisplayContext interface ---
  Ogre::SceneManager * getSceneManager() const override;
  rviz_common::WindowManagerInterface * getWindowManager() const override { return nullptr; }

  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override { return nullptr; }

  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override { return handler_manager_; }

  std::shared_ptr<rviz_common::interaction::ViewPickerIface>
  getViewPicker() const override { return nullptr; }

  rviz_common::FrameManagerIface * getFrameManager() const override;

  QString getFixedFrame() const override;

  uint64_t getFrameCount() const override;

  rviz_common::DisplayFactory * getDisplayFactory() const override { return nullptr; }

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const override;

  void handleChar(QKeyEvent *, rviz_common::RenderPanel *) override {}
  void handleMouseEvent(const rviz_common::ViewportMouseEvent &) override {}

  rviz_common::ToolManager * getToolManager() const override { return nullptr; }
  rviz_common::ViewManager * getViewManager() const override { return nullptr; }

  rviz_common::transformation::TransformationManager *
  getTransformationManager() override { return nullptr; }

  rviz_common::DisplayGroup * getRootDisplayGroup() const override { return nullptr; }

  uint32_t getDefaultVisibilityBit() const override { return 0xFFFFFFFF; }

  rviz_common::BitAllocator * visibilityBits() override { return nullptr; }

  void setStatus(const QString &) override {}

  QString getHelpPath() const override { return QString(); }

  std::shared_ptr<rclcpp::Clock> getClock() override;

  void lockRender() override {}
  void unlockRender() override {}

public Q_SLOTS:
  void queueRender() override;

private:
  Ogre::SceneManager * scene_manager_;
  std::shared_ptr<NoOpHandlerManager> handler_manager_{std::make_shared<NoOpHandlerManager>()};
  std::shared_ptr<HeadlessRosNode> ros_node_;     // must be declared before frame_manager_
  std::shared_ptr<HeadlessFrameManager> frame_manager_;
  std::shared_ptr<rclcpp::Clock> clock_;
  mutable std::atomic<bool> render_dirty_{false};
  mutable std::atomic<uint64_t> frame_count_{0};
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__HEADLESS_DISPLAY_CONTEXT_HPP_
