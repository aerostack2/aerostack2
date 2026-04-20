#include "headless_rviz.hpp"

#include <map>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>
#include <tf2_ros/buffer.hpp>

#include "frame_utils.hpp"

// X11 macro collision guard (Ogre headers pull in X11)
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif

namespace as2_camera_overlay {

// --- HeadlessRosNode Implementation ---

HeadlessRosNode::HeadlessRosNode(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {}

std::string HeadlessRosNode::get_node_name() const { return node_->get_name(); }

std::map<std::string, std::vector<std::string>>
HeadlessRosNode::get_topic_names_and_types() const {
  return node_->get_topic_names_and_types();
}

rclcpp::Node::SharedPtr HeadlessRosNode::get_raw_node() { return node_; }

// --- HeadlessFrameManager Implementation ---

HeadlessFrameManager::HeadlessFrameManager(
    std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Clock::SharedPtr clock,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node)
    : buf_(std::move(buffer)), clock_(std::move(clock)),
      // Use RViz's standard TF transformer to handle the heavy lifting.
      transformer_(
          std::make_shared<
              rviz_default_plugins::transformation::TFFrameTransformer>()) {
  transformer_->initialize(ros_node, clock_);
}

void HeadlessFrameManager::setFixedFrame(const std::string &frame) {
  fixed_frame_ = frame;
  // Notify any listening plugins that the world frame changed.
  Q_EMIT fixedFrameChanged();
}

rclcpp::Time HeadlessFrameManager::getTime() { return clock_->now(); }

/**
 * Asks TF for a 3D position/rotation.
 */
bool HeadlessFrameManager::getTransform(const std::string &frame,
                                        Ogre::Vector3 &position,
                                        Ogre::Quaternion &orientation) {
  // Call our math helper to do the TF lookup.
  return lookupTransformOgre(*buf_, fixed_frame_, frame, rclcpp::Time(0),
                             position, orientation);
}

bool HeadlessFrameManager::getTransform(const std::string &frame,
                                        rclcpp::Time time,
                                        Ogre::Vector3 &position,
                                        Ogre::Quaternion &orientation) {
  return lookupTransformOgre(*buf_, fixed_frame_, frame, time, position,
                             orientation);
}

/**
 * Transforms a specific 3D point (pose) from its own frame into our world
 * frame.
 */
bool HeadlessFrameManager::transform(const std::string &frame,
                                     rclcpp::Time time,
                                     const geometry_msgs::msg::Pose &pose,
                                     Ogre::Vector3 &position,
                                     Ogre::Quaternion &orientation) {
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  // 1. Locate the frame in 3D space.
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, time, frame_pos,
                           frame_rot))
    return false;

  Ogre::Vector3 local_pos;
  Ogre::Quaternion local_rot;
  // 2. Convert the input pose to Ogre format.
  poseToOgre(pose, local_pos, local_rot);

  // 3. Chain the math: WorldPos = FramePos + (FrameRot * LocalPos).
  position = frame_pos + frame_rot * local_pos;
  orientation = frame_rot * local_rot;
  return true;
}

bool HeadlessFrameManager::frameHasProblems(const std::string &frame,
                                            std::string &error) {
  if (!buf_->_frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }
  return false;
}

bool HeadlessFrameManager::transformHasProblems(const std::string &frame,
                                                std::string &error) {
  Ogre::Vector3 pos;
  Ogre::Quaternion rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, rclcpp::Time(0), pos,
                           rot, &error))
    return true;
  return false;
}

bool HeadlessFrameManager::transformHasProblems(const std::string &frame,
                                                rclcpp::Time time,
                                                std::string &error) {
  Ogre::Vector3 pos;
  Ogre::Quaternion rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, time, pos, rot, &error))
    return true;
  return false;
}

const std::string &HeadlessFrameManager::getFixedFrame() {
  return fixed_frame_;
}

rviz_common::transformation::TransformationLibraryConnector::WeakPtr
HeadlessFrameManager::getConnector() {
  return transformer_->getConnector();
}

std::shared_ptr<rviz_common::transformation::FrameTransformer>
HeadlessFrameManager::getTransformer() {
  return std::static_pointer_cast<
      rviz_common::transformation::FrameTransformer>(transformer_);
}

std::vector<std::string> HeadlessFrameManager::getAllFrameNames() {
  return buf_->getAllFrameNames();
}

// --- HeadlessDisplayContext Implementation ---

HeadlessDisplayContext::HeadlessDisplayContext(
    Ogre::SceneManager *scene_manager,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, rclcpp::Node::SharedPtr node,
    const std::string &fixed_frame)
    : scene_manager_(scene_manager),
      ros_node_(std::make_shared<HeadlessRosNode>(node)),
      frame_manager_(std::make_shared<HeadlessFrameManager>(
          tf_buffer, node->get_clock(), ros_node_)),
      clock_(node->get_clock()) {
  frame_manager_->setFixedFrame(fixed_frame);
}

void HeadlessDisplayContext::setFixedFrame(const std::string &frame) {
  frame_manager_->setFixedFrame(frame);
}

Ogre::SceneManager *HeadlessDisplayContext::getSceneManager() const {
  return scene_manager_;
}

rviz_common::FrameManagerIface *
HeadlessDisplayContext::getFrameManager() const {
  return frame_manager_.get();
}

QString HeadlessDisplayContext::getFixedFrame() const {
  return QString::fromStdString(frame_manager_->getFixedFrame());
}

uint64_t HeadlessDisplayContext::getFrameCount() const {
  return frame_count_.load();
}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
HeadlessDisplayContext::getRosNodeAbstraction() const {
  return ros_node_;
}

std::shared_ptr<rclcpp::Clock> HeadlessDisplayContext::getClock() {
  return clock_;
}

/**
 * Triggered by plugins when they need to redraw.
 */
void HeadlessDisplayContext::queueRender() {
  render_dirty_.store(true);
  frame_count_.fetch_add(1);
}

// --- NoOpHandlerManager Implementation ---

void NoOpHandlerManager::addHandler(
    rviz_common::interaction::CollObjectHandle h,
    rviz_common::interaction::SelectionHandlerWeakPtr handler) {
  std::unique_lock<std::recursive_mutex> lk(mutex_);
  handlers_[h] = handler;
}
void NoOpHandlerManager::removeHandler(
    rviz_common::interaction::CollObjectHandle h) {
  std::unique_lock<std::recursive_mutex> lk(mutex_);
  handlers_.erase(h);
}
rviz_common::interaction::SelectionHandlerPtr
NoOpHandlerManager::getHandler(rviz_common::interaction::CollObjectHandle h) {
  std::unique_lock<std::recursive_mutex> lk(mutex_);
  auto it = handlers_.find(h);
  return (it != handlers_.end()) ? it->second.lock() : nullptr;
}
std::unique_lock<std::recursive_mutex> NoOpHandlerManager::lock() {
  return std::unique_lock<std::recursive_mutex>(mutex_);
}
std::unique_lock<std::recursive_mutex>
NoOpHandlerManager::lock(std::defer_lock_t d) {
  return std::unique_lock<std::recursive_mutex>(mutex_, d);
}
rviz_common::interaction::HandlerRange NoOpHandlerManager::handlers() {
  return rviz_common::interaction::HandlerRange(handlers_);
}

} // namespace as2_camera_overlay
