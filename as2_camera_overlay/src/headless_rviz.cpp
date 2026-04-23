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
 *  \file       headless_rviz.cpp
 *  \brief      headless rviz implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#include "headless_rviz.hpp"
#include "frame_utils.hpp"
#include <OgreSceneManager.h>
#include <map>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <vector>
#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif
namespace as2_camera_overlay {
HeadlessRosNode::HeadlessRosNode(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {}
std::string HeadlessRosNode::get_node_name() const { return node_->get_name(); }
std::map<std::string, std::vector<std::string>>
HeadlessRosNode::get_topic_names_and_types() const {
  return node_->get_topic_names_and_types();
}
rclcpp::Node::SharedPtr HeadlessRosNode::get_raw_node() { return node_; }
HeadlessFrameManager::HeadlessFrameManager(
    std::shared_ptr<tf2_ros::Buffer> buffer, rclcpp::Clock::SharedPtr clock,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node)
    : buf_(std::move(buffer)), clock_(std::move(clock)),
      transformer_(
          std::make_shared<
              rviz_default_plugins::transformation::TFFrameTransformer>()) {
  transformer_->initialize(ros_node, clock_);
}
void HeadlessFrameManager::setFixedFrame(const std::string &frame) {
  fixed_frame_ = frame;
  Q_EMIT fixedFrameChanged();
}
rclcpp::Time HeadlessFrameManager::getTime() { return clock_->now(); }
bool HeadlessFrameManager::getTransform(const std::string &frame,
                                        Ogre::Vector3 &position,
                                        Ogre::Quaternion &orientation) {
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
bool HeadlessFrameManager::transform(const std::string &frame,
                                     rclcpp::Time time,
                                     const geometry_msgs::msg::Pose &pose,
                                     Ogre::Vector3 &position,
                                     Ogre::Quaternion &orientation) {
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, time, frame_pos,
                           frame_rot))
    return false;
  Ogre::Vector3 local_pos;
  Ogre::Quaternion local_rot;
  poseToOgre(pose, local_pos, local_rot);
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
void HeadlessDisplayContext::queueRender() {
  render_dirty_.store(true);
  frame_count_.fetch_add(1);
}
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
