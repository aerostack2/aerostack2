#include "headless_display_context.hpp"

namespace as2_camera_overlay {

HeadlessDisplayContext::HeadlessDisplayContext(
  Ogre::SceneManager * scene_manager,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  rclcpp::Node::SharedPtr node,
  const std::string & fixed_frame)
: scene_manager_(scene_manager),
  ros_node_(std::make_shared<HeadlessRosNode>(node)),
  frame_manager_(std::make_shared<HeadlessFrameManager>(tf_buffer, node->get_clock(), ros_node_)),
  clock_(node->get_clock())
{
  frame_manager_->setFixedFrame(fixed_frame);
}

void HeadlessDisplayContext::setFixedFrame(const std::string & frame)
{
  frame_manager_->setFixedFrame(frame);
}

Ogre::SceneManager * HeadlessDisplayContext::getSceneManager() const
{
  return scene_manager_;
}

rviz_common::FrameManagerIface * HeadlessDisplayContext::getFrameManager() const
{
  return frame_manager_.get();
}

QString HeadlessDisplayContext::getFixedFrame() const
{
  return QString::fromStdString(frame_manager_->getFixedFrame());
}

uint64_t HeadlessDisplayContext::getFrameCount() const
{
  return ++frame_count_;
}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
HeadlessDisplayContext::getRosNodeAbstraction() const
{
  return ros_node_;
}

std::shared_ptr<rclcpp::Clock> HeadlessDisplayContext::getClock()
{
  return clock_;
}

void HeadlessDisplayContext::queueRender()
{
  render_dirty_.store(true);
}

}  // namespace as2_camera_overlay
