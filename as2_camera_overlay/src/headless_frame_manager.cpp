#include "headless_frame_manager.hpp"

#include <string>
#include <vector>

#include <tf2_ros/buffer.hpp>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>

#include "frame_helpers.hpp"

// X11 macro collision guard (Ogre headers pull in X11)
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

HeadlessFrameManager::HeadlessFrameManager(
  std::shared_ptr<tf2_ros::Buffer> buffer,
  rclcpp::Clock::SharedPtr clock,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node)
: buf_(std::move(buffer)),
  clock_(std::move(clock)),
  transformer_(std::make_shared<rviz_default_plugins::transformation::TFFrameTransformer>())
{
  transformer_->initialize(ros_node, clock_);
}

void HeadlessFrameManager::setFixedFrame(const std::string & frame)
{
  fixed_frame_ = frame;
  Q_EMIT fixedFrameChanged();
}

rclcpp::Time HeadlessFrameManager::getTime()
{
  return clock_->now();
}

bool HeadlessFrameManager::getTransform(
  const std::string & frame,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  return lookupTransformOgre(*buf_, fixed_frame_, frame, rclcpp::Time(0), position, orientation);
}

bool HeadlessFrameManager::getTransform(
  const std::string & frame,
  rclcpp::Time time,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  return lookupTransformOgre(*buf_, fixed_frame_, frame, time, position, orientation);
}

bool HeadlessFrameManager::transform(
  const std::string & frame,
  rclcpp::Time time,
  const geometry_msgs::msg::Pose & pose,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  Ogre::Vector3 frame_pos;
  Ogre::Quaternion frame_rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, time, frame_pos, frame_rot)) {
    return false;
  }
  Ogre::Vector3 local_pos;
  Ogre::Quaternion local_rot;
  poseToOgre(pose, local_pos, local_rot);
  position = frame_pos + frame_rot * local_pos;
  orientation = frame_rot * local_rot;
  return true;
}

bool HeadlessFrameManager::frameHasProblems(const std::string & frame, std::string & error)
{
  if (!buf_->_frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }
  return false;
}

bool HeadlessFrameManager::transformHasProblems(const std::string & frame, std::string & error)
{
  Ogre::Vector3 pos;
  Ogre::Quaternion rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, rclcpp::Time(0), pos, rot, &error)) {
    return true;
  }
  return false;
}

bool HeadlessFrameManager::transformHasProblems(
  const std::string & frame, rclcpp::Time time, std::string & error)
{
  Ogre::Vector3 pos;
  Ogre::Quaternion rot;
  if (!lookupTransformOgre(*buf_, fixed_frame_, frame, time, pos, rot, &error)) {
    return true;
  }
  return false;
}

const std::string & HeadlessFrameManager::getFixedFrame()
{
  return fixed_frame_;
}

rviz_common::transformation::TransformationLibraryConnector::WeakPtr
HeadlessFrameManager::getConnector()
{
  return transformer_->getConnector();
}

std::shared_ptr<rviz_common::transformation::FrameTransformer>
HeadlessFrameManager::getTransformer()
{
  return std::static_pointer_cast<rviz_common::transformation::FrameTransformer>(transformer_);
}

std::vector<std::string> HeadlessFrameManager::getAllFrameNames()
{
  return buf_->getAllFrameNames();
}

}  // namespace as2_camera_overlay
