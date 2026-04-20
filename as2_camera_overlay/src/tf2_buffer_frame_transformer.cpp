#include "tf2_buffer_frame_transformer.hpp"

#include <string>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>

namespace as2_camera_overlay {

Tf2BufferFrameTransformer::Tf2BufferFrameTransformer(std::shared_ptr<tf2_ros::Buffer> buffer)
: buf_(std::move(buffer)),
  connector_(std::make_shared<rviz_common::transformation::TransformationLibraryConnector>())
{}

std::vector<std::string> Tf2BufferFrameTransformer::getAllFrameNames() const
{
  return buf_->getAllFrameNames();
}

geometry_msgs::msg::PoseStamped Tf2BufferFrameTransformer::transform(
  const geometry_msgs::msg::PoseStamped & pose_in,
  const std::string & target_frame)
{
  try {
    geometry_msgs::msg::PoseStamped out;
    buf_->transform(pose_in, out, target_frame);
    return out;
  } catch (const tf2::TransformException & e) {
    throw rviz_common::transformation::FrameTransformerException(e.what());
  }
}

bool Tf2BufferFrameTransformer::frameHasProblems(
  const std::string & frame, std::string & error) const
{
  if (!buf_->_frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }
  return false;
}

rviz_common::transformation::TransformationLibraryConnector::WeakPtr
Tf2BufferFrameTransformer::getConnector()
{
  return connector_;
}

geometry_msgs::msg::TransformStamped Tf2BufferFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time) const
{
  return buf_->lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::msg::TransformStamped Tf2BufferFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame) const
{
  return buf_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

bool Tf2BufferFrameTransformer::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  std::string * error_msg) const
{
  return buf_->canTransform(target_frame, source_frame, time, error_msg);
}

bool Tf2BufferFrameTransformer::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  std::string * error_msg) const
{
  return buf_->canTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
}

tf2_ros::TransformStampedFuture Tf2BufferFrameTransformer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  tf2_ros::TransformReadyCallback callback)
{
  return buf_->waitForTransform(target_frame, source_frame, time, timeout, callback);
}

void Tf2BufferFrameTransformer::cancel(const tf2_ros::TransformStampedFuture & ts_future)
{
  buf_->cancel(ts_future);
}

}  // namespace as2_camera_overlay
