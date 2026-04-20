#ifndef AS2_CAMERA_OVERLAY__TF2_BUFFER_FRAME_TRANSFORMER_HPP_
#define AS2_CAMERA_OVERLAY__TF2_BUFFER_FRAME_TRANSFORMER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>

namespace as2_camera_overlay {

// Adapts an existing tf2_ros::Buffer as a rviz_common::transformation::FrameTransformer.
// This avoids spawning a second TransformListener on the same node.
class Tf2BufferFrameTransformer : public rviz_common::transformation::FrameTransformer
{
public:
  explicit Tf2BufferFrameTransformer(std::shared_ptr<tf2_ros::Buffer> buffer);

  // FrameTransformer — lifecycle
  void initialize(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr,
    rclcpp::Clock::SharedPtr) override {}
  void clear() override {}
  std::vector<std::string> getAllFrameNames() const override;
  geometry_msgs::msg::PoseStamped transform(
    const geometry_msgs::msg::PoseStamped & pose_in,
    const std::string & target_frame) override;
  bool frameHasProblems(const std::string & frame, std::string & error) const override;
  rviz_common::transformation::TransformationLibraryConnector::WeakPtr getConnector() override;

  // tf2::BufferCoreInterface
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time) const override;
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame,
    const tf2::TimePoint & target_time,
    const std::string & source_frame,
    const tf2::TimePoint & source_time,
    const std::string & fixed_frame) const override;
  bool canTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    std::string * error_msg) const override;
  bool canTransform(
    const std::string & target_frame,
    const tf2::TimePoint & target_time,
    const std::string & source_frame,
    const tf2::TimePoint & source_time,
    const std::string & fixed_frame,
    std::string * error_msg) const override;

  // tf2_ros::AsyncBufferInterface
  tf2_ros::TransformStampedFuture waitForTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration & timeout,
    tf2_ros::TransformReadyCallback callback) override;
  void cancel(const tf2_ros::TransformStampedFuture & ts_future) override;

private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<rviz_common::transformation::TransformationLibraryConnector> connector_;
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__TF2_BUFFER_FRAME_TRANSFORMER_HPP_
