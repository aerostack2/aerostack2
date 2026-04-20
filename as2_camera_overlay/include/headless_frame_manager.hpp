#ifndef AS2_CAMERA_OVERLAY__HEADLESS_FRAME_MANAGER_HPP_
#define AS2_CAMERA_OVERLAY__HEADLESS_FRAME_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/transformation/tf_frame_transformer.hpp>

namespace as2_camera_overlay {

// Thin FrameManagerIface implementation backed by the overlay node's tf2_ros::Buffer.
class HeadlessFrameManager : public rviz_common::FrameManagerIface
{
  Q_OBJECT

public:
  HeadlessFrameManager(
    std::shared_ptr<tf2_ros::Buffer> buffer,
    rclcpp::Clock::SharedPtr clock,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node);

  void setFixedFrame(const std::string & frame) override;
  void setPause(bool) override {}
  bool getPause() override { return false; }
  void setSyncMode(SyncMode) override {}
  SyncMode getSyncMode() override { return SyncOff; }
  void syncTime(rclcpp::Time) override {}
  rclcpp::Time getTime() override;

  bool getTransform(
    const std::string & frame,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) override;

  bool getTransform(
    const std::string & frame,
    rclcpp::Time time,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) override;

  bool transform(
    const std::string & frame,
    rclcpp::Time time,
    const geometry_msgs::msg::Pose & pose,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) override;

  void update() override {}

  bool frameHasProblems(const std::string & frame, std::string & error) override;
  bool transformHasProblems(const std::string & frame, std::string & error) override;
  bool transformHasProblems(
    const std::string & frame, rclcpp::Time time, std::string & error) override;

  const std::string & getFixedFrame() override;

  rviz_common::transformation::TransformationLibraryConnector::WeakPtr
  getConnector() override;

  std::shared_ptr<rviz_common::transformation::FrameTransformer>
  getTransformer() override;

  std::vector<std::string> getAllFrameNames() override;

public Q_SLOTS:
  void setTransformerPlugin(
    std::shared_ptr<rviz_common::transformation::FrameTransformer>) override {}

private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  rclcpp::Clock::SharedPtr clock_;
  std::string fixed_frame_{"map"};
  std::shared_ptr<rviz_default_plugins::transformation::TFFrameTransformer> transformer_;
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__HEADLESS_FRAME_MANAGER_HPP_
