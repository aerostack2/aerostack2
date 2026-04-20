#ifndef AS2_CAMERA_OVERLAY__HEADLESS_ROS_NODE_HPP_
#define AS2_CAMERA_OVERLAY__HEADLESS_ROS_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace as2_camera_overlay {

// Wraps an existing rclcpp::Node as an rviz RosNodeAbstractionIface so that
// rviz displays subscribe on our node rather than spawning a second one.
class HeadlessRosNode : public rviz_common::ros_integration::RosNodeAbstractionIface
{
public:
  explicit HeadlessRosNode(rclcpp::Node::SharedPtr node);

  std::string get_node_name() const override;

  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const override;

  rclcpp::Node::SharedPtr get_raw_node() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__HEADLESS_ROS_NODE_HPP_
