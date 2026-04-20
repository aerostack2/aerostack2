#include "headless_ros_node.hpp"

namespace as2_camera_overlay {

HeadlessRosNode::HeadlessRosNode(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{}

std::string HeadlessRosNode::get_node_name() const
{
  return node_->get_name();
}

std::map<std::string, std::vector<std::string>>
HeadlessRosNode::get_topic_names_and_types() const
{
  return node_->get_topic_names_and_types();
}

rclcpp::Node::SharedPtr HeadlessRosNode::get_raw_node()
{
  return node_;
}

}  // namespace as2_camera_overlay
