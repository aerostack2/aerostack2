#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <as2_core/names/topics.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <ros_gz_bridge/convert.hpp>

class GroundTruthBridge : public rclcpp::Node
{
public:
  GroundTruthBridge();

private:
  std::shared_ptr<gz::transport::Node> ign_node_ptr_;
  std::string model_name_;
  // static char pose_frame_id_[];
  // static char twist_frame_id_[];
  static std::shared_ptr<std::string> pose_frame_id_;
  static std::shared_ptr<std::string> twist_frame_id_;
  static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ps_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ts_pub_;

private:
  static std::string replace_delimiter(
    const std::string & input,
    const std::string & old_delim,
    const std::string new_delim);

  static void ignitionGroundTruthCallback(
    const gz::msgs::Odometry & ign_msg,
    const gz::transport::MessageInfo & msg_info);

};
