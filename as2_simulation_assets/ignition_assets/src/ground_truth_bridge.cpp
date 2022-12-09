#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <as2_core/names/topics.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ros_gz_bridge/convert.hpp>

class GroundTruthBridge : public rclcpp::Node {
public:
  GroundTruthBridge() : Node("ground_truth_bridge") {
    this->declare_parameter<std::string>("name_space");
    this->get_parameter("name_space", model_name_);

    this->declare_parameter<std::string>("pose_frame_id");
    this->get_parameter("pose_frame_id", pose_frame_id_);

    this->declare_parameter<std::string>("twist_frame_id");
    this->get_parameter("twist_frame_id", twist_frame_id_);

    // Initialize the ignition node
    ign_node_ptr_                  = std::make_shared<ignition::transport::Node>();
    std::string ground_truth_topic = "/model/" + model_name_ + "/odometry";
    ign_node_ptr_->Subscribe(ground_truth_topic, this->ignitionGroundTruthCallback);

    ps_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos);
    ts_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos);
  }

private:
  std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
  std::string model_name_;
  static std::string pose_frame_id_;
  static std::string twist_frame_id_;
  static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ps_pub_;
  static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ts_pub_;

private:
  static std::string replace_delimiter(const std::string &input,
                                       const std::string &old_delim,
                                       const std::string new_delim) {
    std::string output;
    output.reserve(input.size());

    std::size_t last_pos = 0;

    while (last_pos < input.size()) {
      std::size_t pos = input.find(old_delim, last_pos);
      output += input.substr(last_pos, pos - last_pos);
      if (pos != std::string::npos) {
        output += new_delim;
        pos += old_delim.size();
      }
      last_pos = pos;
    }
    return output;
  }

  static void ignitionGroundTruthCallback(const ignition::msgs::Odometry &ign_msg,
                                          const ignition::transport::MessageInfo &msg_info) {
    geometry_msgs::msg::PoseStamped ps_msg;
    geometry_msgs::msg::TwistStamped ts_msg;

    ros_gz_bridge::convert_gz_to_ros(ign_msg.header(), ps_msg.header);
    ps_msg.header.frame_id    = pose_frame_id_;
    ps_msg.pose.position.x    = ign_msg.pose().position().x();
    ps_msg.pose.position.y    = ign_msg.pose().position().y();
    ps_msg.pose.position.z    = ign_msg.pose().position().z();
    ps_msg.pose.orientation.w = ign_msg.pose().orientation().w();
    ps_msg.pose.orientation.x = ign_msg.pose().orientation().x();
    ps_msg.pose.orientation.y = ign_msg.pose().orientation().y();
    ps_msg.pose.orientation.z = ign_msg.pose().orientation().z();
    ros_gz_bridge::convert_gz_to_ros(ign_msg.header(), ts_msg.header);
    ts_msg.header.frame_id = twist_frame_id_;
    ts_msg.twist.linear.x  = ign_msg.twist().linear().x();
    ts_msg.twist.linear.y  = ign_msg.twist().linear().y();
    ts_msg.twist.linear.z  = ign_msg.twist().linear().z();
    ts_msg.twist.angular.x = ign_msg.twist().angular().x();
    ts_msg.twist.angular.y = ign_msg.twist().angular().y();
    ts_msg.twist.angular.z = ign_msg.twist().angular().z();

    ps_pub_->publish(ps_msg);
    ts_pub_->publish(ts_msg);
  };
};

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr GroundTruthBridge::ps_pub_  = nullptr;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr GroundTruthBridge::ts_pub_ = nullptr;
std::string GroundTruthBridge::pose_frame_id_                                             = "";
std::string GroundTruthBridge::twist_frame_id_                                            = "";

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthBridge>());
  rclcpp::shutdown();
  return 0;
}
