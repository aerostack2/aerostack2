#ifndef __STATE_ESTIMATOR_PLUGIN_ODOM_ONLY_HPP__
#define __STATE_ESTIMATOR_PLUGIN_ODOM_ONLY_HPP__

#include <as2_state_estimator_plugin_base/plugin_base.hpp>
namespace as2_state_estimator_plugin_external_odom {

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

public:
  Plugin() : as2_state_estimator_plugin_base::StateEstimatorBase(){};
  void on_setup() override {
    std::string odom_topic = as2_names::topics::sensor_measurements::odom;
    node_ptr_->get_parameter("odom_topic", odom_topic);
    odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, as2_names::topics::sensor_measurements::qos,
        std::bind(&Plugin::odom_callback, this, std::placeholders::_1));

    // publish static transform from earth to map and map to odom
    // TODO: MODIFY this to a initial earth to map transform (reading initial position from
    // parameters or msgs )
    geometry_msgs::msg::TransformStamped earth_to_map =
        as2::tf::getTransformation(get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
    geometry_msgs::msg::TransformStamped map_to_odom =
        as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);

    // TODO: CHECK IF WE NEED TO PUBLISH THIS PERIODICALLY
    publish_static_transform(earth_to_map);
    publish_static_transform(map_to_odom);
  };

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // odom should have frame_id = odom and child_frame_id = base_link
    // since we only have this message for generating the tf tree we will publish the transform
    // from odom to base_link directly and the transform from earth to map and map to odom  will
    // be the identity transform

    if (msg->header.frame_id != get_odom_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Received odom in frame %s, expected %s",
                   msg->header.frame_id.c_str(), get_odom_frame().c_str());
      return;
    }
    if (msg->child_frame_id != get_base_frame()) {
      RCLCPP_ERROR(node_ptr_->get_logger(),
                   "Received odom child_frame_id  in frame %s, expected %s",
                   msg->child_frame_id.c_str(), get_base_frame().c_str());
      return;
    }

    auto transform                    = geometry_msgs::msg::TransformStamped();
    transform.header                  = msg->header;
    transform.child_frame_id          = msg->child_frame_id;
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation      = msg->pose.pose.orientation;

    publish_transform(transform);

    // publish pose as "earth to base_link"
    auto pose            = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = get_earth_frame();
    pose.header.stamp    = msg->header.stamp;
    pose.pose            = msg->pose.pose;
    publish_pose(pose);

    // publish twist in "base_link" frame
    auto twist            = geometry_msgs::msg::TwistStamped();
    twist.header.frame_id = get_base_frame();
    twist.header.stamp    = msg->header.stamp;
    twist.twist           = msg->twist.twist;
    publish_twist(twist);
  };
};

}  // namespace as2_state_estimator_plugin_external_odom

#endif  // __STATE_ESTIMATOR_PLUGIN_ODOM_ONLY_HPP__
