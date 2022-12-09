#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include "rclcpp/rclcpp.hpp"

#include <as2_msgs/msg/thrust.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Plugin_test : public rclcpp::Node {
public:
  Plugin_test() : Node("Plugin_test") {
    loader_ = std::make_shared<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>>(
        "controller_plugin_base", "controller_plugin_base::ControllerBase");
    try {
      std::filesystem::path plugin_name_ =
          "/home/rafa/aerostack2_ws/install/controller_plugin_differential_flatness/lib/"
          "libcontroller_plugin_differential_flatness.so";
      controller_ = loader_->createSharedInstance(plugin_name_);
      controller_->initialize(this);
      controller_->updateParams(this->list_parameters({}, 0).names);
      controller_handler_ = std::make_shared<ControllerHandler>(controller_, this);
      RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED [%s]", plugin_name_.c_str());
    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      return;
    }

    tf_handler_ = as2::tf::TfHandler(this);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
    thrust_pub_ = this->create_publisher<as2_msgs::msg::Thrust>(
        as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);

    ref_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        as2_names::topics::motion_reference::trajectory, as2_names::topics::motion_reference::qos,
        std::bind(&Plugin_test::ref_traj_callback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
        std::bind(&Plugin_test::state_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&Plugin_test::timer_callback, this));
  }

private:
  void ref_traj_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
    ref_traj_ = *msg;
    return;
  };

  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    geometry_msgs::msg::PoseStamped pose_msg;
    geometry_msgs::msg::TwistStamped twist_msg;
    geometry_msgs::msg::TwistStamped _twist_msg;
    try {
      // obtain transform from world to base_link
      pose_msg  = tf_handler_.getPoseStamped(odom_frame_id_, base_frame_id_,
                                             tf2_ros::fromMsg(_twist_msg.header.stamp));
      twist_msg = tf_handler_.convert(_twist_msg, odom_frame_id_);

    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(node_ptr_->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
    state_adquired_ = true;
    pose_           = pose_msg;
    twist_          = twist_msg;
    controller_ptr_->updateState(pose_, twist_);
  }

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  std::shared_ptr<pluginlib::ClassLoader<controller_plugin_base::ControllerBase>> loader_;
  std::shared_ptr<ControllerHandler> controller_handler_;
  as2::tf::TfHandler tf_handler_;

  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::TwistStamped twist_;
  geometry_msgs::msg::PoseStamped ref_pose_;
  geometry_msgs::msg::TwistStamped ref_twist_;
  trajectory_msgs::msg::JointTrajectoryPoint ref_traj_;

  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_traj_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}