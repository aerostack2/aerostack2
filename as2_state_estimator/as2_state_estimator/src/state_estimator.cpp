#include <as2_state_estimator/state_estimator.hpp>

StateEstimator::StateEstimator()
    : as2::Node("state_estimator",
                rclcpp::NodeOptions()
                    .allow_undeclared_parameters(true)
                    .automatically_declare_parameters_from_overrides(true)) {
  tf_buffer_            = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_broadcaster_       = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  try {
    this->get_parameter("plugin_name", plugin_name_);
  } catch (const rclcpp::ParameterTypeException& e) {
    RCLCPP_FATAL(this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
                 e.what());
    this->~StateEstimator();
  }
  plugin_name_ += "::Plugin";
  loader_ =
      std::make_shared<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>(
          "as2_state_estimator_plugin_base", "as2_state_estimator_plugin_base::StateEstimatorBase");
  try {
    plugin_ptr_ = loader_->createSharedInstance(plugin_name_);
    plugin_ptr_->setup(this, tf_buffer_, tf_broadcaster_, tfstatic_broadcaster_);
  } catch (const pluginlib::PluginlibException& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
    this->~StateEstimator();
  }
}

// StateEstimator::~StateEstimator() {
//   tf_buffer_.reset();
//   tf_broadcaster_.reset();
//   tfstatic_broadcaster_.reset();
//   // plugin_ptr_.reset();
//   RCLCPP_WARN(this->get_logger(), "Shutting down state estimator");
//   // plugin_ptr_.reset();
//   // loader_->unloadLibraryForClass(plugin_name_);
//   RCLCPP_INFO(this->get_logger(), "SHUTDOWN COMPLETE");
// }
