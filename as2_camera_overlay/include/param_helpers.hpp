#ifndef AS2_CAMERA_OVERLAY__PARAM_HELPERS_HPP_
#define AS2_CAMERA_OVERLAY__PARAM_HELPERS_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace as2_camera_overlay
{

template<typename T>
T getOrDeclare(rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_parameter_value().get<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

inline std::string getOrDeclareStr(
  rclcpp::Node * node, const std::string & name, const char * default_value)
{
  return getOrDeclare<std::string>(node, name, std::string(default_value));
}

}  // namespace as2_camera_overlay

#endif  // AS2_CAMERA_OVERLAY__PARAM_HELPERS_HPP_
