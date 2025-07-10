// Copyright 2023 Universidad Politécnica de Madrid
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       filtered_transform_listener.cpp
 *  \brief      filtered_transform_listener implementation file.
 *  \authors    Tully Foote
 *              Miguel Fernandez Cortizas
 *              Rafael Perez Segui
 ********************************************************************************/


#include "as2_core/utils/filtered_transform_listener.hpp"
#include <memory>
#include <string>
#include <thread>


namespace as2
{

namespace tf
{


FilteredTransformListener::FilteredTransformListener(
  tf2::BufferCore & buffer, bool spin_thread,
  bool static_only)
: buffer_(buffer)
{
  rclcpp::NodeOptions options;
  // create a unique name for the node
  // but specify its name in .arguments to override any __node passed on the command line.
  // avoiding sstream because it's behavior can be overridden by external libraries.
  // See this issue: https://github.com/ros2/geometry2/issues/540
  char node_name[42];
  snprintf(
    node_name, sizeof(node_name), "transform_listener_impl_%zx",
    reinterpret_cast<size_t>(this)
  );
  options.arguments({"--ros-args", "-r", "__node:=" + std::string(node_name)});
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  optional_default_node_ = rclcpp::Node::make_shared("_", options);
  init(
    optional_default_node_->get_node_base_interface(),
    optional_default_node_->get_node_logging_interface(),
    optional_default_node_->get_node_parameters_interface(),
    optional_default_node_->get_node_topics_interface(),
    spin_thread, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
    tf2_ros::detail::get_default_transform_listener_sub_options(),
    tf2_ros::detail::get_default_transform_listener_static_sub_options(),
    static_only);
}

FilteredTransformListener::~FilteredTransformListener()
{
  if (spin_thread_) {
    executor_->cancel();
    dedicated_listener_thread_->join();
  }
}

void FilteredTransformListener::subscription_callback(
  const tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
  bool is_static)
{
  const tf2_msgs::msg::TFMessage & msg_in = *msg;
  // TODO(tfoote) find a way to get the authority
  std::string authority = "Authority undetectable";
  for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
    // check the filter_rules_
    // if any of the filter rules return false, skip this Transform
    bool skip = false;
    for (auto filter_rule : filter_rules_) {
      if (!filter_rule(msg_in.transforms[i])) {
        skip = true;
        break;
      }
    }
    if (skip) {
      continue;
    }

    try {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    } catch (const tf2::TransformException & ex) {
      // /\todo Use error reporting
      std::string temp = ex.what();
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "Failure to set received transform from %s to %s with error: %s\n",
        msg_in.transforms[i].child_frame_id.c_str(),
        msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
}


}  // namespace tf

}  // namespace as2
