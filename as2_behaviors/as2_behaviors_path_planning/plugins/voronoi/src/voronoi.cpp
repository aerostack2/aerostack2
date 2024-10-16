// Copyright 2024 Universidad Politécnica de Madrid
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

/*!******************************************************************************
 *  \file       voronoi.cpp
 *  \brief      voronoi implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include <voronoi.hpp>

namespace voronoi
{
void Plugin::initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;
  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing Voronoi plugin");
}

void Plugin::occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  last_occ_grid_ = *(msg);
}

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped drone_pose, as2_msgs::action::NavigateToPoint::Goal goal)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Activating Voronoi plugin");
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Drone pose: [%f, %f] (%s)", drone_pose.pose.position.x,
    drone_pose.pose.position.y, drone_pose.header.frame_id.c_str());
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Going to [%f, %f] (%s)", goal.point.point.x,
    goal.point.point.y, goal.point.header.frame_id.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "Target frame (%s)", last_occ_grid_.header.frame_id.c_str());
  return true;
}

bool Plugin::on_deactivate()
{
  return true;
}

bool Plugin::on_modify()
{
  return true;
}

bool Plugin::on_pause()
{
  return true;
}

bool Plugin::on_resume()
{
  return true;
}

void Plugin::on_execution_end()
{
}

as2_behavior::ExecutionStatus Plugin::on_run()
{
  return as2_behavior::ExecutionStatus::SUCCESS;
}

// visualization_msgs::msg::Marker Plugin::get_path_marker(
//   std::string frame_id, rclcpp::Time stamp,
//   std::vector<cv::Point> path, nav_msgs::msg::MapMetaData map_info,
//   std_msgs::msg::Header map_header)
// {
//   visualization_msgs::msg::Marker marker;
//   marker.header.frame_id = frame_id;
//   marker.header.stamp = stamp;
//   marker.ns = "a_star";
//   marker.id = 33;
//   marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//   marker.action = visualization_msgs::msg::Marker::ADD;
//   marker.scale.x = 0.1;
//   marker.lifetime = rclcpp::Duration::from_seconds(0);  // Lifetime forever

//   for (auto & p : path) {
//     auto point = utils::pixelToPoint(p, map_info, map_header);
//     marker.points.push_back(point.point);
//     std_msgs::msg::ColorRGBA color;
//     color.a = 1.0;
//     color.r = 0.0;
//     color.g = 0.0;
//     color.b = 1.0;
//     marker.colors.push_back(color);
//   }
//   return marker;
// }

}  // namespace voronoi

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voronoi::Plugin, as2_behaviors_path_planning::PluginBase)
