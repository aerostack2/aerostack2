// Copyright 2025 Universidad Politécnica de Madrid
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
 *  \file       a_star_graph.cpp
 *  \brief      a_star_graph implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include <a_star_graph.hpp>
#include <utils.hpp>

namespace a_star_graph
{
void Plugin::initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;

  a_star_planner_ = AStarGraphPlanner();

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing A* plugin");

  // node_ptr_->declare_parameter("safety_distance", 0.5);
  safety_distance_ = node_ptr_->get_parameter("safety_distance").as_double();

  // node_ptr_->declare_parameter("enable_path_optimizer", false);
  use_path_optimizer_ = node_ptr_->get_parameter("enable_path_optimizer").as_bool();

  // node_ptr_->declare_parameter("enable_visualization", true);
  enable_visualization_ = node_ptr_->get_parameter("enable_visualization").as_bool();
  enable_visualization_ = true;  // TODO(pariaspe): not publish when false

  map_sub_ = node_ptr_->create_subscription<as2_msgs::msg::Graph>(
    "map", 1, std::bind(&Plugin::map_cbk, this, std::placeholders::_1));

  if (enable_visualization_) {
    viz_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::Marker>("viz/path_marker", 10);
    // viz_obstacle_grid_pub_ =
    //   node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("plugin_viz/obstacle_map", 10);
  }
}

void Plugin::map_cbk(const as2_msgs::msg::Graph::SharedPtr msg)
{
  last_map_update_ = *(msg);
}

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped drone_pose, as2_msgs::action::NavigateToPoint::Goal goal)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Activating A* plugin");
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Drone pose: [%f, %f, %f] (%s)", drone_pose.pose.position.x,
    drone_pose.pose.position.y, drone_pose.pose.position.z, drone_pose.header.frame_id.c_str());
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Going to [%f, %f, %f] (%s)", goal.point.point.x,
    goal.point.point.y, goal.point.point.z, goal.point.header.frame_id.c_str());

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Target frame (%s)", last_map_update_.header.frame_id.c_str());

  as2_msgs::msg::Node drone_node = node_closer_to_pose(last_map_update_, drone_pose);
  as2_msgs::msg::Node goal_node = node_closer_to_pose(last_map_update_, goal.point.point);
  Point2i drone_cell = Point2i(drone_node.uuid, drone_node.uuid);
  Point2i goal_cell = Point2i(goal_node.uuid, goal_node.uuid);

  a_star_planner_.update_graph_msg(last_map_update_);
  std::vector<Point2i> path = a_star_planner_.solve(drone_cell, goal_cell);
  if (path.size() == 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Path to goal not found. Goal Rejected.");
    return false;
  }

  if (use_path_optimizer_) {
    // TODO(pariaspe): Implement path optimizer
    RCLCPP_WARN(node_ptr_->get_logger(), "Path optimizer not implemented yet");
    // path = path_optimizer::solve(path);
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Path size: %ld", path.size());

  // Visualize path
  auto path_marker = get_path_marker("earth", node_ptr_->get_clock()->now(), path);
  RCLCPP_INFO(node_ptr_->get_logger(), "Publishing path");
  viz_pub_->publish(path_marker);

  // TODO(pariasp): split path generator from visualization, right now enable_viz is always true
  path_ = path_marker.points;
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

as2_msgs::msg::Node Plugin::node_closer_to_pose(
  const as2_msgs::msg::Graph & graph, const geometry_msgs::msg::PoseStamped & ps)
{
  return node_closer_to_pose(graph, ps.pose.position);
}

as2_msgs::msg::Node Plugin::node_closer_to_pose(
  const as2_msgs::msg::Graph & graph, const geometry_msgs::msg::Point & point)
{
  int min_dist = std::numeric_limits<int>::max();
  int min_idx = 0;
  for (const as2_msgs::msg::Node & node : graph.nodes) {
    int dist = std::sqrt(
      std::pow(node.position.x - point.x, 2) +
      std::pow(node.position.y - point.y, 2) +
      std::pow(node.position.z - point.z, 2));
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = node.uuid;
    }
  }
  return graph.nodes[min_idx];
}

visualization_msgs::msg::Marker Plugin::get_path_marker(
  std::string frame_id, rclcpp::Time stamp,
  std::vector<Point2i> path)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "a_star";
  marker.id = 33;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.15;
  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Lifetime forever

  for (auto & p : path) {
    geometry_msgs::msg::Point point;
    marker.points.emplace_back(last_map_update_.nodes[p.x].position);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    marker.colors.push_back(color);
  }
  return marker;
}

}  // namespace a_star_graph

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(a_star_graph::Plugin, as2_behaviors_path_planning::PluginBase)
