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
#include <utils.hpp>

#define FREE_SPACE 0
#define OCC_SPACE 100
#define UNKNOWN_SPACE -1
#define MAX_DIST 100.0f


namespace voronoi
{
void Plugin::initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;

  graph_searcher_ = VoronoiSearcher();

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing Voronoi plugin");

  // node_ptr_->declare_parameter("enable_visualization", true);
  enable_visualization_ = node_ptr_->get_parameter("enable_visualization").as_bool();
  enable_visualization_ = true;  // TODO(pariaspe): not publish when false

  occ_grid_sub_ = node_ptr_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map_filtered", 1, std::bind(&Plugin::occ_grid_cbk, this, std::placeholders::_1));

  if (enable_visualization_) {
    viz_pub_ =
      node_ptr_->create_publisher<visualization_msgs::msg::Marker>("plugin_viz/marker", 10);
    viz_voronoi_grid_pub_ =
      node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("plugin_viz/voronoi", 10);
    viz_dist_field_grid_pub_ = node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "plugin_viz/dist_field", 10);
  }
}

void Plugin::occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  last_occ_grid_ = *(msg);

  Plugin::update_costs(last_occ_grid_);

  Plugin::viz_voronoi_grid();
  Plugin::viz_dist_field_grid();
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

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Target frame (%s)", last_dist_field_grid_.header.frame_id.c_str());

  Point2i goal_cell = utils::pointToCell(
    goal.point, last_dist_field_grid_.info, last_dist_field_grid_.header.frame_id, tf_buffer_);
  Point2i origin_cell = utils::poseToCell(
    drone_pose, last_dist_field_grid_.info, last_dist_field_grid_.header.frame_id, tf_buffer_);

  RCLCPP_INFO(node_ptr_->get_logger(), "Origin cell: [%d, %d]", origin_cell.x, origin_cell.y);
  RCLCPP_INFO(node_ptr_->get_logger(), "Goal cell: [%d, %d]", goal_cell.x, goal_cell.y);

  graph_searcher_.update_voronoi(dynamic_voronoi_);
  std::vector<Point2i> path = graph_searcher_.solve_graph(origin_cell, goal_cell);
  if (path.size() == 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Path to goal not found. Goal Rejected.");
    return false;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Path size: %ld", path.size());

  // Visualize path
  auto path_marker = get_path_marker(
    last_dist_field_grid_.header.frame_id, node_ptr_->get_clock()->now(), path,
    last_dist_field_grid_.info, last_dist_field_grid_.header);
  RCLCPP_INFO(node_ptr_->get_logger(), "Publishing path");
  viz_pub_->publish(path_marker);

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

bool Plugin::outline_map(nav_msgs::msg::OccupancyGrid & occ_grid, uint8_t value)
{
  int8_t * char_map = occ_grid.data.data();
  unsigned int size_x = occ_grid.info.width;
  unsigned int size_y = occ_grid.info.height;
  if (char_map == nullptr) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "char_map == nullptr");
    return false;
  }

  int8_t * pc = char_map;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  pc = char_map + (size_y - 1U) * size_x;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  pc = char_map;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  pc = char_map + size_x - 1U;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  return true;
}

void Plugin::update_dynamic_voronoi(nav_msgs::msg::OccupancyGrid & occ_grid)
{
  unsigned int size_x = occ_grid.info.width;
  unsigned int size_y = occ_grid.info.height;
  if (last_size_x_ != size_x || last_size_y_ != size_y) {
    dynamic_voronoi_.initializeEmpty(static_cast<int>(size_x), static_cast<int>(size_y));

    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells;
  std::vector<IntPoint> new_occupied_cells;
  for (int j = 0; j < static_cast<int>(size_y); ++j) {
    for (int i = 0; i < static_cast<int>(size_x); ++i) {
      int cell_index = j * occ_grid.info.width + i;
      if (dynamic_voronoi_.isOccupied(i, j) && occ_grid.data[cell_index] == FREE_SPACE) {
        new_free_cells.emplace_back(i, j);
      }

      if (!dynamic_voronoi_.isOccupied(i, j) && occ_grid.data[cell_index] == OCC_SPACE) {
        new_occupied_cells.emplace_back(i, j);
      }

      if (!dynamic_voronoi_.isOccupied(i, j) && occ_grid.data[cell_index] == UNKNOWN_SPACE) {
        new_occupied_cells.emplace_back(i, j);
      }
    }
  }

  for (const IntPoint & cell : new_free_cells) {
    dynamic_voronoi_.clearCell(cell.x, cell.y);
  }

  for (const IntPoint & cell : new_occupied_cells) {
    dynamic_voronoi_.occupyCell(cell.x, cell.y);
  }
}

void Plugin::update_costs(nav_msgs::msg::OccupancyGrid & occ_grid)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!outline_map(occ_grid, OCC_SPACE)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to outline map.");
    return;
  }

  update_dynamic_voronoi(occ_grid);

  // Start timing.
  const auto start_timestamp = std::chrono::system_clock::now();

  dynamic_voronoi_.update();
  dynamic_voronoi_.prune();

  // dynamic_voronoi_.visualize("voronoi.ppm");

  // End timing.
  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Runtime=%f ms.", diff.count() * 1e3);
}

void Plugin::viz_voronoi_grid()
{
  nav_msgs::msg::OccupancyGrid occ_grid;
  occ_grid.header.frame_id = last_occ_grid_.header.frame_id;
  occ_grid.header.stamp = node_ptr_->now();
  occ_grid.info = last_occ_grid_.info;
  occ_grid.data = last_occ_grid_.data;

  for (int j = 0; j < static_cast<int>(occ_grid.info.height); ++j) {
    for (int i = 0; i < static_cast<int>(occ_grid.info.width); ++i) {
      int cell_index = j * occ_grid.info.width + i;
      if (dynamic_voronoi_.isVoronoi(i, j)) {
        occ_grid.data[cell_index] = 0;
      } else {
        occ_grid.data[cell_index] = 100;
      }
    }
  }

  viz_voronoi_grid_pub_->publish(occ_grid);
}

void Plugin::viz_dist_field_grid()
{
  // nav_msgs::msg::OccupancyGrid occ_grid;
  last_dist_field_grid_ = last_occ_grid_;
  last_dist_field_grid_.header.frame_id = last_occ_grid_.header.frame_id;
  last_dist_field_grid_.header.stamp = node_ptr_->now();
  last_dist_field_grid_.info = last_occ_grid_.info;
  last_dist_field_grid_.data = last_occ_grid_.data;

  for (int j = 0; j < static_cast<int>(last_dist_field_grid_.info.height); ++j) {
    for (int i = 0; i < static_cast<int>(last_dist_field_grid_.info.width); ++i) {
      int cell_index = j * last_dist_field_grid_.info.width + i;
      if (dynamic_voronoi_.isOccupied(i, j)) {
        last_dist_field_grid_.data[cell_index] = -1;
        continue;
      }
      float dist = dynamic_voronoi_.getDistance(i, j);
      // dist = dist * dist;
      dist = MAX_DIST - std::min(dist, MAX_DIST);
      last_dist_field_grid_.data[cell_index] = static_cast<int8_t>(dist);
    }
  }

  viz_dist_field_grid_pub_->publish(last_dist_field_grid_);
}

visualization_msgs::msg::Marker Plugin::get_path_marker(
  std::string frame_id, rclcpp::Time stamp,
  std::vector<Point2i> path, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "a_star";
  marker.id = 33;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Lifetime forever

  for (auto & p : path) {
    auto point = utils::cellToPoint(p.x, p.y, map_info, map_header);
    marker.points.push_back(point.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    marker.colors.push_back(color);
  }
  return marker;
}

}  // namespace voronoi

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voronoi::Plugin, as2_behaviors_path_planning::PluginBase)
