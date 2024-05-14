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
 *  \file       mapping_2d.cpp
 *  \brief      2d mapping plugin.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include "mapping_2d.hpp"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

void mapping_2d::Plugin::on_setup()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "2D Mapping plugin setup");

  node_ptr_->declare_parameter("scan_range_max", 0.0);
  scan_range_max_ = node_ptr_->get_parameter("scan_range_max").as_double();
  node_ptr_->declare_parameter("map_resolution", 0.0);
  map_resolution_ = node_ptr_->get_parameter("map_resolution").as_double();
  // TODO(parias): Check if map_width and map_height units, meters or cell?
  node_ptr_->declare_parameter("map_width", 0);
  map_width_ = node_ptr_->get_parameter("map_width").as_int();
  node_ptr_->declare_parameter("map_height", 0);
  map_height_ = node_ptr_->get_parameter("map_height").as_int();

  RCLCPP_INFO(
    node_ptr_->get_logger(), "Parameters: scan_range_max: %f, map_resolution: %f, map_width: %d, "
    "map_height: %d",
    scan_range_max_, map_resolution_, map_width_, map_height_);

  laser_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
    "sensor_measurements/lidar/scan",
    as2_names::topics::sensor_measurements::qos,
    std::bind(
      &mapping_2d::Plugin::on_laser_scan, this,
      std::placeholders::_1));

  map_pub_ = node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  map_filtered_pub_ = node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  occ_grid_->header.stamp = node_ptr_->now();
  occ_grid_->header.frame_id = "earth";
  occ_grid_->info.resolution = map_resolution_;  // [m/cell]
  occ_grid_->info.width = map_width_;  // [cell]
  occ_grid_->info.height = map_height_;  // [cell]
  // Earth in the center of the map
  occ_grid_->info.origin.position.x = -map_width_ / 2 * map_resolution_;  // [m]
  occ_grid_->info.origin.position.y = -map_height_ / 2 * map_resolution_;  // [m]
  occ_grid_->data.assign(map_width_ * map_height_, -1);  // unknown
}

void mapping_2d::Plugin::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_msg =
    std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occupancy_grid_msg->header = occ_grid_->header;
  occupancy_grid_msg->header.stamp = msg->header.stamp;
  occupancy_grid_msg->info = occ_grid_->info;
  occupancy_grid_msg->data.assign(
    occupancy_grid_msg->info.width * occupancy_grid_msg->info.height, -1);  // unknown

  geometry_msgs::msg::PointStamped drone_pose;
  drone_pose.header = msg->header;
  drone_pose.point.x = 0;
  drone_pose.point.y = 0;
  std::vector<int> drone_cell;
  try {
    drone_cell = point_to_cell(
      drone_pose, occupancy_grid_msg->info,
      occupancy_grid_msg->header.frame_id, tf_buffer_);
  } catch (const tf2::ExtrapolationException & e) {
    std::cerr << e.what() << '\n';
    return;
  }

  for (int i = 0; i < msg->ranges.size(); i++) {
    float angle = msg->angle_min + i * msg->angle_increment;

    if (msg->ranges[i] < msg->range_min) {
      continue;
    }

    // No hit, change to max_range
    if (std::isinf(msg->ranges[i])) {
      msg->ranges[i] = msg->range_max;
    }

    // Clip range to parameter to clean noise
    if (msg->ranges[i] > scan_range_max_) {
      msg->ranges[i] = scan_range_max_;
    }

    geometry_msgs::msg::PointStamped in;
    in.header = msg->header;
    in.point.x = msg->ranges[i] * std::cos(angle);
    in.point.y = msg->ranges[i] * std::sin(angle);

    std::vector<int> cell;
    try {
      cell = point_to_cell(
        in, occupancy_grid_msg->info, occupancy_grid_msg->header.frame_id,
        tf_buffer_);
    } catch (const tf2::ExtrapolationException & e) {
      std::cerr << e.what() << '\n';
      continue;
    }

    // Points between drone and laser hit are free
    std::vector<std::vector<int>> middle_cells = get_middle_points(drone_cell, cell);
    for (const std::vector<int> & p : middle_cells) {
      int cell_index = p[1] * occupancy_grid_msg->info.width + p[0];
      if (is_cell_index_valid(p)) {
        occupancy_grid_msg->data[cell_index] = 0;  // free
      }
    }

    // Update cell of the laser hit/miss
    int cell_index = cell[1] * occupancy_grid_msg->info.width + cell[0];
    if (is_cell_index_valid(cell)) {
      occupancy_grid_msg->data[cell_index] =
        (msg->ranges[i] < std::min(msg->range_max, static_cast<float>(scan_range_max_))) ? 100 : 0;
    }
  }

  publish_map(*occupancy_grid_msg);
}

void mapping_2d::Plugin::publish_map(const nav_msgs::msg::OccupancyGrid & map_update)
{
  occ_grid_->header = map_update.header;
  occ_grid_->info = map_update.info;
  occ_grid_->data =
    cv::Mat(add_occ_grid_update(map_update.data, occ_grid_->data));
  map_pub_->publish(*occ_grid_);

  nav_msgs::msg::OccupancyGrid occ_grid_filtered = filter_occ_grid(*occ_grid_);
  map_filtered_pub_->publish(occ_grid_filtered);
}

// AUX METHODS
std::vector<std::vector<int>> mapping_2d::Plugin::get_middle_points(
  std::vector<int> p1,
  std::vector<int> p2)
{
  std::vector<std::vector<int>> middle_points;
  int dx = p2[0] - p1[0];
  int dy = p2[1] - p1[1];
  int steps = std::max(std::abs(dx), std::abs(dy));
  float xinc = dx / static_cast<float>(steps);
  float yinc = dy / static_cast<float>(steps);

  float x = p1[0];
  float y = p1[1];

  for (int i = 0; i < steps - 1; ++i) {
    x += xinc;
    y += yinc;
    middle_points.push_back({static_cast<int>(std::round(x)), static_cast<int>(std::round(y))});
  }

  return middle_points;
}

bool mapping_2d::Plugin::is_cell_index_valid(std::vector<int> cell)
{
  return cell[0] >= 0 && cell[0] < map_width_ && cell[1] >= 0 &&
         cell[1] < map_height_;
}

std::vector<int8_t> mapping_2d::Plugin::add_occ_grid_update(
  const std::vector<int8_t> & update, const std::vector<int8_t> & occ_grid_data)
{
  // TODO(parias): Parametrize weights for hit and miss. Also, threshold for keeping obstacles

  // Values at occ_grid update are: 0 (free), 100 (occupied) or -1 (unknown)
  cv::Mat aux = cv::Mat(update).clone();
  aux.setTo(-10, aux == 0);  // free with weight -> 10
  aux.setTo(40, aux == 100);  // occupied with weight -> 40
  aux.setTo(0, aux == -1);   // unknown with weight -> 0

  aux += cv::Mat(occ_grid_data);
  aux.setTo(-1, aux == -1);
  aux.setTo(0, aux < -1);
  aux.setTo(100, aux > 100);

  // Keeping obstacles
  aux.setTo(100, cv::Mat(occ_grid_data) > 80);
  return aux;
}

nav_msgs::msg::OccupancyGrid mapping_2d::Plugin::filter_occ_grid(
  const nav_msgs::msg::OccupancyGrid & occ_grid)
{
  // TODO(parias): Implement a filter to remove noise from the map
  return occ_grid;
}

std::vector<int>
mapping_2d::Plugin::point_to_cell(
  geometry_msgs::msg::PointStamped point,
  nav_msgs::msg::MapMetaData map_info, std::string target_frame_id,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  geometry_msgs::msg::PointStamped out;
  geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(
    target_frame_id, point.header.frame_id, point.header.stamp,
    rclcpp::Duration::from_seconds(0.5));
  tf2::doTransform(point, out, transform);

  std::vector<int> cell;
  cell.push_back(
    static_cast<int>((out.point.x - map_info.origin.position.x) /
    map_info.resolution));
  cell.push_back(
    static_cast<int>((out.point.y - map_info.origin.position.y) /
    map_info.resolution));
  cell.push_back(static_cast<int>(std::round(out.point.z * 100)));
  return cell;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mapping_2d::Plugin, as2_map_server_plugin_base::MapServerBase)
