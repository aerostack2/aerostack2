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
 *  \file       utils.hpp
 *  \brief      utils implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cell_node.hpp"

namespace utils
{

/**
 * @brief Convert a point to a cell in the occupancy grid
 * @param point point to convert
 * @param map_info occupancy grid metadata
 * @param target_frame_id target frame id
 * @param tf_buffer tf buffer
 */
Point2i pointToCell(
  geometry_msgs::msg::PointStamped point,
  nav_msgs::msg::MapMetaData map_info, std::string target_frame_id,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  geometry_msgs::msg::PointStamped out;
  geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(
    target_frame_id, point.header.frame_id, point.header.stamp,
    rclcpp::Duration::from_seconds(0.5));
  tf2::doTransform(point, out, transform);

  Point2i cell;
  cell.x = static_cast<int>((out.point.x - map_info.origin.position.x) / map_info.resolution);
  cell.y = static_cast<int>((out.point.y - map_info.origin.position.y) / map_info.resolution);
  return cell;
}

/**
 * @brief Convert a pose to a cell in the occupancy grid
 * @param pose pose to convert
 * @param map_info occupancy grid metadata
 * @param target_frame_id target frame id
 * @param tf_buffer tf buffer
 */
Point2i poseToCell(
  geometry_msgs::msg::PoseStamped pose,
  nav_msgs::msg::MapMetaData map_info, std::string target_frame_id,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  geometry_msgs::msg::PointStamped point;
  point.header = pose.header;
  point.point = pose.pose.position;
  return pointToCell(point, map_info, target_frame_id, tf_buffer);
}

/**
 * @brief Convert a cell to a point in the map frame
 * @param cell_x cell x
 * @param cell_y cell y
 * @param map_info occupancy grid metadata
 * @param map_header occupancy grid header
 */
geometry_msgs::msg::PointStamped cellToPoint(
  int cell_x, int cell_y, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  geometry_msgs::msg::PointStamped point;
  point.header = map_header;
  point.point.x = cell_x * map_info.resolution + map_info.origin.position.x -
    map_info.resolution / 2;               // middle of cell
  point.point.y = cell_y * map_info.resolution + map_info.origin.position.y -
    map_info.resolution / 2;               // middle of cell
  return point;
}

/**
 * @brief Convert a cell to a point in the map frame
 * @param cell cell to convert
 * @param map_info occupancy grid metadata
 * @param map_header occupancy grid header
 */
geometry_msgs::msg::PointStamped cellToPoint(
  Point2i cell, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  return cellToPoint(cell.x, cell.y, map_info, map_header);
}

/**
 * @brief Calculate perpendicular distance from a point to a line segment
 * @param point The point to measure distance from
 * @param line_start Start of the line segment
 * @param line_end End of the line segment
 * @return Perpendicular distance
 */
double perpendicular_distance(
  const Point2i & point, const Point2i & line_start,
  const Point2i & line_end)
{
  if (line_start == line_end) {
    double dx = point.x - line_start.x;
    double dy = point.y - line_start.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // Calculate perpendicular distance using cross product formula
  double dx = line_end.x - line_start.x;
  double dy = line_end.y - line_start.y;

  double numerator = std::abs(
    dy * point.x - dx * point.y +
    line_end.x * line_start.y -
    line_end.y * line_start.x);
  double denominator = std::sqrt(dx * dx + dy * dy);

  return numerator / denominator;
}

/**
 * @brief Simplify path using Ramer-Douglas-Peucker algorithm
 * @param path Vector of Point2i coordinates representing the path
 * @param epsilon Maximum distance threshold for point removal (larger = more aggressive simplification)
 * @return Simplified vector of Point2i coordinates
 */
std::vector<Point2i> simplify_path_rdp(const std::vector<Point2i> & path, double epsilon)
{
  if (path.size() < 3) {
    return path;
  }

  // Find point with maximum distance from line segment
  double dmax = 0.0;
  size_t index = 0;
  size_t end = path.size() - 1;

  for (size_t i = 1; i < end; ++i) {
    double d = perpendicular_distance(path[i], path[0], path[end]);
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }

  std::vector<Point2i> result;

  // If max distance is greater than epsilon, recursively simplify
  if (dmax > epsilon) {
    // Recursive calls
    std::vector<Point2i> path1(path.begin(), path.begin() + index + 1);
    std::vector<Point2i> path2(path.begin() + index, path.end());

    std::vector<Point2i> rec_results1 = simplify_path_rdp(path1, epsilon);
    std::vector<Point2i> rec_results2 = simplify_path_rdp(path2, epsilon);

    // Build result list (avoid duplicating the middle point)
    result.insert(result.end(), rec_results1.begin(), rec_results1.end() - 1);
    result.insert(result.end(), rec_results2.begin(), rec_results2.end());
  } else {
    // Just keep start and end points
    result.push_back(path[0]);
    result.push_back(path[end]);
  }

  return result;
}

}  // namespace utils

#endif  // UTILS_HPP_
