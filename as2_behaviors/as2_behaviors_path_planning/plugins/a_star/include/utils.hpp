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

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace utils
{

/* Point to occupancy grid cell */
// TODO(pariaspe): change from vector to pair or array
inline std::vector<int>
pointToCell(
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

/* OccupancyGrid cell to point in map frame */
inline geometry_msgs::msg::PointStamped
cellToPoint(
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

// convert cell coordinates to pixel coordinates
inline cv::Point2i cellToPixel(
  int cell_x, int cell_y,
  nav_msgs::msg::MapMetaData map_info)
{
  int pixel_x = map_info.height - cell_x - 1;
  int pixel_y = map_info.width - cell_y - 1;
  return cv::Point2i(pixel_x, pixel_y);
}

// convert pixel coordinates to cell coordinates
inline std::vector<int> pixelToCell(
  cv::Point2i pixel,
  nav_msgs::msg::MapMetaData map_info)
{
  int cell_x = map_info.width - pixel.x;
  int cell_y = map_info.height - pixel.y;
  return {cell_x, cell_y};
}

inline cv::Point2i pointToPixel(
  geometry_msgs::msg::PointStamped point,
  nav_msgs::msg::MapMetaData map_info,
  std::string target_frame_id,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  std::vector<int> cell =
    pointToCell(point, map_info, target_frame_id, tf_buffer);
  return cellToPixel(cell[0], cell[1], map_info);
}

inline cv::Point2i pointToPixel(
  geometry_msgs::msg::PoseStamped pose,
  nav_msgs::msg::MapMetaData map_info,
  std::string target_frame_id,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  geometry_msgs::msg::PointStamped point;
  point.header = pose.header;
  point.point = pose.pose.position;
  return utils::pointToPixel(point, map_info, target_frame_id, tf_buffer);
}

inline geometry_msgs::msg::PointStamped
pixelToPoint(
  cv::Point2i pixel, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  std::vector<int> cell = pixelToCell(pixel, map_info);
  return cellToPoint(cell[0], cell[1], map_info, map_header);
}

inline geometry_msgs::msg::PointStamped
pixelToPoint(
  int px_x, int px_y, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  cv::Point2i pixel = cv::Point2i(px_x, px_y);
  return pixelToPoint(pixel, map_info, map_header);
}

/*
 * Occupancy grid to binary image
 *
 * @param occ_grid: occupancy grid
 * @param thresh: threshold value
 * @return: binary image
 */
inline cv::Mat gridToImg(
  nav_msgs::msg::OccupancyGrid occ_grid,
  double thresh = 30, bool unknown_as_free = false)
{
  // TODO(pariaspe): explore method
  // cv::convertScaleAbs(labels, label1);

  cv::Mat mat =
    cv::Mat(occ_grid.data, CV_8UC1).reshape(1, occ_grid.info.height);

  // Grid frame to image frame
  cv::transpose(mat, mat);
  cv::flip(mat, mat, 0);
  cv::flip(mat, mat, 1);
  // Converto to unsigned 8bit matrix
  cv::Mat mat_unsigned = cv::Mat(mat.rows, mat.cols, CV_8UC1);

  int value = unknown_as_free ? 0 : 128;
  mat.setTo(value, mat == -1).convertTo(mat_unsigned, CV_8UC1);
  // Thresholding to get binary image
  cv::threshold(mat_unsigned, mat_unsigned, thresh, 255, cv::THRESH_BINARY_INV);
  return mat_unsigned;
}

/*
 * Binary image to occupancy grid
 *
 * @param img: binary image
 * @param header: header of the occupancy grid
 * @param grid_resolution: resolution of the occupancy grid
 * @return: occupancy grid
 */
inline nav_msgs::msg::OccupancyGrid
imgToGrid(
  const cv::Mat img, const std_msgs::msg::Header & header,
  double grid_resolution)
{
  cv::Mat mat = img.clone();

  // Grid header
  nav_msgs::msg::OccupancyGrid occ_grid;
  occ_grid.header = header;
  occ_grid.info.width = mat.cols;
  occ_grid.info.height = mat.rows;
  occ_grid.info.resolution = grid_resolution;
  occ_grid.info.origin.position.x =
    -mat.cols / 2 * grid_resolution;   // only valid if frame is earth?
  occ_grid.info.origin.position.y = -mat.rows / 2 * grid_resolution;

  // Image frame to grid frame
  cv::flip(mat, mat, 1);
  cv::flip(mat, mat, 0);
  cv::transpose(mat, mat);

  mat.setTo(30, mat == 255);
  mat.setTo(100, mat == 0);
  mat.setTo(0, mat == 30);
  occ_grid.data.assign(mat.data, mat.data + mat.total());
  return occ_grid;
}

}  // namespace utils

#endif  // UTILS_HPP_
