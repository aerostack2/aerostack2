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
 *  \file       a_star_searcher.cpp
 *  \brief      a_star_searcher implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include "a_star_searcher.hpp"

nav_msgs::msg::OccupancyGrid AStarSearcher::update_grid(
  const nav_msgs::msg::OccupancyGrid & occ_grid, const Point2i & drone_pose,
  double safety_distance)
{
  cv::Mat mat = gridToImg(occ_grid);

  cv::Point2i origin = cellToPixel(drone_pose, occ_grid.info);

  int iterations = std::ceil(safety_distance / occ_grid.info.resolution);  // ceil to be safe
  // Supposing that drone current cells are free, mask around drone pose
  cv::Mat mask = cv::Mat::zeros(mat.size(), CV_8UC1);
  cv::Point2i p1 = cv::Point2i(origin.y - iterations, origin.x - iterations);
  cv::Point2i p2 = cv::Point2i(origin.y + iterations, origin.x + iterations);
  cv::rectangle(mask, p1, p2, 255, -1);
  cv::bitwise_or(mat, mask, mat);

  cv::erode(mat, mat, cv::Mat(), cv::Point(-1, -1), iterations);

  // Visualize obstacle map
  mat.at<uchar>(origin.x, origin.y) = 128;
  // mat.at<uchar>(goal_px.x, goal_px.y) = 128;
  auto obs_grid = imgToGrid(mat, occ_grid.header, occ_grid.info.resolution);

  update_graph(mat);

  return obs_grid;
}

double AStarSearcher::calc_h_cost(Point2i current, Point2i end)
{
  if (!use_heuristic_) {
    return 0;
  }
  return std::sqrt(
    std::pow(current.x - end.x, 2) +
    std::pow(current.y - end.y, 2));
}

double AStarSearcher::calc_g_cost(Point2i current)
{
  return 1;
}

int AStarSearcher::hash_key(Point2i point)
{
  auto px = cellToPixel(point, graph_);
  return px.x * graph_.cols + px.y;
}

bool AStarSearcher::cell_in_limits(Point2i point)
{
  auto px = cellToPixel(point, graph_);
  return px.x >= 0 && px.x < graph_.cols &&
         px.y >= 0 && px.y < graph_.rows;
}

bool AStarSearcher::cell_occuppied(Point2i point)
{
  auto px = cellToPixel(point, graph_);
  return graph_.at<uchar>(px.x, px.y) == 0;
}

/* Utils */

cv::Point2i AStarSearcher::cellToPixel(Point2i cell, int rows, int cols)
{
  int pixel_x = rows - cell.x - 1;
  int pixel_y = cols - cell.y - 1;
  return cv::Point2i(pixel_x, pixel_y);
}

cv::Point2i AStarSearcher::cellToPixel(Point2i cell, cv::Mat map)
{
  return cellToPixel(cell, map.rows, map.cols);
}

cv::Point2i AStarSearcher::cellToPixel(Point2i cell, nav_msgs::msg::MapMetaData map_info)
{
  return cellToPixel(cell, map_info.height, map_info.width);
}

Point2i AStarSearcher::pixelToCell(
  cv::Point2i pixel, nav_msgs::msg::MapMetaData map_info)
{
  Point2i cell;
  cell.x = map_info.width - pixel.x;
  cell.y = map_info.height - pixel.y;
  return cell;
}

cv::Mat AStarSearcher::gridToImg(
  nav_msgs::msg::OccupancyGrid occ_grid, double thresh, bool unknown_as_free)
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

nav_msgs::msg::OccupancyGrid AStarSearcher::imgToGrid(
  const cv::Mat img, const std_msgs::msg::Header & header, double grid_resolution)
{
  cv::Mat mat = img.clone();

  // Grid header
  nav_msgs::msg::OccupancyGrid occ_grid;
  occ_grid.header = header;
  occ_grid.info.width = mat.cols;
  occ_grid.info.height = mat.rows;
  occ_grid.info.resolution = grid_resolution;
  occ_grid.info.origin.position.x =
    -mat.cols / 2 * grid_resolution;    // only valid if frame is earth?
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
