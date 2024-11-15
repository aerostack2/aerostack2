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
 *  \file       a_star_searcher.hpp
 *  \brief      a_star_searcher header file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef A_STAR_SEARCHER_HPP_
#define A_STAR_SEARCHER_HPP_

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "graph_searcher.hpp"

class AStarSearcher : public GraphSearcher<cv::Mat>
{
public:
  /**
   * @brief Update the occupancy grid
   * @param occ_grid occupancy grid
   * @param drone_pose drone pose in cell coordinates
   * @param safety_distance safety distance in meters
   */
  nav_msgs::msg::OccupancyGrid update_grid(
    const nav_msgs::msg::OccupancyGrid & occ_grid, const Point2i & drone_pose,
    double safety_distance);

protected:
  bool use_heuristic_ = true;

  double calc_h_cost(Point2i current, Point2i end) override;
  double calc_g_cost(Point2i current) override;
  int hash_key(Point2i point) override;
  bool cell_in_limits(Point2i point) override;
  bool cell_occuppied(Point2i point) override;

public:
  /**
   * @brief Convert cell coordinates to pixel coordinates
   * @param cell cell coordinates
   * @param rows number of rows
   * @param cols number of columns
   */
  cv::Point2i cellToPixel(Point2i cell, int rows, int cols);

  /**
   * @brief Convert cell coordinates to pixel coordinates
   * @param cell cell coordinates
   * @param map map
   */
  cv::Point2i cellToPixel(Point2i cell, cv::Mat map);

  /**
   * @brief Convert cell coordinates to pixel coordinates
   * @param cell cell coordinates
   * @param map_info map metadata
   */
  cv::Point2i cellToPixel(Point2i cell, nav_msgs::msg::MapMetaData map_info);

  /**
   * @brief Convert pixel coordinates to cell coordinates
   * @param pixel pixel coordinates
   * @param map_info map metadata
   */
  Point2i pixelToCell(cv::Point2i pixel, nav_msgs::msg::MapMetaData map_info);

  /**
   * Occupancy grid to binary image
   *
   * @param occ_grid: occupancy grid
   * @param thresh: threshold value
   * @param unknown_as_free: if true, unknown cells are considered free
   * @return: binary image
   */
  cv::Mat gridToImg(
    nav_msgs::msg::OccupancyGrid occ_grid, double thresh = 30, bool unknown_as_free = false);

  /**
   * Binary image to occupancy grid
   *
   * @param img: binary image
   * @param header: header of the occupancy grid
   * @param grid_resolution: resolution of the occupancy grid
   * @return: occupancy grid
   */
  nav_msgs::msg::OccupancyGrid
  imgToGrid(const cv::Mat img, const std_msgs::msg::Header & header, double grid_resolution);
};

#endif  // A_STAR_SEARCHER_HPP_
