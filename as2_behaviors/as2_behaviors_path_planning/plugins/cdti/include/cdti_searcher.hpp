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

#ifndef CDTI_SEARCHER_HPP_
#define CDTI_SEARCHER_HPP_

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "graph_searcher.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "healperFunctions.h"
/*
#ifndef POINT2I_STRUCT

#define POINT2I_STRUCT
struct Point2i { int x; int y;
  Point2i() : x(0), y(0) {}
  Point2i(int x_, int y_) : x(x_), y(y_) {}
};
#endif
*/
class CDTIRoutingSearcher
{
public:
 std::vector<Point2i> solve_dijkstra(
    const nav_msgs::msg::OccupancyGrid & occ_grid, 
    Point2i start_cell, 
    Point2i goal_cell,
    double safety_distance);
bool cell_occupied(Point2i cell);
private:
  int cell_to_index(int x, int y, int width) { return y * width + x; }
  Point2i index_to_cell(int index, int width) { return {index % width, index / width}; }



  int width_;
  int height_;
  nav_msgs::msg::OccupancyGrid last_grid_;
};

#endif  // A_STAR_SEARCHER_HPP_
