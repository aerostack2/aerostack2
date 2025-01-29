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
 *  \file       voronoi_planner.cpp
 *  \brief      voronoi_planner implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include <algorithm>

#include "voronoi_planner.hpp"

VoronoiPlanner::VoronoiPlanner()
{
  valid_movements_.clear();
  valid_movements_.reserve(8);
  valid_movements_.emplace_back(-1, 0);
  valid_movements_.emplace_back(0, -1);
  valid_movements_.emplace_back(0, 1);
  valid_movements_.emplace_back(1, 0);
  valid_movements_.emplace_back(-1, -1);
  valid_movements_.emplace_back(-1, 1);
  valid_movements_.emplace_back(1, -1);
  valid_movements_.emplace_back(1, 1);
}

void VoronoiPlanner::update_voronoi(const DynamicVoronoi & voronoi)
{
  graph_ = voronoi;
}

std::vector<Point2i> VoronoiPlanner::get_neighbors(const CellNodePtr & cell_ptr)
{
  std::vector<Point2i> neighbors;
  for (auto & movement : valid_movements_) {
    Point2i new_node = cell_ptr->coordinates();
    new_node.x += movement.x;
    new_node.y += movement.y;
    neighbors.emplace_back(new_node);
  }
  return neighbors;
}

double VoronoiPlanner::calc_h_cost(Point2i current, Point2i end)
{
  if (!use_heuristic_) {
    return 0;
  }
  return std::sqrt(
    std::pow(current.x - end.x, 2) +
    std::pow(current.y - end.y, 2));
}

double VoronoiPlanner::calc_g_cost(Point2i current)
{
  float dist = graph_.getDistance(current.x, current.y);
  dist = 300.0f - std::min(dist, 300.0f);
  return dist;
}

int VoronoiPlanner::hash_key(Point2i point)
{
  return point.y * graph_.getSizeX() + point.x;
}

bool VoronoiPlanner::node_in_limits(Point2i point)
{
  return point.x >= 0 && point.x < graph_.getSizeX() &&
         point.y >= 0 && point.y < graph_.getSizeY();
}

bool VoronoiPlanner::node_occuppied(Point2i point)
{
  return graph_.isOccupied(point.x, point.y);
}
