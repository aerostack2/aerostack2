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
 *  \file       a_star_graph_planner.cpp
 *  \brief      a_star_graph_planner implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include "a_star_graph_planner.hpp"

AStarGraphPlanner::AStarGraphPlanner() {}

as2_msgs::msg::Graph AStarGraphPlanner::update_graph_msg(const as2_msgs::msg::Graph & graph)
{
  graph_ = graph;
  return graph;
}

std::vector<Point2i> AStarGraphPlanner::get_neighbors(const CellNodePtr & cell_ptr)
{
  std::vector<Point2i> neighbors;
  int start = cell_ptr->coordinates().x * graph_.nodes.size();
  int end = start + graph_.nodes.size();
  for (int i = start; i < end; i++) {
    if (graph_.distances[i] > 0.0 && !std::isinf(graph_.distances[i])) {
      neighbors.push_back(Point2i(i % graph_.nodes.size(), i % graph_.nodes.size()));
    }
  }
  return neighbors;
}

double AStarGraphPlanner::calc_h_cost(Point2i current, Point2i end)
{
  if (!use_heuristic_) {
    return 0;
  }
  return std::sqrt(
    std::pow(current.x - end.x, 2) +
    std::pow(current.y - end.y, 2));
}

double AStarGraphPlanner::calc_g_cost(Point2i current)
{
  return graph_.distances[hash_key(current)];
}

int AStarGraphPlanner::hash_key(Point2i point)
{
  return point.x * graph_.nodes.size() + point.y;
}

bool AStarGraphPlanner::node_in_limits(Point2i point)
{
  return true;
}

bool AStarGraphPlanner::node_occuppied(Point2i point)
{
  // every node in graph is free
  return false;
}
