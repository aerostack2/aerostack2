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
 *  \file       a_star_graph_planner.hpp
 *  \brief      a_star_graph_planner header file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#ifndef A_STAR_GRAPH_PLANNER_HPP_
#define A_STAR_GRAPH_PLANNER_HPP_

#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "as2_msgs/msg/graph.hpp"
#include "as2_msgs/msg/node.hpp"
#include "search_algorithm.hpp"

class AStarGraphPlanner : public SearchAlgorithm
{
public:
  AStarGraphPlanner();

  /**
   * @brief Update the graph
   * @param graph new graph msg
   */
  as2_msgs::msg::Graph update_graph_msg(const as2_msgs::msg::Graph & graph);

protected:
  as2_msgs::msg::Graph graph_;

  bool use_heuristic_ = true;

  std::vector<Point2i> get_neighbors(const CellNodePtr & cell_ptr) override;
  double calc_h_cost(Point2i current, Point2i end) override;
  double calc_g_cost(Point2i current) override;
  int hash_key(Point2i point) override;
  bool node_in_limits(Point2i point) override;
  bool node_occuppied(Point2i point) override;
};

#endif  // A_STAR_GRAPH_PLANNER_HPP_
