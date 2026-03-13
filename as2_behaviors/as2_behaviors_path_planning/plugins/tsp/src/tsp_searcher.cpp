// Copyright 2026 Universidad Politécnica de Madrid
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
 *  \file       tsp_searcher.cpp
 *  \brief      graph_searcher implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#include "tsp_searcher.hpp"
#include <cmath>
#include <fmt/ranges.h>
#include <iostream>
#include <string>

std::vector<Point2i> TSPRoutingSearcher::solve_tsp(
const as2_msgs::msg::AGraph & graph_msg, double penalty_x, double penalty_y)
{
    std::vector<double> x_vec;
    x_vec.reserve(graph_msg.nproperties.size());
    for (const auto &node : graph_msg.nproperties) {
        x_vec.push_back(node.x);
}    std::vector<double> y_vec;
    y_vec.reserve(graph_msg.nproperties.size());
    for (const auto &node : graph_msg.nproperties) {
        y_vec.push_back(node.y);
    }

    std::vector<std::vector<int>> adj;
    adj.reserve(graph_msg.adjesency_list.size());

    for (const auto &vertex_list : graph_msg.adjesency_list) {
        std::vector<int> converted(vertex_list.vertecies.begin(), vertex_list.vertecies.end());
        adj.push_back(converted);
    }

    Graph graph = buildGraphFromRaw(x_vec.data(), y_vec.data(), adj, penalty_x, penalty_y);
    auto start_node = graph_msg.start_node;
    std::vector<int> targets;
    for (const auto &waypoint : graph_msg.route) {
        for (const auto &vertex : waypoint.vertecies) {
            targets.push_back(vertex);
        }
    }

    std::vector<int> path = greedyTargetTSP(graph, start_node, targets);

    std::vector<Point2i> point_path;
    for (int idx : path) {
        point_path.emplace_back(static_cast<int>(x_vec[idx]), static_cast<int>(y_vec[idx]));
    }


    return point_path;
}
bool TSPRoutingSearcher::cell_occupied(Point2i cell) {
    return false;  
}
