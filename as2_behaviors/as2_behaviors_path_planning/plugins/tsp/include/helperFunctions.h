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
 *  \file       helperFunctions.hpp
 *  \brief      helper functions header file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <stack>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <utility>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <as2_msgs/msg/a_graph.hpp>

struct NodeCoordinates
{
  uint32_t id;
  double x;
  double y;
};

typedef boost::adjacency_list <
  boost::vecS,
  boost::vecS,
  boost::undirectedS,
  NodeCoordinates,
  boost::property < boost::edge_weight_t, double >>
  Graph;
typedef boost::graph_traits < Graph > ::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits < Graph > ::vertex_descriptor VertexDescriptor;
typedef std::vector < VertexDescriptor > VertexDescriptorList;

std::pair < std::vector < VertexDescriptor >, std::vector < double >> computeDijkstra(
  const Graph & g, int source);
std::vector < int > getPath(const std::vector < VertexDescriptor > &predecessors, int target);

Graph buildGraphFromRaw(
  const double * x_coords,
  const double * y_coords,
  const std::vector < std::vector < int >> &adj_list,
  double penalty_x, double penalty_y);
std::vector < int > greedyTargetTSP(
  const Graph & graph, int startVertex,
  const std::vector < int > &targets);
