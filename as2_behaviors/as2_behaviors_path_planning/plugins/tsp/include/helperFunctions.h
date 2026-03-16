#pragma once

#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <unordered_map>
#include <stack>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp> 
#include <fstream>  
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/range/iterator_range.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/graph/metric_tsp_approx.hpp>
#include <as2_msgs/msg/a_graph.hpp>

struct NodeCoordinates{
    uint32_t id;
    double x;
    double y;
};

typedef boost::adjacency_list<
    boost::vecS, 
    boost::vecS, 
    boost::undirectedS, 
    NodeCoordinates,
    boost::property<boost::edge_weight_t, double> 
> Graph;
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;
typedef std::vector<VertexDescriptor> VertexDescriptorList;

std::pair<std::vector<VertexDescriptor>, std::vector<double>> computeDijkstra(const Graph& g, int source);
std::vector<int> getPath(const std::vector<VertexDescriptor>& predecessors, int target);

Graph buildGraphFromRaw(
    const double *x_coords,
    const double *y_coords,
    const std::vector<std::vector<int>> &adj_list,
    double penalty_x, double penalty_y);
std::vector<int> greedyTargetTSP(const Graph& graph, int startVertex, const std::vector<int>& targets);
