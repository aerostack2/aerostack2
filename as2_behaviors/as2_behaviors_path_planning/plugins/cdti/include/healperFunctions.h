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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "as2_msgs/msg/a_graph.hpp"
#include "as2_msgs/msg/path.hpp"
#include "as2_msgs/msg/paths.hpp"
#include "as2_msgs/msg/weight.hpp"
#include "as2_msgs/msg/n_properties.hpp"
#include <boost/graph/metric_tsp_approx.hpp>

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

//std::pair<bool, std::vector<int>> checkGraphDegree (const boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS>& graph);
//std::vector<int> getRouteFromGraph(const Graph& originalGraph, int startVertex);
Graph generateRandomGraph(int numVertices, int numEdges);
std::vector<EdgeDescriptor> getMST(const Graph& graph);
Graph graphFromMST(const Graph& originalGraph, const std::vector<EdgeDescriptor>& mst_edges);
std::pair<std::vector<VertexDescriptor>, std::vector<double>> computeDijkstra(const Graph& g, int source);
std::vector<int> getPath(const std::vector<VertexDescriptor>& predecessors, int target);
Graph graphFromNodesEdges(
    const std::vector<as2_msgs::msg::NProperties>& nproperties, 
    const std::vector<as2_msgs::msg::Path>& aristas, 
    const std::vector<as2_msgs::msg::Weight>& weights
);
std::vector<int> heuresticTSP(const Graph& graph, int startVertex);