#include "healperFunctions.h"

std::vector<int> heuresticTSP(const Graph& graph, int startVertex){
    VertexDescriptorList vertices;
    std::vector<VertexDescriptor> tour;
    int total_distance = 0;
    auto visitor = boost::make_tsp_tour_len_visitor(
        graph, 
        std::back_inserter(vertices), 
        total_distance,
        boost::get(boost::edge_weight, graph)
    );

    metric_tsp_approx(
        graph, 
        visitor
    );
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        if (vertices[i] == startVertex) {
            for (int j = 0; j < static_cast<int>(vertices.size()); ++j) {
                tour.push_back(vertices[(i + j) % vertices.size()]);
            }
            break;
        }
    }
    std::vector<int> tour_int;
    for (const auto& v : tour) {
        tour_int.push_back(v);
    }
    return tour_int;
}

Graph graphFromNodesEdges(
    const std::vector<cdti_routing::msg::NProperties>& nproperties, 
    const std::vector<cdti_routing::msg::Path>& aristas, 
    const std::vector<cdti_routing::msg::Weight>& weights) 
{
    Graph graph(nproperties.size());

    for (size_t i = 0; i < nproperties.size(); ++i) {
        graph[i].id = nproperties[i].id;
        graph[i].x = nproperties[i].x;
        graph[i].y = nproperties[i].y;
    }

    for (size_t i = 0; i < aristas.size(); ++i) {
        for (size_t j = 0; j < aristas[i].path.size(); ++j) {
            int u = i;
            int v = aristas[i].path[j];
            
            if (i < weights.size() && j < weights[i].weight.size()) {
                double weight = weights[i].weight[j];
                boost::add_edge(u, v, weight, graph);
            }
        }
    }
    return graph;
}


std::pair<std::vector<VertexDescriptor>, std::vector<double>> 
computeDijkstra(const Graph& g, int source)
{
    std::vector<VertexDescriptor> predecessor(boost::num_vertices(g));
    std::vector<double> distance(boost::num_vertices(g));

    boost::dijkstra_shortest_paths(
        g,
        source,
        boost::predecessor_map(
            boost::make_iterator_property_map(
                predecessor.begin(),
                boost::get(boost::vertex_index, g)))
        .distance_map(
            boost::make_iterator_property_map(
                distance.begin(),
                boost::get(boost::vertex_index, g)))
    );

    return { predecessor, distance };
}


std::vector<int> getPath(const std::vector<VertexDescriptor>& predecessors, int target) {
    std::vector<int> path;
    if (predecessors.empty() || target < 0 || target >= static_cast<int>(predecessors.size())) {
        return path; 
    }

    int curr = target;
    size_t max_path_length = predecessors.size(); 

    for (size_t i = 0; i <= max_path_length; ++i) {
        path.push_back(curr);

        if (predecessors[curr] == curr) {
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        curr = predecessors[curr];
    }

    return {}; 
}

std::vector<EdgeDescriptor> getMST(const Graph& graph) {
    std::vector<EdgeDescriptor> mst_edges;
    
    boost::kruskal_minimum_spanning_tree(graph, std::back_inserter(mst_edges));
    
    return mst_edges;
}

Graph graphFromMST(const Graph& originalGraph, const std::vector<EdgeDescriptor>& mst_edges) {
    Graph mst_graph(boost::num_vertices(originalGraph));
    
    for (size_t i = 0; i < boost::num_vertices(originalGraph); ++i) {
        mst_graph[i].id = originalGraph[i].id;
        mst_graph[i].x = originalGraph[i].x;
        mst_graph[i].y = originalGraph[i].y;
    }

    for (const auto& edge : mst_edges) {
        int u = boost::source(edge, originalGraph);
        int v = boost::target(edge, originalGraph);
        double weight = boost::get(boost::edge_weight_t(), originalGraph, edge);
        
        boost::add_edge(u, v, weight, mst_graph);    
    }
    
    return mst_graph;
}

std::vector<int> checkGraphDegree (const boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS>& graph) {
    std::vector<int> verticesNoPar;
    for (int i = 0; i < boost::num_vertices(graph); ++i) {
        
        if (boost::degree(i, graph)%2 != 0 || boost::degree(i, graph) == 0) {
            verticesNoPar.push_back(i);
        }
    }

    return verticesNoPar;
}
/*

std::vector<int> getRouteFromGraph(const Graph& originalGraph, int startVertex) {
    std::stack<int> curr_path;
    std::vector<int> circuit;
    Graph graph = originalGraph;
    
    auto degreeCheck = checkGraphDegree(graph);
    if (!degreeCheck.first || boost::num_edges(graph) == 0) {
        std::cout << "Graph is not Eulerian or has no edges. Cannot find Eulerian circuit." << std::endl;
        return degreeCheck.second;
    }

    curr_path.push(startVertex);
    int curr_v = startVertex;

    while (!curr_path.empty())
    {
        if (boost::degree(curr_v, graph) > 0) {
            curr_path.push(curr_v);
            int next_v = *boost::adjacent_vertices(curr_v, graph).first;
            boost::remove_edge(curr_v, next_v, graph);
            curr_v = next_v;
    }
        else {
            circuit.push_back(curr_v);
            curr_v = curr_path.top();
            curr_path.pop();
        }
    }
    for (int i=circuit.size()-1; i>=0; i--) {
		std::cout << circuit[i];
		if (i) 	std::cout<<" -> ";
	}
    return circuit;
}
*/
