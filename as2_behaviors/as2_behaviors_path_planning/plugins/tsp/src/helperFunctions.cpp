#include "helperFunctions.h"

std::vector<int> getPath(const std::vector<VertexDescriptor> &predecessors, int target)
{
    std::vector<int> path;
    if (predecessors.empty() || target < 0 || target >= static_cast<int>(predecessors.size()))
    {
        return path;
    }

    int curr = target;
    size_t max_path_length = predecessors.size();

    for (size_t i = 0; i <= max_path_length; ++i)
    {
        path.push_back(curr);

        if (predecessors[curr] == curr)
        {
            std::reverse(path.begin(), path.end());
            return path;
        }

        curr = predecessors[curr];
    }

    return {};
}

std::pair<std::vector<VertexDescriptor>, std::vector<double>>
computeDijkstra(const Graph &g, int source)
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
                    boost::get(boost::vertex_index, g))));

    return {predecessor, distance};
}


std::vector<int> greedyTargetTSP(const Graph &graph, int startVertex, const std::vector<int> &targets)
{
    std::vector<int> fullPath;

    std::set<int> unvisitedTargets(targets.begin(), targets.end());
    unvisitedTargets.erase(startVertex);

    int current = startVertex;
    fullPath.push_back(current);

    while (!unvisitedTargets.empty())
    {

        auto [predecessors, distances] = computeDijkstra(graph, current);
        int bestTarget = -1;
        double minDist = std::numeric_limits<double>::max();

        for (int target : unvisitedTargets)
        {
            if (distances[target] < minDist)
            {
                minDist = distances[target];
                bestTarget = target;
            }
        }

        if (bestTarget == -1 || minDist == std::numeric_limits<double>::max())
        {
            return {};
        }

        std::vector<int> segment = getPath(predecessors, bestTarget);

        for (size_t i = 1; i < segment.size(); ++i)
        {
            fullPath.push_back(segment[i]);
        }

        current = bestTarget;
        unvisitedTargets.erase(current);
    }
    if (current != startVertex)
    {
        auto [predecessors, distances] = computeDijkstra(graph, current);

        if (distances[startVertex] == std::numeric_limits<double>::max())
        {
            return {};
        }

        std::vector<int> returnSegment = getPath(predecessors, startVertex);

        for (size_t i = 1; i < returnSegment.size(); ++i)
        {
            fullPath.push_back(returnSegment[i]);
        }
    }

    return fullPath;
}

Graph buildGraphFromRaw(
    const double *x_coords,
    const double *y_coords,
    const std::vector<std::vector<int>> &adj_list,
    double penalty_x, double penalty_y)
{
    Graph graph(adj_list.size());

    for (int i = 0; i < adj_list.size(); ++i)
    {
        graph[i].id = i;
        graph[i].x = x_coords[i];
        graph[i].y = y_coords[i];
    }


 
        for (int u = 0; u < adj_list.size(); ++u) 
        {
        if (u >= static_cast<int>(adj_list.size())) break;

        const auto& neighbors = adj_list[u];

        for (int v : neighbors) {
            if (u < v) {
                
                double dx = std::abs(x_coords[u] - x_coords[v]);
                double dy = std::abs(y_coords[u] - y_coords[v]);

                double weight = std::sqrt(std::pow(dx * penalty_x, 2) + std::pow(dy * penalty_y, 2));

                boost::add_edge(u, v, weight, graph);
            }
        }

        }
    
    return graph;
}

