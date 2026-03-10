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

std::pair<double, double> getCentroid(const std::vector<std::pair<double, double>> &points)
{
    double sx = 0.0, sy = 0.0;
    for (const auto &p : points)
    {
        sx += p.first;
        sy += p.second;
    }
    return {sx / points.size(), sy / points.size()};
}

void sortVertices(std::vector<std::pair<double, double>> &points)
{
    if (points.empty())
        return;

    std::pair<double, double> center = getCentroid(points);

    std::sort(points.begin(), points.end(), [center](const std::pair<double, double> &a, const std::pair<double, double> &b)
              {
        double angleA = std::atan2(a.second - center.second, a.first - center.first);
        double angleB = std::atan2(b.second - center.second, b.first - center.first);
        return angleA < angleB; });
}

int orientation(const std::pair<double, double> &p, const std::pair<double, double> &q, const std::pair<double, double> &r)
{
    double val = (q.second - p.second) * (r.first - q.first) -
                 (q.first - p.first) * (r.second - q.second);
    if (std::abs(val) < std::numeric_limits<double>::min())
        return 0;
    return (val > 0) ? 1 : 2;
}

bool onSegment(const std::pair<double, double> &p, const std::pair<double, double> &q, const std::pair<double, double> &r)
{
    return q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) &&
           q.second <= std::max(p.second, r.second) && q.second >= std::min(p.second, r.second);
}

bool Intersect(const std::pair<double, double> &p1, const std::pair<double, double> &q1, const std::pair<double, double> &p2, const std::pair<double, double> &q2)
{
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false;
}

bool edgeIntersectsPolygon(const std::pair<double, double> &u, const std::pair<double, double> &v, const std::vector<std::pair<double, double>> &polygon)
{
    double minX_edge = std::min(u.first, v.first), maxX_edge = std::max(u.first, v.first);
    double minY_edge = std::min(u.second, v.second), maxY_edge = std::max(u.second, v.second);

    double minX_poly = std::numeric_limits<double>::max(), maxX_poly = -std::numeric_limits<double>::max(), minY_poly = std::numeric_limits<double>::max(), maxY_poly = -std::numeric_limits<double>::max();
    for (const auto &p : polygon)
    {
        if (p.first < minX_poly)
            minX_poly = p.first;
        if (p.first > maxX_poly)
            maxX_poly = p.first;
        if (p.second < minY_poly)
            minY_poly = p.second;
        if (p.second > maxY_poly)
            maxY_poly = p.second;
    }

    if (maxX_edge < minX_poly || minX_edge > maxX_poly ||
        maxY_edge < minY_poly || minY_edge > maxY_poly)
    {
        return false;
    }

    size_t n = polygon.size();
    for (size_t i = 0; i < n; i++)
    {
        const std::pair<double, double> &p1 = polygon[i];
        const std::pair<double, double> &p2 = polygon[(i + 1) % n];

        if (Intersect(u, v, p1, p2))
            return true;
    }
    return false;
}

bool edgeIntersectsAllPolygon(const std::pair<double, double> &u, const std::pair<double, double> &v,
                              const std::vector<std::vector<std::pair<double, double>>> &polygons)
{
    for (const auto &poly : polygons)
    {
        if (edgeIntersectsPolygon(u, v, poly))
        {
            return true;
        }
    }
    return false;
}