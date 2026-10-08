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
 *  \file       helper_functions.cpp
 *  \brief      helper functions implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#include "helper_functions.h"

std::vector<int> getPath(const std::vector<int> & predecessors, int target)
{
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

void computeDijkstra(
  int numV,
  const AdjList & adjList,
  int source,
  std::vector<int> & predecessor,
  std::vector<double> & distance)
{
  predecessor.assign(numV, -1);
  distance.assign(numV, std::numeric_limits<double>::max());

  predecessor[source] = source;
  distance[source] = 0.0;

  using PQEntry = std::pair<double, int>;
  std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
  pq.push({0.0, source});

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > distance[u]) {
      continue;
    }

    for (const auto & [v, w] : adjList[u]) {
      const double newDist = d + w;
      if (newDist < distance[v]) {
        distance[v] = newDist;
        predecessor[v] = u;
        pq.push({newDist, v});
      }
    }
  }
}

std::vector<int> greedyTargetTSP(
  const AdjList & adjList, int startVertex,
  const std::vector<int> & targets)
{
  std::vector<int> fullPath;

  std::set<int> unvisitedTargets(targets.begin(), targets.end());
  unvisitedTargets.erase(startVertex);

  int current = startVertex;
  fullPath.push_back(current);

  while (!unvisitedTargets.empty()) {
    std::vector<int> predecessors;
    std::vector<double> distances;
    computeDijkstra(adjList.size(), adjList, current, predecessors, distances);

    int bestTarget = -1;
    double minDist = std::numeric_limits<double>::max();

    for (int target : unvisitedTargets) {
      if (distances[target] < minDist) {
        minDist = distances[target];
        bestTarget = target;
      }
    }

    if (bestTarget == -1 || minDist == std::numeric_limits<double>::max()) {
      return {};
    }

    std::vector<int> segment = getPath(predecessors, bestTarget);

    for (size_t i = 1; i < segment.size(); ++i) {
      fullPath.push_back(segment[i]);
    }

    current = bestTarget;
    unvisitedTargets.erase(current);
  }

  if (current != startVertex) {
    std::vector<int> predecessors;
    std::vector<double> distances;
    computeDijkstra(adjList.size(), adjList, current, predecessors, distances);

    if (distances[startVertex] == std::numeric_limits<double>::max()) {
      return {};
    }

    std::vector<int> returnSegment = getPath(predecessors, startVertex);

    for (size_t i = 1; i < returnSegment.size(); ++i) {
      fullPath.push_back(returnSegment[i]);
    }
  }

  return fullPath;
}

AdjList buildGraphFromRaw(
  const double * x_coords,
  const double * y_coords,
  const std::vector<std::vector<int>> & adj_list,
  double penalty_x, double penalty_y)
{
  const int n = static_cast<int>(adj_list.size());
  AdjList adjList(n);

  for (int u = 0; u < n; ++u) {
    for (int v : adj_list[u]) {
      if (u < v) {
        double dx = std::abs(x_coords[u] - x_coords[v]);
        double dy = std::abs(y_coords[u] - y_coords[v]);
        double weight = std::sqrt(std::pow(dx * penalty_x, 2) + std::pow(dy * penalty_y, 2));
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight});
      }
    }
  }

  return adjList;
}
