// Copyright 2024 Universidad Politécnica de Madrid
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
 *  \file       a_star_searcher.cpp
 *  \brief      a_star_searcher implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include "cdti_searcher.hpp"
#include <cmath>

std::vector<Point2i> CDTIRoutingSearcher::solve_dijkstra(
  const nav_msgs::msg::OccupancyGrid & occ_grid, 
      Point2i start_cell, 
      Point2i goal_cell,
      double safety_distance)
{
  last_grid_ = occ_grid;
  width_ = occ_grid.info.width;
  height_ = occ_grid.info.height;
  cv::Mat map_mat = cv::Mat(height_, width_, CV_8SC1, (void*)occ_grid.data.data());
    
    cv::Mat bin_map;
    cv::threshold(map_mat, bin_map, 50, 255, cv::THRESH_BINARY_INV);   

    int erosion_size = std::ceil(safety_distance / occ_grid.info.resolution);
    if(erosion_size > 0) {
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*erosion_size+1, 2*erosion_size+1));
        cv::erode(bin_map, bin_map, element);
    }



    int total_nodes = width_ * height_;

    std::vector<as2_msgs::msg::NProperties> nproperties(total_nodes);
    std::vector<as2_msgs::msg::Path> aristas(total_nodes);
    std::vector<as2_msgs::msg::Weight> weights(total_nodes);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int current_idx = cell_to_index(x, y, width_);

            nproperties[current_idx].id = current_idx;
            nproperties[current_idx].x = x;
            nproperties[current_idx].y = y;

            if (bin_map.at<uint8_t>(y, x) > 0) {
                
                int neighbors[2][2] = {{1, 0}, {0, 1}};

                for (auto& offset : neighbors) {
                    int nx = x + offset[0];
                    int ny = y + offset[1];

                    if (nx < width_ && ny < height_) {
                        if (bin_map.at<uint8_t>(ny, nx) > 0) {
                            int neighbor_idx = cell_to_index(nx, ny, width_);
                            
                            aristas[current_idx].path.push_back(neighbor_idx);
                            weights[current_idx].weight.push_back(1.0);
                        }
                    }
                }
            }
        }
    }

    Graph graph = graphFromNodesEdges(nproperties, aristas, weights);

    int start_node = cell_to_index(start_cell.x, start_cell.y, width_);
    int goal_node = cell_to_index(goal_cell.x, goal_cell.y, width_);
    
    if (start_node >= (int)boost::num_vertices(graph) || goal_node >= (int)boost::num_vertices(graph)) {
        return {};
    }

   auto [predecessors, _] = computeDijkstra(graph, start_node);
    std::vector<int> path_nodes = getPath(predecessors, goal_node);

    std::vector<Point2i> path_points;
    if (path_nodes.empty()) return {};

    for (int node_id : path_nodes) {
        path_points.push_back(index_to_cell(node_id, width_));
    }

    return path_points;

}
bool CDTIRoutingSearcher::cell_occupied(Point2i cell) {
    if (width_ == 0 || last_grid_.data.empty()) return true;
    int idx = cell_to_index(cell.x, cell.y, width_);
    if (idx < 0 || idx >= (int)last_grid_.data.size()) return true;
    return last_grid_.data[idx] > 50;
}
