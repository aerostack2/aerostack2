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
 *  \file       graph_searcher.hpp
 *  \brief      graph_searcher header file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#ifndef GRAPH_SEARCHER_HPP_
#define GRAPH_SEARCHER_HPP_

#include <math.h>
#include <iostream>
#include <memory>
#include <limits>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#include "cell_node.hpp"

template<typename T>
class GraphSearcher
{
public:
  GraphSearcher()
  {
    valid_movements_.clear();
    valid_movements_.reserve(8);
    valid_movements_.emplace_back(-1, 0);
    valid_movements_.emplace_back(0, -1);
    valid_movements_.emplace_back(0, 1);
    valid_movements_.emplace_back(1, 0);
    valid_movements_.emplace_back(-1, -1);
    valid_movements_.emplace_back(-1, 1);
    valid_movements_.emplace_back(1, -1);
    valid_movements_.emplace_back(1, 1);
  }

private:
  std::unordered_map<int, CellNodePtr> nodes_visited_;
  std::unordered_map<int, CellNodePtr> nodes_to_visit_;
  std::vector<Point2i> valid_movements_;

protected:
  T graph_;
  bool use_heuristic_ = false;

  virtual void update_graph(const T & graph)
  {
    graph_ = graph;
  }

  virtual double calc_h_cost(Point2i current, Point2i end) = 0;
  virtual double calc_g_cost(Point2i current) = 0;
  virtual int hash_key(Point2i point) = 0;
  virtual bool cell_in_limits(Point2i point) = 0;

public:
  virtual bool cell_occuppied(Point2i point) = 0;

public:
  std::vector<Point2i> solve_graph(Point2i start, Point2i end)
  {
    std::vector<Point2i> path;

    nodes_to_visit_.clear();
    nodes_visited_.clear();

    int p_key = hash_key(start);
    nodes_to_visit_.emplace(p_key, std::make_shared<CellNode>(start, nullptr, 0));

    while (nodes_to_visit_.size() > 0) {
      // find the less cost node
      std::shared_ptr<CellNode> cell_ptr = nullptr;
      double min_cost = std::numeric_limits<double>::infinity();
      for (auto & node : nodes_to_visit_) {
        float cost = node.second->get_total_cost();
        if (cost < min_cost) {
          cell_ptr = node.second;
          min_cost = cost;
        }
      }

      // no next node to visit and goal is not reached
      if (cell_ptr == nullptr) {
        throw std::runtime_error("node without ptr");
      }

      // if goal is finded
      if (cell_ptr->coordinates() == end) {
        std::shared_ptr<CellNode> parent_ptr = cell_ptr;
        do {
          path.emplace_back(parent_ptr->coordinates());
          parent_ptr = parent_ptr->parent_ptr();
        } while (parent_ptr != nullptr);
        break;
      }

      // if goal is not found yet, add neighbors to visit
      for (auto & movement : valid_movements_) {
        Point2i new_node = cell_ptr->coordinates();
        new_node.x += movement.x;
        new_node.y += movement.y;
        int key = hash_key(new_node);

        // cel inside map limits
        if (!cell_in_limits(new_node)) {
          continue;
        }
        // already visited
        if (nodes_visited_.find(key) != nodes_visited_.end()) {
          continue;
        }
        // already added to visit
        if (nodes_to_visit_.find(key) != nodes_to_visit_.end()) {
          continue;
        }
        // cell occupied
        if (cell_occuppied(new_node)) {
          continue;
        }

        nodes_to_visit_.emplace(
          key,
          std::make_shared<CellNode>(
            new_node, cell_ptr, calc_g_cost(new_node), calc_h_cost(new_node, end)));
      }
      // add node to visited and remove from to visit
      int key = hash_key(cell_ptr->coordinates());
      nodes_visited_.emplace(key, cell_ptr);
      nodes_to_visit_.erase(key);
    }

    if (path.size() > 0) {
      std::vector<Point2i> inverted_path;
      inverted_path.reserve(path.size());
      for (int i = path.size() - 1; i >= 0; i--) {
        inverted_path.emplace_back(path[i]);
      }
      path = inverted_path;
    }

    // Visualize path planning result (for cv::Mat graphs)
    visualize_search(start, end, path);

    return path;
  }

private:
  void visualize_search(
    const Point2i & start, const Point2i & end,
    const std::vector<Point2i> & path)
  {
    // Only visualize if graph_ is cv::Mat (compile-time check via SFINAE or runtime check)
    if constexpr (std::is_same_v<T, cv::Mat>) {
      if (graph_.empty()) {
        return;
      }

      // Create color visualization image
      cv::Mat vis_img;
      if (graph_.channels() == 1) {
        cv::cvtColor(graph_, vis_img, cv::COLOR_GRAY2BGR);
      } else {
        vis_img = graph_.clone();
      }

      // Mark visited nodes in blue
      for (const auto & node : nodes_visited_) {
        Point2i pos = node.second->coordinates();
        if (pos.x >= 0 && pos.x < vis_img.rows && pos.y >= 0 && pos.y < vis_img.cols) {
          int px = vis_img.cols - pos.y - 1;
          int py = vis_img.rows - pos.x - 1;
          vis_img.at<cv::Vec3b>(py, px) = cv::Vec3b(200, 100, 50);  // Light blue
        }
      }

      // Mark path in green
      for (const auto & pos : path) {
        if (pos.x >= 0 && pos.x < vis_img.rows && pos.y >= 0 && pos.y < vis_img.cols) {
          int px_x = vis_img.cols - pos.y - 1;
          int px_y = vis_img.rows - pos.x - 1;
          vis_img.at<cv::Vec3b>(px_y, px_x) = cv::Vec3b(0, 255, 0);  // Green
        }
      }

      // Mark start in cyan
      int pix_x = vis_img.cols - start.y - 1;
      int pix_y = vis_img.rows - start.x - 1;
      // std::cout << "Start position: (" << start.x << ", " << start.y << "), Pixel position: (" <<
      //   pix_x << ", " << pix_y << ")" << std::endl;
      if (start.x >= 0 && start.x < vis_img.rows && start.y >= 0 && start.y < vis_img.cols) {
        cv::circle(vis_img, cv::Point(pix_x, pix_y), 1, cv::Scalar(255, 255, 0), -1);
      }

      // Mark end in magenta
      int pixel_x = vis_img.cols - end.y - 1;
      int pixel_y = vis_img.rows - end.x - 1;
      if (end.x >= 0 && end.x < vis_img.rows && end.y >= 0 && end.y < vis_img.cols) {
        cv::circle(vis_img, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(255, 0, 255), -1);
      }

      // Save visualization
      cv::imwrite("solve_graph_vis.png", vis_img);
    }
  }

};

#endif  // GRAPH_SEARCHER_HPP_
