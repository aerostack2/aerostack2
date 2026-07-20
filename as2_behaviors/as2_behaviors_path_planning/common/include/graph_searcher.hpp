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
#include <queue>
#include <unordered_map>
#include <vector>
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
  // Min-heap ordered by total cost (g + h). May contain stale entries,
  // superseded by a cheaper open node or already moved to the closed set;
  // see solve_graph() for the lazy-deletion handling of those entries.
  struct CellNodeCmp
  {
    bool operator()(const CellNodePtr & a, const CellNodePtr & b) const
    {
      return a->get_total_cost() > b->get_total_cost();
    }
  };

  std::unordered_map<int, CellNodePtr> nodes_visited_;
  // Authoritative open set: best known node currently open for each key.
  std::unordered_map<int, CellNodePtr> nodes_to_visit_;
  std::priority_queue<CellNodePtr, std::vector<CellNodePtr>, CellNodeCmp> open_heap_;
  std::vector<Point2i> valid_movements_;

protected:
  T graph_;
  bool use_heuristic_ = false;

  virtual void update_graph(const T & graph)
  {
    graph_ = graph;
  }

  /**
   * @brief Heuristic cost estimate from current to end.
   * @warning Must be admissible and consistent with respect to calc_g_cost()'s
   * step costs: solve_graph() relies on this to never re-expand a node once it
   * has been moved to the closed set.
   */
  virtual double calc_h_cost(Point2i current, Point2i end) = 0;
  virtual double calc_g_cost(Point2i current, Point2i parent) = 0;
  virtual int hash_key(Point2i point) = 0;
  virtual bool cell_in_limits(Point2i point) = 0;
  virtual bool cell_occuppied(Point2i point) = 0;

public:
  std::vector<Point2i> solve_graph(Point2i start, Point2i end)
  {
    std::vector<Point2i> path;

    nodes_to_visit_.clear();
    nodes_visited_.clear();
    open_heap_ = std::priority_queue<CellNodePtr, std::vector<CellNodePtr>, CellNodeCmp>();

    int p_key = hash_key(start);
    CellNodePtr start_ptr = std::make_shared<CellNode>(start, nullptr, 0, calc_h_cost(start, end));
    nodes_to_visit_.emplace(p_key, start_ptr);
    open_heap_.push(start_ptr);

    while (!open_heap_.empty()) {
      std::shared_ptr<CellNode> cell_ptr = open_heap_.top();
      open_heap_.pop();

      int cur_key = hash_key(cell_ptr->coordinates());

      // lazy deletion: skip stale heap entries (already closed, or superseded
      // by a cheaper node pushed later for the same key)
      if (nodes_visited_.find(cur_key) != nodes_visited_.end()) {
        continue;
      }
      auto authoritative_it = nodes_to_visit_.find(cur_key);
      if (authoritative_it == nodes_to_visit_.end() || authoritative_it->second != cell_ptr) {
        continue;
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
        // already visited: never reopen (see calc_h_cost invariant)
        if (nodes_visited_.find(key) != nodes_visited_.end()) {
          continue;
        }
        // cell occupied
        if (cell_occuppied(new_node)) {
          continue;
        }

        CellNodePtr candidate = std::make_shared<CellNode>(
          new_node, cell_ptr, calc_g_cost(new_node, cell_ptr->coordinates()),
          calc_h_cost(new_node, end));

        auto open_it = nodes_to_visit_.find(key);
        if (open_it != nodes_to_visit_.end()) {
          // already open: relax only if this path is strictly cheaper
          if (candidate->get_g_cost() < open_it->second->get_g_cost()) {
            open_it->second = candidate;
            open_heap_.push(candidate);
          }
          continue;
        }

        nodes_to_visit_.emplace(key, candidate);
        open_heap_.push(candidate);
      }
      // add node to visited and remove from to visit
      nodes_visited_.emplace(cur_key, cell_ptr);
      nodes_to_visit_.erase(cur_key);
    }

    if (path.size() > 0) {
      std::vector<Point2i> inverted_path;
      inverted_path.reserve(path.size());
      for (int i = path.size() - 1; i >= 0; i--) {
        inverted_path.emplace_back(path[i]);
      }
      path = inverted_path;
    }
    return path;
  }
};

#endif  // GRAPH_SEARCHER_HPP_
