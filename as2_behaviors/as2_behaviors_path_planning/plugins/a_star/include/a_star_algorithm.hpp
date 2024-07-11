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
 *  \file       a_star_algorithm.hpp
 *  \brief      A* algorithm for path planning.
 *  \authors    Miguel Fernandez-Cortizas, Pedro Arias Pérez
 ********************************************************************************/

#ifndef A_STAR_ALGORITHM_HPP_
#define A_STAR_ALGORITHM_HPP_

#include <math.h>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// Std libraries

static int convertPointToInt(const cv::Point2i & point)
{
  return point.x * 1000 + point.y;
}

class Node;
using NodePtr = std::shared_ptr<Node>;

class Node
{
  double g_cost_;
  double h_cost_;
  double f_cost_;

  cv::Point2i coordinates_;
  NodePtr parent_ptr_;

public:
  Node(
    const cv::Point2i & coords, const NodePtr & parent_ptr,
    const cv::Point2i goal)
  : coordinates_(coords), parent_ptr_(parent_ptr)
  {
    computeCosts(goal);
    // std::cout << "new node created: "<< coords << std::endl;
  }
  cv::Point2i get_coordinates() const {return coordinates_;}
  NodePtr get_parent() const {return parent_ptr_;}
  double get_g_cost() const {return g_cost_;}
  double get_h_cost() const {return h_cost_;}
  double get_f_cost() const {return f_cost_;}

  operator int() const {return convertPointToInt(coordinates_);}

  double computeCosts(const cv::Point2i & goal)
  {
    const cv::Point2i & point = get_coordinates();
    h_cost_ = std::sqrt(
      std::pow(point.x - goal.x, 2) +
      std::pow(point.y - goal.y, 2));
    if (parent_ptr_ == nullptr) {
      g_cost_ = 0;
    } else {
      g_cost_ = parent_ptr_->get_g_cost() + 1;
      if (!(parent_ptr_->get_coordinates().x == point.x ||
        parent_ptr_->get_coordinates().y == point.y))
      {
        // g_cost_ += 1;
        g_cost_ += 0;
      }
    }
    f_cost_ = g_cost_ + h_cost_;
    return f_cost_;
  }
};

class AStarPlanner
{
private:
  cv::Mat ocuppancy_grid_;
  cv::Point2i goal_;
  cv::Point2i origin_point_;

  // using hash tables for speed
  std::unordered_map<int, NodePtr> nodes_visited_;
  std::unordered_map<int, NodePtr> nodes_to_visit_;
  std::vector<cv::Point2i> valid_movements_;

  // Pseudocode A* algorithm

public:
  AStarPlanner()
  {
    valid_movements_.clear();
    // valid_movements_.reserve(4);
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

  void setOcuppancyGrid(const cv::Mat & mat) {ocuppancy_grid_ = mat.clone();}
  void setOriginPoint(const cv::Point2i & point) {origin_point_ = point;}
  void setGoal(const cv::Point2i & point) {goal_ = point;}

private:
  void addNeighborsToVisit(const NodePtr & node)
  {
    for (auto & movement : valid_movements_) {
      cv::Point2i new_node_position = node->get_coordinates() + movement;
      // Check if the new node is in the map limits
      if (new_node_position.x > ocuppancy_grid_.rows - 1 ||
        new_node_position.y > ocuppancy_grid_.cols - 1 ||
        (new_node_position.x < 0 || new_node_position.y < 0))
      {
        // std::cout << new_node_position <<  "OUT_OF_LIMITS" << std::endl;
        continue;
      }

      // Check if the node is occuped
      if (ocuppancy_grid_.at<uchar>(new_node_position.x, new_node_position.y) ==
        0)
      {
        // std::cout << new_node_position << "OCUPPIED" << std::endl;
        continue;
      }

      // Check if the node is already visited
      if (nodes_visited_.find(convertPointToInt(new_node_position)) !=
        nodes_visited_.end())
      {
        // std::cout << new_node_position <<  "ALREADY VISITED" << std::endl;
        continue;
      }
      // Check if the node is in nodes_to_visit_ vector if not add to the list
      if (std::find_if(
          nodes_to_visit_.begin(), nodes_to_visit_.end(),
          [&new_node_position](std::pair<const int, NodePtr> ptr) {
            return ptr.second->get_coordinates() ==
            new_node_position;
          }) != nodes_to_visit_.end())
      {
        // std::cout << new_node_position <<  "ALREADY IN VISIT LIST" <<
        // std::endl;
        continue;
      }
      // if all tests are passed
      nodes_to_visit_.emplace(
        convertPointToInt(new_node_position),
        std::make_shared<Node>(new_node_position, node, goal_));
    }
  }

  NodePtr findNextNodeToVisit()
  {
    // look the less cost node
    NodePtr node_ptr = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();
    for (auto & node : nodes_to_visit_) {
      if (node.second->get_f_cost() < min_cost) {
        node_ptr = node.second;
        min_cost = node.second->get_f_cost();
      }
    }
    return node_ptr;
  }

public:
  std::vector<cv::Point2i> solveGraph()
  {
    std::vector<cv::Point2i> path;

    nodes_to_visit_.clear();
    nodes_visited_.clear();

    nodes_to_visit_.emplace(
      convertPointToInt(origin_point_),
      std::make_shared<Node>(origin_point_, nullptr, goal_));

    while (nodes_to_visit_.size() > 0) {
      auto new_node = findNextNodeToVisit();
      if (new_node == nullptr) {
        throw std::runtime_error("node without ptr");
      }

      // if goal is finded
      if (new_node->get_coordinates().x == goal_.x &&
        new_node->get_coordinates().y == goal_.y)
      {
        // std::cout << "PATH GENERATED" << std::endl;
        NodePtr parent_ptr = new_node;
        do {
          path.emplace_back(parent_ptr->get_coordinates());
          // std::cout << parent_ptr->get_coordinates() << std::endl;
          parent_ptr = parent_ptr->get_parent();
        } while (parent_ptr != nullptr);
        break;
      }
      // if is not goal
      addNeighborsToVisit(new_node);
      nodes_visited_.emplace(*new_node, new_node);
      nodes_to_visit_.erase(*new_node);
    }
    if (path.size() > 0) {
      std::vector<cv::Point2i> inverted_path;
      inverted_path.reserve(path.size());
      for (int i = path.size() - 1; i >= 0; i--) {
        inverted_path.emplace_back(path[i]);
      }
      return inverted_path;
    }
    return path;
  }
};

#endif  // A_STAR_ALGORITHM_HPP_
