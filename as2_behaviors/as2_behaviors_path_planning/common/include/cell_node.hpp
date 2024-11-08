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
 *  \file       cell_node.hpp
 *  \brief      cell_node header file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#ifndef CELL_NODE_HPP_
#define CELL_NODE_HPP_

#include <memory>

/**
 * @brief Point2i class, an integer 2d point
 */
class Point2i
{
public:
  /**
   * @brief Constructor for Point2i class
   */
  Point2i()
  : x(0), y(0) {}

  /**
   * @brief Constructor for Point2i class
   * @param _x x value
   * @param _y y value
   */
  Point2i(int _x, int _y)
  : x(_x), y(_y) {}

  bool operator==(const Point2i & other)
  {
    return (x == other.x) && (y == other.y);
  }

  bool operator!=(const Point2i & other)
  {
    return !operator==(other);
  }

  int x, y;
};

class CellNode;
using CellNodePtr = std::shared_ptr<CellNode>;

/**
 * @brief CellNode class, a node in a grid
 */
class CellNode
{
  Point2i coordinates_;
  CellNodePtr parent_ptr_;

protected:
  double g_cost_;
  double h_cost_;

public:
  /**
   * @brief Constructor for CellNode class
   * @param coordinates coordinates of the cell
   * @param parent_ptr parent node
   * @param g_cost cost to reach this node
   * @param h_cost heuristic cost to reach the goal
   */
  CellNode(
    const Point2i & coordinates, const CellNodePtr & parent_ptr, double g_cost,
    double h_cost = 0)
  : coordinates_(coordinates), parent_ptr_(parent_ptr)
  {
    h_cost_ = h_cost;
    set_g_cost(g_cost);
  }

  void set_g_cost(double g_cost)
  {
    if (parent_ptr_ == nullptr) {
      g_cost_ = g_cost;
    } else {
      g_cost_ = parent_ptr_->get_g_cost() + g_cost;
    }
  }

  Point2i coordinates() {return coordinates_;}
  int x() {return coordinates_.x;}
  int y() {return coordinates_.y;}
  CellNodePtr parent_ptr() {return parent_ptr_;}

  double get_g_cost() {return g_cost_;}
  double get_h_cost() {return h_cost_;}
  double get_total_cost() {return g_cost_ + h_cost_;}
};

#endif  // CELL_NODE_HPP_
