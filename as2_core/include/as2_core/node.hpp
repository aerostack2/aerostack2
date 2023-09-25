/*!*******************************************************************************************
 *  \file       node.hpp
 *  \brief      Aerostack2 node header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef AEROSTACK2_NODE_HPP_
#define AEROSTACK2_NODE_HPP_

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/timer.hpp>
#include <string>

#include "as2_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "as2_core/node_with_init.hpp"

namespace as2 {
/**
 * @brief Basic Aerostack2 Node, it heritages all the functionality of an rclcpp::Node
 */

class Node : public RosNode, public NodeWithInit {
public:
  /**
   * @brief Construct a new Node object
   *
   * @param name Node name
   */
  Node(const std::string &name,
       const std::string &ns,
       const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : RosNode(name, ns, options) {
    RCLCPP_INFO(this->get_logger(), "Construct with name [%s] and namespace [%s]", name.c_str(),
                ns.c_str());
    init(this);
  }

  Node(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : RosNode(name, options) {
    RCLCPP_INFO(this->get_logger(), "Construct with name [%s]", name.c_str());
    init(this);
  }
};  // class as2::Node

}  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
