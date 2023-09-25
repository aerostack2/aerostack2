/*!*******************************************************************************************
 *  \file       core_functions.cpp
 *  \brief      Aerostack2 core functions implementation file.
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

#include "core_functions.hpp"
#include "as2_core/node.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace as2 {

void spinLoop(std::shared_ptr<Node> node, std::function<void()> run_function)
{
  if (node->get_loop_frequency() <= 0) {
    rclcpp::spin(node->get_node_base_interface());
    return;
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    if (run_function != nullptr) run_function();
    if (!node->sleep()) {
      // TODO: fix this
      // if sleep returns false, it means that loop rate cannot keep up with the desired rate
      // RCLCPP_INFO(
      //   node->get_logger(),
      //   "Spin loop rate exceeded, stable frequency [%.3f Hz] cannot be assured ",
      //   node->get_loop_frequency());
    }
  }
}

void spinLoop(std::shared_ptr<AerialPlatform> node, std::function<void()> run_function)
{
  node->configure();
  node->activate();

  if (node->get_loop_frequency() <= 0) {
    rclcpp::spin(node->get_node_base_interface());
    return;
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    if (run_function != nullptr) run_function();
    if (!node->sleep()) {
      // TODO: fix this
      // if sleep returns false, it means that loop rate cannot keep up with the desired rate
      // RCLCPP_INFO(
      //   node->get_logger(),
      //   "Spin loop rate exceeded, stable frequency [%.3f Hz] cannot be assured ",
      //   node->get_loop_frequency());
    }
  }
  // TODO: improve this
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

}  // namespace as2
