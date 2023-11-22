// Copyright 2023 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       core_functions.cpp
 *  \brief      Aerostack2 core functions implementation file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "core_functions.hpp"

namespace as2
{
void spinLoop(std::shared_ptr<as2::Node> node, std::function<void()> run_function)
{
  node->configure();
  node->activate();

  if (node->get_loop_frequency() <= 0) {
    rclcpp::spin(node->get_node_base_interface());
    return;
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    if (run_function != nullptr) {
      run_function();
    }
    if (!node->sleep()) {
      // TODO(miferco97): fix this
      // if sleep returns false, it means that loop rate cannot keep up with the desired rate
      // RCLCPP_INFO(
      //   node->get_logger(),
      //   "Spin loop rate exceeded, stable frequency [%.3f Hz] cannot be assured ",
      //   node->get_loop_frequency());
    }
  }
  // TODO(miferco97): improve this
  node->deactivate();
  node->cleanup();
  node->shutdown();
}

}  // namespace as2
