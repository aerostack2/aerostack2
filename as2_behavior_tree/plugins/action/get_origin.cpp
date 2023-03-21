/*!*******************************************************************************************
 *  \file       get_origin_service.cpp
 *  \brief      Get Origin service implementation as behavior tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Javier Melero Deza
 *
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

#include "as2_behavior_tree/action/get_origin.hpp"

namespace as2_behavior_tree {
GetOrigin::GetOrigin(const std::string &xml_tag_name,
                     const BT::NodeConfiguration &conf)
    : nav2_behavior_tree::BtServiceNode<as2_msgs::srv::GetOrigin>(xml_tag_name,
                                                                  conf) {}

void GetOrigin::on_tick() {

  this->request_->structure_needs_at_least_one_member = 0;
}

BT::NodeStatus GetOrigin::on_completion() {
  setOutput("latitude", (float)(this->future_result_.get()->origin.latitude));
  setOutput("longitude", (float)(this->future_result_.get()->origin.longitude));
  setOutput("altitude", (float)(this->future_result_.get()->origin.altitude));
  return this->future_result_.get()->success ? BT::NodeStatus::SUCCESS
                                             : BT::NodeStatus::FAILURE;
}
} // namespace as2_behavior_tree
