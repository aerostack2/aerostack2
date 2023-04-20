/*!*******************************************************************************************
 *  \file       arm_service.hpp
 *  \brief      Arm and disarm services implementation as behavior tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#ifndef ARM_SERVICE_HPP
#define ARM_SERVICE_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_service_node.hpp"

#include <std_srvs/srv/set_bool.hpp>

namespace as2_behavior_tree {
class ArmService
    : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool> {
public:
  ArmService(const std::string &xml_tag_name,
             const BT::NodeConfiguration &conf);

  void on_tick() override;

  BT::NodeStatus
  on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

class DisarmService
    : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool> {
public:
  DisarmService(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

  void on_tick() override;

  BT::NodeStatus
  on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

} // namespace as2_behavior_tree

#endif // ARM_SERVICE_HPP