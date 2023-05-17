/*!*******************************************************************************************
 *  \file       crazyflie_swarm_launch.cpp
 *  \brief      Runs the crazyflie_platform swarm.
 *  \authors    Miguel Fernández Cortizas
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
#include <as2_core/utils/yaml_utils.hpp>
#include <iostream>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/utilities.hpp>
#include "as2_core/core_functions.hpp"
#include "as2_core/node.hpp"
#include "crazyflie_platform.hpp"

#define SWARM_ARG_NAME "swarm_config_file"
#define PARAMS_ARG_NAME "params-file"
#define URI_ARG_NAME "uri"
#define MEDIUM_FREQ_NODE 75

std::string find_argument_value(const std::string& argument, int argc, char** argv) {
  std::string res = "";
  std::string arg = "--" + argument;
  for (int i = 0; i < argc; i++) {
    if (arg == argv[i]) {
      if (i + 1 < argc) {
        res = argv[i + 1];
        return res;
      }
    }
  }
  return res;
}

YAML::Node traverse_map(const YAML::Node& node, const std::string& key) {
  YAML::Node res;
  if (node[key]) {
    res = node[key];
  } else {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      if (it->second.IsMap()) {
        res = traverse_map(it->second, key);
        if (res) {
          break;
        }
      }
    }
  }
  return res;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::string swarm_config_file = find_argument_value(SWARM_ARG_NAME, argc, argv);
  if (swarm_config_file.empty()) {
    std::string params_file = find_argument_value(PARAMS_ARG_NAME, argc, argv);
    try {
      YAML::Node params     = YAML::LoadFile(params_file);
      YAML::Node swarm_path = traverse_map(params, SWARM_ARG_NAME);
      if (swarm_path) {
        swarm_config_file = swarm_path.as<std::string>();
      }

    } catch (std::exception& e) {
      std::cout << "Error reading file: " << e.what() << std::endl;
      return 1;
    }
  }
  if (swarm_config_file.empty()) {
    std::cout
        << "Swarm config file not found. Please provide it as an argument or in the params file."
        << std::endl;
    return 1;
  }
  std::cout << "Swarm config file: " << swarm_config_file << std::endl;

  YAML::Node swarm_config = YAML::LoadFile(swarm_config_file);
  for (YAML::const_iterator it = swarm_config.begin(); it != swarm_config.end(); ++it) {
    std::string name       = it->first.as<std::string>();
    YAML::Node node_config = it->second;
    if (node_config.IsMap()) {
      for (YAML::const_iterator it2 = node_config.begin(); it2 != node_config.end(); ++it2) {
        std::string key = it2->first.as<std::string>();
        if (key == URI_ARG_NAME) {
          std::string value = it2->second.as<std::string>();
          std::cout << "Creating node " << name << " with uri " << value << std::endl;
          nodes.emplace_back(std::make_shared<CrazyfliePlatform>(name, value));
        }
      }
    }
  }

  /* auto cf_0 = std::make_shared<CrazyfliePlatform>("cf0", "radio://0/80/2M/E7E7E7E700");
  auto cf_1 = std::make_shared<CrazyfliePlatform>("cf1", "radio://0/80/2M/E7E7E7E701");
  auto cf_2 = std::make_shared<CrazyfliePlatform>("cf2", "radio://0/80/2M/E7E7E7E702"); */
  if (nodes.empty()) {
    std::cout << "No nodes created. Exiting." << std::endl;
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::Rate r(int(MEDIUM_FREQ_NODE * nodes.size()));
  while (rclcpp::ok()) {
    for (auto& node : nodes) {
      executor.spin_node_some(node);
      r.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}
