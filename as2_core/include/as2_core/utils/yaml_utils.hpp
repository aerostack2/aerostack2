/*!*******************************************************************************************
 *  \file       yaml_utils.hpp
 *  \brief      Utils for loading YAML files in Aerostack2.
 *  \authors    Miguel Fernández Cortizas
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

#ifndef __AS2_YAML_UTILS_HPP__
#define __AS2_YAML_UTILS_HPP__

#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace as2 {
namespace yaml {

std::filesystem::path get_project_export_path_from_xml_path(const std::filesystem::path &xml_path);
std::vector<std::filesystem::path> find_yaml_files_inside(const std::filesystem::path &dir);

YAML::Node find_tag_across_multiple_yaml_files(const std::vector<std::filesystem::path> &yaml_files,
                                               const std::string &tag);

uint8_t parse_uint_from_string(const std::string &str);
std::vector<uint8_t> parse_uint_from_string(const std::vector<std::string> &str_v);

// Recursive function to find the tag inside the yaml file
YAML::Node find_tag_in_yaml_node(const YAML::Node &node, const std::string &tag);

template <typename T = std::string>
std::vector<T> find_tag_in_yaml_file(const std::filesystem::path &yaml_file,
                                     const std::string &tag) {
  std::vector<T> result;
  std::ifstream config_file(yaml_file);
  // check if file exists
  if (!config_file.good()) {
    throw std::runtime_error("Could not open yaml file: " + yaml_file.string());
  }
  YAML::Node node = YAML::LoadFile(yaml_file.string());
  if (!node.IsNull()) {
    YAML::Node tag_node = find_tag_in_yaml_node(node, tag);
    if (!tag_node.IsNull()) {
      for (const auto &tag_value : tag_node) {
        result.push_back(tag_value.as<T>());
      }
    }
  } else {
    throw std::runtime_error("Could not parse yaml file: " + yaml_file.string());
  }

  return result;
};

template <typename T = std::string>
std::vector<T> find_tag_from_project_exports_path(const std::filesystem::path &project_exports_path,
                                                  const std::string &tag) {
  std::vector<T> values;
  std::vector<std::filesystem::path> yaml_files = find_yaml_files_inside(project_exports_path);
  if (yaml_files.size() == 0) {
    throw std::runtime_error("Could not find any yaml files in: " + project_exports_path.string());
    return std::vector<T>();
  }

  for (const auto &yaml_file : yaml_files) {
    auto tags = find_tag_in_yaml_file<T>(yaml_file, tag);
    if (tags.size() > 0) {
      values = tags;
    }
  }
  return values;
}

}  // namespace yaml
}  // namespace as2

#endif  // __AS2_YAML_UTILS_HPP__
