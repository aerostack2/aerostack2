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
 *  \file       yaml_utils.cpp
 *  \brief      Utils for loading YAML files in Aerostack2 implementation
 *  \authors    Miguel Fernández Cortizas
 ********************************************************************************/

#include "as2_core/utils/yaml_utils.hpp"

namespace as2
{
namespace yaml
{
std::filesystem::path get_project_export_path_from_xml_path(const std::filesystem::path & xml_path)
{
  std::filesystem::path config_path = xml_path;
  while (config_path.has_parent_path() && config_path.parent_path() != "/") {
    if (config_path.parent_path().filename() == "share") {
      return config_path;
    }
    config_path = config_path.parent_path();
  }
  return std::filesystem::path();
}

std::vector<std::filesystem::path> find_yaml_files_inside(const std::filesystem::path & dir)
{
  std::vector<std::filesystem::path> files;
  for (auto & p : std::filesystem::recursive_directory_iterator(dir)) {
    if (p.path().extension() == ".yaml") {
      files.push_back(p.path());
    }
  }
  return files;
}

// Recursive function to find the tag inside the yaml file

YAML::Node find_tag_in_yaml_node(const YAML::Node & node, const std::string & tag)
{
  if (node.IsSequence() || node.IsMap()) {
    for (auto & child : node) {
      if (child.first.as<std::string>() == tag) {
        return child.second;
      }
      YAML::Node result = find_tag_in_yaml_node(child.first, tag);
      if (!result.IsNull()) {
        return result;
      }
    }
  }
  return YAML::Node();
}

YAML::Node search_tag_across_multiple_yaml_files(
  const std::vector<std::filesystem::path> & yaml_files, const std::string & tag)
{
  for (const auto & yaml_file : yaml_files) {
    YAML::Node config = YAML::LoadFile(yaml_file.string());
    YAML::Node available_modes_node = as2::yaml::find_tag_in_yaml_node(config, tag);
    if (!(available_modes_node == YAML::Node())) {
      return available_modes_node;
    }
  }
  return YAML::Node();
}

uint8_t parse_uint_from_string(const std::string & str)
{
  uint8_t value;
  // check if string begins with 0b or 0x and remove it
  // if it does not, it is assumed to be decimal
  if (str.substr(0, 2) == "0b") {
    auto string_value = str.substr(2);
    value = stoi(string_value, 0, 2);
  } else if (str.substr(0, 2) == "0x") {
    std::stringstream ss(str.substr(2));
    ss >> std::hex >> value;
  } else {
    std::stringstream ss(str);
    ss >> value;
  }
  return value;
}

std::vector<uint8_t> parse_uint_from_string(const std::vector<std::string> & str_v)
{
  std::vector<uint8_t> values;
  for (const auto & str : str_v) {
    values.emplace_back(parse_uint_from_string(str));
  }
  return values;
}

}  // namespace yaml
}  // namespace as2
