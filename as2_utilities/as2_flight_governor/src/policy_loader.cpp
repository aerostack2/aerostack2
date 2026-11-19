// Copyright 2026 Vishwa
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
//    * Neither the name of the copyright holder nor the names of its
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

/**
* @file policy_loader.cpp
*
* Builds flight governor policies from YAML.
*
* @author Vishwa
*/

#include "as2_flight_governor/policy_loader.hpp"

#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "as2_flight_governor/policies.hpp"

namespace as2_flight_governor
{

namespace
{

class Fields
{
public:
  Fields(const YAML::Node & node, std::string where)
  : node_(node), where_(std::move(where)) {}

  double number(const std::string & key)
  {
    used_.insert(key);
    const YAML::Node value = node_[key];
    if (!value) {
      fail("missing required field '" + key + "'");
    }
    return toDouble(value, key);
  }

  double number(const std::string & key, double fallback)
  {
    used_.insert(key);
    const YAML::Node value = node_[key];
    return value ? toDouble(value, key) : fallback;
  }

  Vec3 vec3(const std::string & key)
  {
    used_.insert(key);
    const YAML::Node value = node_[key];
    if (!value) {
      fail("missing required field '" + key + "'");
    }
    if (!value.IsSequence() || value.size() != 3) {
      fail("field '" + key + "' must be a list of 3 numbers");
    }
    return Vec3{toDouble(value[0], key), toDouble(value[1], key), toDouble(value[2], key)};
  }

  Action action()
  {
    used_.insert("action");
    const YAML::Node value = node_["action"];
    if (!value) {
      return Action::CLAMP;
    }
    const std::string text = value.as<std::string>();
    if (text == "clamp") {return Action::CLAMP;}
    if (text == "veto") {return Action::VETO;}
    fail("field 'action' must be 'clamp' or 'veto', got '" + text + "'");
    return Action::CLAMP;
  }

  /// Reject keys that no policy reads, so typos do not silently disable a limit.
  void rejectUnknown()
  {
    for (const auto & entry : node_) {
      const std::string key = entry.first.as<std::string>();
      if (used_.count(key) == 0) {
        fail("unknown field '" + key + "'");
      }
    }
  }

  [[noreturn]] void fail(const std::string & message) const
  {
    throw std::runtime_error("flight governor policy " + where_ + ": " + message);
  }

private:
  double toDouble(const YAML::Node & value, const std::string & key) const
  {
    try {
      return value.as<double>();
    } catch (const YAML::Exception &) {
      fail("field '" + key + "' must be a number");
    }
  }

  YAML::Node node_;
  std::string where_;
  std::set<std::string> used_{"name", "type"};
};

PolicyPtr buildPolicy(const YAML::Node & node, std::size_t index)
{
  if (!node.IsMap()) {
    throw std::runtime_error(
            "flight governor policy #" + std::to_string(index) + ": must be a map");
  }
  if (!node["name"] || !node["type"]) {
    throw std::runtime_error(
            "flight governor policy #" + std::to_string(index) +
            ": 'name' and 'type' are required");
  }
  const std::string name = node["name"].as<std::string>();
  const std::string type = node["type"].as<std::string>();
  Fields f(node, "'" + name + "'");

  PolicyPtr policy;
  try {
    if (type == "geofence_box") {
      const Vec3 min = f.vec3("min");
      const Vec3 max = f.vec3("max");
      const double lookahead = f.number("lookahead", 1.0);
      const Action action = f.action();
      policy = std::make_shared<GeofenceBoxPolicy>(name, min, max, lookahead, action);
    } else if (type == "max_speed") {
      const double horizontal = f.number("horizontal", -1.0);
      const double vertical = f.number("vertical", -1.0);
      policy = std::make_shared<MaxSpeedPolicy>(name, horizontal, vertical);
    } else if (type == "keepout_sphere") {
      const Vec3 center = f.vec3("center");
      const double radius = f.number("radius");
      const double margin = f.number("margin", 0.0);
      const double lookahead = f.number("lookahead", 1.0);
      const Action action = f.action();
      policy = std::make_shared<KeepoutSpherePolicy>(
        name, center, radius, margin, lookahead, action);
    } else if (type == "speed_zone") {
      const Vec3 center = f.vec3("center");
      const double radius = f.number("radius");
      const double max_speed = f.number("max_speed");
      policy = std::make_shared<SpeedZonePolicy>(name, center, radius, max_speed);
    } else {
      f.fail("unknown type '" + type + "'");
    }
  } catch (const std::invalid_argument & e) {
    f.fail(e.what());
  }
  f.rejectUnknown();
  return policy;
}

}  // namespace

std::vector<PolicyPtr> loadPolicies(const YAML::Node & root)
{
  const YAML::Node list = root["policies"];
  if (!list || !list.IsSequence()) {
    throw std::runtime_error("flight governor: the file must contain a 'policies' list");
  }

  std::vector<PolicyPtr> policies;
  std::set<std::string> names;
  for (std::size_t i = 0; i < list.size(); ++i) {
    PolicyPtr policy = buildPolicy(list[i], i);
    if (!names.insert(policy->name()).second) {
      throw std::runtime_error(
              "flight governor policy '" + policy->name() + "': duplicate name");
    }
    policies.push_back(policy);
  }
  return policies;
}

std::vector<PolicyPtr> loadPoliciesFromFile(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
            "flight governor: cannot read policies file '" + path + "': " + e.what());
  }
  return loadPolicies(root);
}

}  // namespace as2_flight_governor
