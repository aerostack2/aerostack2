// Copyright 2026 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names
//    of its
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
 * @file generate_polynomial_trajectory_base.cpp
 *
 * @brief Minimal non-virtual implementation for the plugin base class.
 *
 * @authors Rafael Perez-Segui
 */

#include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

namespace generate_polynomial_trajectory_behavior_plugin_base
{

void GeneratePolynomialTrajectoryBase::initialize(
  as2::Node * node, const std::string & plugin_name)
{
  // Cache execution context before delegating plugin-specific setup.
  node_ptr_ = node;
  plugin_name_ = plugin_name;
  ownInitialize();
}

void GeneratePolynomialTrajectoryBase::setVehicleState(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::TwistStamped & twist)
{
  vehicle_pose_ = pose;
  vehicle_twist_ = twist;
}

as2_msgs::msg::PoseStampedWithID
GeneratePolynomialTrajectoryBase::buildCurrentWaypoint() const
{
  as2_msgs::msg::PoseStampedWithID current;
  current.id = "current";
  current.pose = vehicle_pose_;
  return current;
}

std::string GeneratePolynomialTrajectoryBase::qualifyParameterName(
  const std::string & param_name) const
{
  if (plugin_name_.empty()) {
    return param_name;
  }

  // Keep explicit fully-qualified names unchanged.
  const std::string prefix = plugin_name_ + ".";
  if (param_name.rfind(prefix, 0) == 0) {
    return param_name;
  }
  return prefix + param_name;
}

}  // namespace generate_polynomial_trajectory_behavior_plugin_base
