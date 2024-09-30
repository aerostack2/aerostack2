/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_SIM_SYSTEMS_MULTICOPTERINDICONTROL_COMMON_HH_
#define GZ_SIM_SYSTEMS_MULTICOPTERINDICONTROL_COMMON_HH_

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include <sdf/sdf.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"

#include "Parameters.hpp"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
namespace multicopter_control
{
/// \brief Struct containing linear and angular velocities
struct EigenTwist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

/// \brief Frame data of a link including its pose and linear velocity in
/// world frame as well as its angular velocity in body frame
struct FrameData
{
  // Even though this struct contains Eigen objects, None of them are
  // fixed-size vectorizable, so there is no need to override the new operator
  Eigen::Vector3d angularVelocityBody;
};

/// \brief Loads rotor configuration from SDF
/// \param[in] _ecm Immutable reference to the entity component manager
/// \param[in] _sdf Pointer to the SDF element of the system
/// \param[in] _model Model to which the system is attached
/// \param[in] _comLink Link associated with the center of mass.
RotorConfiguration loadRotorConfiguration(
  const EntityComponentManager & _ecm,
  const sdf::ElementPtr & _sdf,
  const Model & _model,
  const Entity & _comLink);

/// \brief Creates components necessary for obtaining the frame data of the
/// given link
/// \param[in] _ecm Mutable reference to the entity component manager
/// \param[in] _link Link on which the components will be created.
void createFrameDataComponents(
  EntityComponentManager & _ecm,
  const Entity & _link);

/// \brief Retrieves the frame data of the given link and applies noise
/// \param[in] _ecm Imutable reference to the entity component manager
/// \param[in] _link Link on which the components will be created.
/// \param[in] _noise Noise parameters
std::optional<FrameData> getFrameData(
  const EntityComponentManager & _ecm,
  const Entity & _link,
  const NoiseParameters & _noise);

}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
