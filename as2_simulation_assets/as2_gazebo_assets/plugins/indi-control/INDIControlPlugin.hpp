// Copyright 2024 Universidad Politécnica de Madrid
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
#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLPLUGIN_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLPLUGIN_HPP_

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>

#include "gz/sim/System.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include "Common.hpp"
#include "IndiController.hpp"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class MulticopterINDIControl
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  using RotorConfiguration = std::vector<multicopter_control::Rotor>;

public:
  MulticopterINDIControl() = default;

public:
  void Configure(
    const Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    EntityComponentManager & _ecm,
    EventManager & _eventMgr) override;

public:
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  /// \brief Callback for ACRO messages
  /// The controller waits for the first ACRO message before publishing any
  /// rotor velocities.
  /// \param[in] _msg ACRO message

private:
  void OnACRO(const msgs::Float_V & _msg);

  /// \brief Callback for enable messages
  /// \param[in] _msg Callback message. If false, the controller sends a zero
  /// rotor velocity command once and gets disabled. If the vehicle is in the
  /// air, disabling the controller will cause it to fall. If true, the
  /// controller becomes enabled and waits for an ACRO message.

private:
  void OnEnable(const msgs::Boolean & _msg);

  /// \brief Publish provided rotor velocities
  /// \param[in] _ecm Mutable reference to the EntityComponentManager
  /// \param[in] _vels Rotor velocities to be published

private:
  void PublishRotorVelocities(
    gz::sim::EntityComponentManager & _ecm,
    const Eigen::VectorXd & _vels);

  /// \brief Get the vehicle inertial from child links and nested models
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \param[in] _entity Model entity to get inertial for

private:
  math::Inertiald VehicleInertial(
    const EntityComponentManager & _ecm,
    Entity _entity);

private:
  Eigen::Matrix<double, 4, 4> compute_mixer_matrix_4D(const RotorConfiguration & motors);

/// \brief Model interface

private:
  Model model {kNullEntity};

/// \brief Link name

private:
  std::string comLinkName;

/// \brief Link Entity

private:
  Entity comLinkEntity;

/// \brief Topic namespace

private:
  std::string robotNamespace;

/// \brief Topic for ACRO commands

private:
  std::string commandSubTopic{"acro_vel"};

/// \brief Topic for enabling commands

private:
  std::string enableSubTopic{"enable"};

/// \brief Gazebo communication node

private:
  transport::Node node;

/// \brief Holds the computed rotor angular velocities

private:
  Eigen::VectorXd rotorVelocities = Eigen::VectorXd::Zero(4);

/// \brief INDI controller, particularized for a quadrotor

private:
  indi_controller::IndiController<double, 4> indiController;

  /// \brief Noise parameters read from SDF

private:
  multicopter_control::NoiseParameters noiseParameters;

/// \brief Current ACRO command. This is the reference
/// the controller will try to maintain.

private:
  std::optional<msgs::Float_V> acroVelMsg;

/// \brief Maximum commanded angular velocity

private:
  math::Vector3d maximumAngularVelocity;

/// \brief Mutex for acroVelMsg

private:
  std::mutex acroVelMsgMutex;

/// \brief Rotor velocities message

private:
  msgs::Actuators rotorVelocitiesMsg;

/// \brief Becomes true when the system is done initializing

private:
  bool initialized{false};

/// \brief True as long as the controller is active

private:
  std::atomic<bool> controllerActive{true};
};
}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__INDI_CONTROL__INDICONTROLPLUGIN_HPP_
