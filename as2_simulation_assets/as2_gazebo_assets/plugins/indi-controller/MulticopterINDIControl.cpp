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

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/twist.pb.h>

#include <limits>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Inertial.hh>
#include <gz/math/Vector3.hh>

#include <gz/math/eigen3/Conversions.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include "MulticopterINDIControl.hpp"


using namespace gz;
using namespace sim;
using namespace systems;
using namespace multicopter_control;

void MulticopterINDIControl::Configure(
  const Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  EntityComponentManager & _ecm,
  EventManager & /*_eventMgr*/)
{

  this->model = Model(_entity);

  if (!this->model.Valid(_ecm)) {
    gzerr << "MulticopterINDIControl should be attached to a model "
          << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement("comLinkName")) {
    this->comLinkName = sdfClone->Get<std::string>("comLinkName");
  }

  if (this->comLinkName.empty()) {
    gzerr << "found an empty comLinkName parameter. Failed to initialize.\n";
    return;
  }

  this->comLinkEntity = this->model.LinkByName(_ecm, this->comLinkName);

  if (this->comLinkEntity == kNullEntity) {
    gzerr << "Link " << this->comLinkName
          << " could not be found. Failed to initialize.\n";
    return;
  }

  createFrameDataComponents(_ecm, this->comLinkEntity);

  VehicleParameters vehicleParams;

  math::Inertiald vehicleInertial;
  vehicleInertial = this->VehicleInertial(_ecm, this->model.Entity());

  vehicleParams.mass = vehicleInertial.MassMatrix().Mass();
  vehicleParams.inertia = math::eigen3::convert(vehicleInertial.Moi());
  if (sdfClone->HasElement("rotorConfiguration")) {
    vehicleParams.rotorConfiguration =
      loadRotorConfiguration(
      _ecm, sdfClone->GetElement(
        "rotorConfiguration"), this->model, this->comLinkEntity);
  } else {
    gzerr << "Please specify rotorConfiguration.\n";
  }

  this->rotorVelocities.resize(vehicleParams.rotorConfiguration.size());

  auto worldEntity = _ecm.EntityByComponents(components::World());

  if (kNullEntity == worldEntity) {
    gzerr << "World entity missing." << std::endl;
    return;
  }

  auto gravityComp = _ecm.Component<components::Gravity>(worldEntity);

  if (nullptr == gravityComp) {
    gzerr << "World missing gravity." << std::endl;
    return;
  }

  vehicleParams.gravity = math::eigen3::convert(gravityComp->Data());

  pid_controller::PIDParams pidParams;

  if (sdfClone->HasElement("Kp_gains")) {
    pidParams.Kp_gains = math::eigen3::convert(sdfClone->Get<math::Vector3d>("Kp_gains"));
  } else {
    gzerr << "Please specify proportional gains for INDI controller.\n";
    return;
  }

  if (sdfClone->HasElement("Ki_gains")) {
    pidParams.Ki_gains = math::eigen3::convert(sdfClone->Get<math::Vector3d>("Ki_gains"));
  } else {
    gzerr << "Please specify integral gains for INDI controller.\n";
    return;
  }

  if (sdfClone->HasElement("Kd_gains")) {
    pidParams.Kd_gains = math::eigen3::convert(sdfClone->Get<math::Vector3d>("Kd_gains"));
  } else {
    gzerr << "Please specify derivative gains for INDI controller.\n";
  }

  if (sdfClone->HasElement("antiwindup_cte")) {
    pidParams.antiwindup_cte =
      math::eigen3::convert(sdfClone->Get<math::Vector3d>("antiwindup_cte"));
  }

  if (sdfClone->HasElement("alpha")) {
    pidParams.alpha = math::eigen3::convert(sdfClone->Get<math::Vector3d>("alpha"));
  }

  if (sdfClone->HasElement("reset_integral_flag")) {
    pidParams.reset_integral_flag = sdfClone->Get<bool>("reset_integral_flag");
  }

  if (sdfClone->HasElement("proportional_saturation_flag")) {
    pidParams.proportional_saturation_flag = sdfClone->Get<bool>("proportional_saturation_flag");
    if (pidParams.proportional_saturation_flag) {
      if (sdfClone->HasElement("upper_output_saturation")) {
        pidParams.upper_output_saturation =
          math::eigen3::convert(sdfClone->Get<math::Vector3d>("upper_output_saturation"));
      } else {
        gzerr << "Output saturation requires specifying upper and lower saturation limits.\n";
        return;
      }

      if (sdfClone->HasElement("lower_output_saturation")) {
        pidParams.upper_output_saturation =
          math::eigen3::convert(sdfClone->Get<math::Vector3d>("lower_output_saturation"));
      } else {
        gzerr << "Output saturation requires specifying upper and lower saturation limits.\n";
        return;
      }
    }

    // Compute mixer_matrix and mixer_matrix_inverse from multicopter_simulator

    Eigen::Matrix<double, 6, 4> mixer_matrix = compute_mixer_matrix_4D(
      vehicleParams.rotorConfiguration);

    Eigen::Matrix<double, 4, 6> mixer_matrix_inverse =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixer_matrix);

    indiController = indi_controller::IndiController(
      vehicleParams.inertia, mixer_matrix_inverse,
      pidParams);

    if (sdfClone->HasElement("robotNamespace")) {
      this->robotNamespace = transport::TopicUtils::AsValidTopic(
        sdfClone->Get<std::string>("robotNamespace"));
      if (this->robotNamespace.empty()) {
        gzerr << "Robot namespace ["
              << sdfClone->Get<std::string>("robotNamespace") << "] is invalid."
              << std::endl;
        return;
      }
    } else {
      gzerr << "Please specify namespace.\n";
      return;
    }

    sdfClone->Get<std::string>(
      "commandSubTopic",
      this->commandSubTopic, this->commandSubTopic);
    this->commandSubTopic = transport::TopicUtils::AsValidTopic(
      this->commandSubTopic);
    if (this->commandSubTopic.empty()) {
      gzerr << "Invalid command sub-topic." << std::endl;
      return;
    }

    sdfClone->Get<std::string>(
      "enableSubTopic",
      this->enableSubTopic, this->enableSubTopic);
    this->enableSubTopic = transport::TopicUtils::AsValidTopic(
      this->enableSubTopic);
    if (this->enableSubTopic.empty()) {
      gzerr << "Invalid enable sub-topic." << std::endl;
      return;
    }

    // Suscribe to ACRO commands
    std::string topic{this->robotNamespace + "/" + this->commandSubTopic};

    this->node.Subscribe(topic, &MulticopterINDIControl::OnACRO, this);
    gzmsg << "MulticopterINDIControl subscribing to ACRO messages on ["
          << topic << "]" << std::endl;

    std::string enableTopic{this->robotNamespace + "/" + this->enableSubTopic};

    this->node.Subscribe(enableSubTopic, &MulticopterINDIControl::OnEnable, this);
    gzmsg << "MulticopterINDIControl suscribing to Boolean message on ["
          << enableTopic << "]" << std::endl;

    // Create the Actuators component to take control of rotor speeds
    this->rotorVelocitiesMsg.mutable_velocity()->Resize(
      this->rotorVelocities.size(), 0);

    _ecm.CreateComponent(
      this->model.Entity(),
      components::Actuators(this->rotorVelocitiesMsg));

    this->initialized = true;
  }
}

//////////////////////////////////////////////////
void MulticopterINDIControl::PreUpdate(
  const UpdateInfo & _info,
  EntityComponentManager & _ecm)
{
  GZ_PROFILE("MulticopterINDIControl::PreUpdate");

  if (!this->initialized) {
    return;
  }

  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (_info.paused) {
    return;
  }

  if (!this->controllerActive) {
    if (this->rotorVelocities.squaredNorm() > 0) {
      this->rotorVelocities.setZero();
      this->PublishRotorVelocities(_ecm, this->rotorVelocities);
      std::lock_guard<std::mutex> lock(this->acroVelMsgMutex);
      this->acroVelMsg.reset();
    }
    return;
  }

  msgs::Quaternion acroVel{};
  {
    std::lock_guard<std::mutex> lock(this->acroVelMsgMutex);
    if (!this->acroVelMsg.has_value()) {
      return;
    }

    acroVel.set_x(acroVelMsg->x());
    acroVel.set_y(acroVelMsg->y());
    acroVel.set_z(acroVelMsg->z());
    acroVel.set_w(acroVelMsg->w());

  }

  Eigen::Vector3d rpyRates{acroVel.x(), acroVel.y(),
    acroVel.z()};

  double thrust = acroVel.w();


  std::optional<FrameData> frameData =
    getFrameData(_ecm, this->comLinkEntity, this->noiseParameters);
  if (!frameData.has_value()) {
    // Errors would have already been printed
    return;
  }

  this->rotorVelocities = this->indiController.acro_to_motor_angular_velocity(
    frameData->angularVelocityBody, thrust,
    rpyRates, std::chrono::duration<double>(_info.dt).count());

  this->PublishRotorVelocities(_ecm, this->rotorVelocities);
}

//////////////////////////////////////////////////
void MulticopterINDIControl::OnACRO(const msgs::Quaternion & _msg)
{
  std::lock_guard<std::mutex> lock(this->acroVelMsgMutex);
  this->acroVelMsg = _msg;
}

//////////////////////////////////////////////////
void MulticopterINDIControl::OnEnable(const msgs::Boolean & _msg)
{
  this->controllerActive = _msg.data();
}

//////////////////////////////////////////////////
void MulticopterINDIControl::PublishRotorVelocities(
  EntityComponentManager & _ecm,
  const Eigen::VectorXd & _vels)
{
  if (_vels.size() != this->rotorVelocitiesMsg.velocity_size()) {
    this->rotorVelocitiesMsg.mutable_velocity()->Resize(_vels.size(), 0);
  }
  for (int i = 0; i < this->rotorVelocities.size(); ++i) {
    this->rotorVelocitiesMsg.set_velocity(i, _vels(i));
  }
  // Publish the message by setting the Actuators component on the model entity.
  // This assumes that the MulticopterMotorModel system is attached to this
  // model
  auto actuatorMsgComp =
    _ecm.Component<components::Actuators>(this->model.Entity());

  if (actuatorMsgComp) {
    auto compFunc = [](const msgs::Actuators & _a, const msgs::Actuators & _b)
      {
        return std::equal(
          _a.velocity().begin(), _a.velocity().end(),
          _b.velocity().begin());
      };
    auto state = actuatorMsgComp->SetData(this->rotorVelocitiesMsg, compFunc) ?
      ComponentState::PeriodicChange :
      ComponentState::NoChange;
    _ecm.SetChanged(this->model.Entity(), components::Actuators::typeId, state);
  } else {
    _ecm.CreateComponent(
      this->model.Entity(),
      components::Actuators(this->rotorVelocitiesMsg));
  }
}

//////////////////////////////////////////////////
math::Inertiald MulticopterINDIControl::VehicleInertial(
  const EntityComponentManager & _ecm, Entity _entity)
{
  math::Inertiald vehicleInertial;

  for (const Entity & link :
    _ecm.ChildrenByComponents(_entity, components::Link()))
  {
    auto inertial = _ecm.Component<components::Inertial>(link);
    if (nullptr == inertial) {
      gzerr << "Could not find inertial component on link "
            << this->comLinkName << std::endl;
      return vehicleInertial;
    }
    vehicleInertial += inertial->Data();
  }

  for (const Entity & modelEnt :
    _ecm.ChildrenByComponents(_entity, components::Model()))
  {
    vehicleInertial += this->VehicleInertial(_ecm, modelEnt);
  }
  return vehicleInertial;
}

// compute_mixer_matrix function from multirotor_simulator model.hpp file.
// Variable names are changed to match the members from RotorConfiguration
// struct.
// forceConstant = thrust_coeficient
// momentConstant = torque_coeficient
Eigen::Matrix<double, 6, 4> MulticopterINDIControl::compute_mixer_matrix_4D(
  const RotorConfiguration & motors)
{
  for (auto motor : motors) {
    // assert(motor.pose.rotation == Eigen::Matrix<Precision, 3, 3>::Identity());
  }
  const int num_motors = motors.size();

  // Create mixer matrix
  // [Fz]
  // [Tx] = mixer_matrix * [wn²]
  // [Ty]
  // [Tz]

  // Mixer matrix is a 6xn matrix, being n the number of motors
  Eigen::Matrix<double, 6, 4> mixer_matrix_ = Eigen::Matrix<double, 6, 4>::Zero();

  for (int i = 0; i < num_motors; i++) {
    int motor_rotation_direction = motors[i].direction;    // 1 for CW, -1 for CCW
    assert(motor_rotation_direction == 1 || motor_rotation_direction == -1);

    // Compute the contribution of this motor to the forces and torques in the
    // body frame The contribution is proportional to the motor's thrust and
    // torque coefficients, and it depends on the motor's position relative to
    // the multirotor's body frame. Here, we assume the motors only produce
    // forces along the z-axis (thrust) and torques around the x and y axes on
    // body frame.

    double dx = motors[i].armLength * cos(motors[i].angle);
    double dy = motors[i].armLength * sin(motors[i].angle);

    // Total force and torque by motor in body frame

    // Force in body frame = forceConstant
    mixer_matrix_(0, i) = motors[i].forceConstant;

    // Torque in body frame
    mixer_matrix_(1, i) = dy * motors[i].forceConstant;
    mixer_matrix_(2, i) =
      static_cast<double>(-1) * dx * motors[i].forceConstant;
    mixer_matrix_(3, i) =
      static_cast<double>(motor_rotation_direction) * motors[i].momentConstant;
  }
  return mixer_matrix_;
}

GZ_ADD_PLUGIN(
  MulticopterINDIControl,
  System,
  MulticopterINDIControl::ISystemConfigure,
  MulticopterINDIControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  MulticopterINDIControl,
  "gz::sim::systems::MulticopterINDIControl")
