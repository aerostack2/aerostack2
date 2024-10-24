/*!*******************************************************************************************
 *  \file       indi_controller_gtest.hpp
 *  \brief      Class gtest
 *  \authors    Rafael Pérez Seguí
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

#include "gtest/gtest.h"

#include "multicopter_acro_control/src/LeeACROController.hh"

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

///////////////////////////////////////////////////////////////////
std::unique_ptr<LeeACROController> InstantiateController()
{
  std::unique_ptr<LeeACROController> controller(
    new LeeACROController());

  Rotor rotor_fr{
    //5.2461,       // angle (real)
    5.4978,       // angle (45)
    0.2555,       // armLength
    8.54858e-06,  // forceConstant
    0.016,        // momentConstant
    -1             // direction
  };

  Rotor rotor_bl{
    //2.1045,       // angle (real)
    2.3562,       // angle (45)
    0.2555,       // armLength
    8.54858e-06,  // forceConstant
    0.016,        // momentConstant
    -1             // direction
  };

  Rotor rotor_fl{
    // 1.0371,       // angle (real)
    0.78554,      // angle (45)
    0.2555,       // armLength
    8.54858e-06,  // forceConstant
    0.016,        // momentConstant
    1            // direction
  };

  Rotor rotor_br{
    //4.1787,       // angle (real)
    3.927,        // angle (45)
    0.2555,       // armLength
    8.54858e-06,  // forceConstant
    0.016,        // momentConstant
    1            // direction
  };

  std::vector<Rotor> rotorConfig{rotor_fr, rotor_bl, rotor_fl, rotor_br};

  controller->controllerParameters.angularRateGain = Eigen::Vector3d{0.335, 0.335, 0.2};
  controller->vehicleParameters.mass = 1.53;
  controller->vehicleParameters.inertia = Eigen::Vector3d(0.025, 0.009, 0.033).asDiagonal();
  controller->vehicleParameters.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  controller->vehicleParameters.rotorConfiguration = rotorConfig;

  if (controller->InitializeParameters()) {
    return controller;
  } else {
    return nullptr;
  }
}

//////////////////////////////////////////////////
bool LeeACROController::InitializeParameters()
{
  auto allocationMatrix =
    calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);
  if (!allocationMatrix.has_value()) {
    return false;
  }

  this->normalizedAttitudeGain =
    this->controllerParameters.AttitudeGain.transpose() *
    this->vehicleParameters.inertia.inverse();

  this->normalizedAngularRateGain =
    this->controllerParameters.angularRateGain.transpose() *
    this->vehicleParameters.inertia.inverse();

  Eigen::Matrix4d moi;
  moi.setZero();
  moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
  moi(3, 3) = 1;

  this->angularAccToRotorVelocities.resize(
    this->vehicleParameters.rotorConfiguration.size(), 4);
  const auto & aMat = *allocationMatrix;
  this->angularAccToRotorVelocities =
    aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

  return true;
}

TEST(ACROController, CalculateRotorVelocities_cross) {
  std::unique_ptr<LeeACROController> lee_acro_controller = InstantiateController();

  // Input
  double hover_thrust = 1.53 * 9.81;
  double speed = 1.0;
  ACROCommand acro_command{0.0, 0.0, 0.0, hover_thrust};

  Eigen::Vector3d current_angular_velocities = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector4d output_motor_angular_velocity = Eigen::Vector4d::Zero();

  // Move Up
  acro_command.thrust = 15.0;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[1], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[1], output_motor_angular_velocity[2], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[2], output_motor_angular_velocity[3], 0.2);

  // Move clockwise
  acro_command.yawRate = -speed;
  acro_command.thrust = hover_thrust;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[1], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[2], output_motor_angular_velocity[3], 0.2);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);

  // Move counter-clockwise
  acro_command.yawRate = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[1], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[2], output_motor_angular_velocity[3], 0.2);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);

  // Move forward
  acro_command.yawRate = 0.0;
  acro_command.pitchRate = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[2], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[1], output_motor_angular_velocity[3], 0.2);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move backwards
  acro_command.pitchRate = -speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[2], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[1], output_motor_angular_velocity[3], 0.2);
  EXPECT_GT(output_motor_angular_velocity[0], output_motor_angular_velocity[1]);

  // Move left
  acro_command.pitchRate = 0.0;
  acro_command.rollRate = -speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[3], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[1], output_motor_angular_velocity[2], 0.2);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);

  // Move right
  acro_command.rollRate = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[3], 0.2);
  EXPECT_NEAR(output_motor_angular_velocity[1], output_motor_angular_velocity[2], 0.2);
  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[3]);

  // Move diagonally forward left
  acro_command.rollRate = -speed;
  acro_command.pitchRate = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[1], 0.2);
  EXPECT_LT(output_motor_angular_velocity[2], output_motor_angular_velocity[0]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);

  // Move diagonally forward right
  acro_command.rollRate = speed;
  acro_command.pitchRate = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[2], output_motor_angular_velocity[3], 0.2);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[2], output_motor_angular_velocity[1]);

  // Move diagonally bacwards left
  acro_command.rollRate = -speed;
  acro_command.pitchRate = -speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[2], output_motor_angular_velocity[3], 0.2);
  EXPECT_LT(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
  EXPECT_LT(output_motor_angular_velocity[2], output_motor_angular_velocity[0]);

  // Move diagonally backwards right
  acro_command.rollRate = speed;
  acro_command.pitchRate = -speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);
  EXPECT_NEAR(output_motor_angular_velocity[0], output_motor_angular_velocity[1], 0.2);
  EXPECT_LT(output_motor_angular_velocity[3], output_motor_angular_velocity[0]);
  EXPECT_LT(output_motor_angular_velocity[0], output_motor_angular_velocity[2]);
}

TEST(ACROController, CalculateRotorVelocities_cross_step) {
  std::unique_ptr<LeeACROController> lee_acro_controller = InstantiateController();

  // Input
  double hover_thrust = 1.53 * 9.81;
  double speed = 10.0;
  ACROCommand acro_command{0.0, 0.0, 0.0, hover_thrust};

  Eigen::Vector3d current_angular_velocities = Eigen::Vector3d::Zero();

  // Output
  Eigen::Vector4d output_motor_angular_velocity = Eigen::Vector4d::Zero();
  Eigen::Vector4d prev_output_motor_angular_velocity = Eigen::Vector4d::Zero();

  // Follow rollRate Step reference
  acro_command.rollRate = speed;
  // current_angular_velocities[0] = speed - 0.05;
  // lee_acro_controller->CalculateRotorVelocities(
  //   current_angular_velocities, acro_command,
  //   prev_output_motor_angular_velocity);
  current_angular_velocities[0] = speed;
  lee_acro_controller->CalculateRotorVelocities(
    current_angular_velocities, acro_command,
    output_motor_angular_velocity);

  // EXPECT_EQ(prev_output_motor_angular_velocity[0], output_motor_angular_velocity[0]);
  // EXPECT_EQ(prev_output_motor_angular_velocity[1], output_motor_angular_velocity[1]);
  // EXPECT_EQ(prev_output_motor_angular_velocity[2], output_motor_angular_velocity[2]);
  // EXPECT_EQ(prev_output_motor_angular_velocity[3], output_motor_angular_velocity[3]);

  EXPECT_GT(output_motor_angular_velocity[1], output_motor_angular_velocity[0]);
  EXPECT_EQ(output_motor_angular_velocity[0], output_motor_angular_velocity[3]);
  EXPECT_EQ(output_motor_angular_velocity[1], output_motor_angular_velocity[2]);
}
}

}
}
}
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
