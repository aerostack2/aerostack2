// Copyright 2025 Universidad Politécnica de Madrid
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

/**
* @file simple_example.cpp
*
* EKF example
*
* @authors Rodrigo Da Silva Gómez
*/

#include "ekf/ekf_wrapper.hpp"
#include "ekf/ekf_datatype.hpp"

int main(int argc, char ** argv)
{
  // Create an instance of the EKFWrapper
  ekf::EKFWrapper ekf_wrapper;

  ekf::State initial_state;
  // Set the initial state to Zero
  std::array<double, ekf::State::size> initial_state_values = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0};

  ekf::Covariance initial_covariance;
  // Set the initial covariance to Identity
  std::array<double, ekf::Covariance::size> initial_covariance_values;
  for (std::size_t i = 0; i < ekf::Covariance::size; ++i) {
    initial_covariance_values[i] = (i % 16 == 0) ? 0.0 : 0.0; // Identity matrix
  }
  initial_covariance_values[ekf::Covariance::ABX] = 1e-1;
  initial_covariance_values[ekf::Covariance::ABY] = 1e-1;
  initial_covariance_values[ekf::Covariance::ABZ] = 1e-1;
  initial_covariance_values[ekf::Covariance::WBX] = 1e-1;
  initial_covariance_values[ekf::Covariance::WBY] = 1e-1;
  initial_covariance_values[ekf::Covariance::WBZ] = 1e-1;

  ekf_wrapper.reset(
    ekf::State(initial_state_values),
    ekf::Covariance(initial_covariance_values));

  // Print the initial state and Covariance
  std::cout << "Initial State: \n" << ekf_wrapper.get_state().to_string() << std::endl;
  std::cout << "Initial Covariance: \n" << ekf_wrapper.get_state_covariance().to_string() <<
    std::endl;

  // Set gravity Vector
  ekf::Gravity gravity = ekf::Gravity(std::array<double, ekf::Gravity::size>({0.00, 0.0, 9.81}));
  ekf_wrapper.set_gravity(gravity);

  // IMU noise parameters
  Eigen::Vector<double, 6> imu_noise;
  imu_noise << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  double accelerometer_noise_density = 1e-2;
  double gyroscope_noise_density = 1e-4;
  double accelerometer_random_walk = 1e-4;
  double gyroscope_random_walk = 1e-6;
  ekf_wrapper.set_noise_parameters(
    imu_noise,
    accelerometer_noise_density,
    gyroscope_noise_density,
    accelerometer_random_walk,
    gyroscope_random_walk);

  std::cout << "IMU noise parameters set." << std::endl;
  std::cout << "Accelerometer noise density: " << ekf_wrapper.get_noise_parameters()[0] <<
    std::endl;
  std::cout << "Gyroscope noise density: " << ekf_wrapper.get_noise_parameters()[1] << std::endl;
  std::cout << "Accelerometer random walk: " << ekf_wrapper.get_noise_parameters()[2] << std::endl;
  std::cout << "Gyroscope random walk: " << ekf_wrapper.get_noise_parameters()[3] << std::endl;

  // Create an IMU input measurement
  ekf::Input imu_input;
  std::array<double, ekf::Input::size> imu_values = {0.0, 0.0, 9.81, 0.0, 0.0, 1.0};
  imu_input.set(imu_values);

  ekf::State prev_state = ekf_wrapper.get_state();
  ekf::Covariance prev_covariance = ekf_wrapper.get_state_covariance();
  std::cout << "Previous State: \n" << prev_state.to_string() << std::endl;
  std::cout << "Previous Covariance: \n" << prev_covariance.to_string() << std::endl;

  // Predict the next state with a time step of 1 seconds
  double seconds = 1.0;
  double dt = 1.0 / 200.0;
  int steps = seconds / dt;
  std::cout << "Predicting for " << steps << " steps with dt = " << dt << ". Total of " <<
    seconds << " seconds." << std::endl;
  ekf_wrapper.predict(imu_input, dt);
  std::cout << "State: \n" << ekf_wrapper.get_state().to_string() << std::endl;
  std::cout << "Covariance: \n" << ekf_wrapper.get_state_covariance().to_string() << std::endl;
  for (int i = 0; i < steps - 1; ++i) {
    // std::cout << "Step " << i + 1 << " of " << steps << std::endl;
    ekf_wrapper.predict(imu_input, dt);
    // Print state and covariance
    // std::cout << "State: \n" << ekf_wrapper.get_state().to_string() << std::endl;
    // std::cout << "Covariance: \n" << ekf_wrapper.get_state_covariance().to_string() << std::endl;
  }

  ekf::Covariance process_noise_covariance =
    ekf_wrapper.compute_process_noise_covariance(dt);
  std::cout << "Process Noise Covariance: \n" << process_noise_covariance.to_string() << std::endl;

  // Print the updated state and Covariance
  std::cout << "Updated State: \n" << ekf_wrapper.get_state().to_string() << std::endl;
  std::cout << "Updated Covariance: \n" << ekf_wrapper.get_state_covariance().to_string() <<
    std::endl;

  // Test pose Update
  ekf::PoseMeasurement pose_measurement;
  std::array<double, ekf::PoseMeasurement::size> pose_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pose_measurement.set(pose_values);

  ekf::PoseMeasurementCovariance pose_measurement_covariance;
  std::array<double, ekf::PoseMeasurementCovariance::size> pose_covariance_values;
  pose_covariance_values = {3.3e-5, 3.3e-5, 3.3e-5, 1e-8, 1e-8, 1e-8};
  pose_measurement_covariance.set(pose_covariance_values);
  std::cout << "Pose Measurement: \n" << pose_measurement.to_string() << std::endl;
  std::cout << "Pose Measurement Covariance: \n" << pose_measurement_covariance.to_string() <<
    std::endl;

  ekf::State state;
  ekf::Covariance covariance;
  Eigen::Matrix4d map_to_odom;

  state = ekf_wrapper.get_state();
  covariance = ekf_wrapper.get_state_covariance();
  map_to_odom = ekf_wrapper.get_map_to_odom();

  // Print the map to odom transformation
  std::cout << "Map to Odom Transformation: \n" << map_to_odom << std::endl;

  // Print the odom to base transformation
  Eigen::Matrix4d odom_to_base = ekf_wrapper.get_T_b_c(
    Eigen::Vector3d(state.get_position().data()),
    Eigen::Vector3d(state.get_orientation().data()),
    map_to_odom);

  std::cout << "Odom to Base Transformation: \n" << odom_to_base << std::endl;


  // Update the state with the pose measurement
  // Update and predict in a loop
  for (int i = 0; i < 100; ++i) {
    ekf_wrapper.predict(imu_input, dt);
    ekf_wrapper.predict(imu_input, dt);
    ekf_wrapper.predict(imu_input, dt);
    ekf_wrapper.update_pose(pose_measurement, pose_measurement_covariance);
  }


  state = ekf_wrapper.get_state();
  covariance = ekf_wrapper.get_state_covariance();
  map_to_odom = ekf_wrapper.get_map_to_odom();

  std::cout << "State after pose update: \n" << state.to_string() << std::endl;
  std::cout << "Covariance after pose update: \n" <<
    covariance.to_string() << std::endl;

  // Print the map to odom transformation
  std::cout << "Map to Odom Transformation: \n" << map_to_odom << std::endl;

  // Print the odom to base transformation
  odom_to_base = ekf_wrapper.get_T_b_c(
    Eigen::Vector3d(state.get_position().data()),
    Eigen::Vector3d(state.get_orientation().data()),
    map_to_odom);

  std::cout << "Odom to Base Transformation: \n" << odom_to_base << std::endl;

  // Reset the state to zero
  ekf_wrapper.reset(
    ekf::State(initial_state_values),
    ekf::Covariance(initial_covariance_values));

  // Turn 90 degrees in yaw in 1 second
  imu_values = {0.0, 0.0, 9.81, 0.0, 0.0, M_PI / 2.0};
  imu_input.set(imu_values);
  seconds = 1.0;
  dt = 1.0 / 200.0;
  steps = seconds / dt;
  std::cout << "Predicting for " << steps << " steps with dt = " << dt << ". Total of " <<
    seconds << " seconds." << std::endl;
  for (int i = 0; i < steps; ++i) {
    ekf_wrapper.predict(imu_input, dt);
  }
  state = ekf_wrapper.get_state();
  covariance = ekf_wrapper.get_state_covariance();
  map_to_odom = ekf_wrapper.get_map_to_odom();
  std::cout << "State after yaw rotation: \n" << state.to_string() << std::endl;
  std::cout << "Covariance after yaw rotation: \n" <<
    covariance.to_string() << std::endl;
  // Print the map to odom transformation
  std::cout << "Map to Odom Transformation: \n" << map_to_odom << std::endl;

  // Accelerate forward for 1 second
  imu_values = {1.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  imu_input.set(imu_values);
  seconds = 1.0;
  dt = 1.0 / 200.0;
  steps = seconds / dt;
  std::cout << "Predicting for " << steps << " steps with dt = " << dt << ". Total of " <<
    seconds << " seconds." << std::endl;
  for (int i = 0; i < steps; ++i) {
    ekf_wrapper.predict(imu_input, dt);
  }
  state = ekf_wrapper.get_state();
  covariance = ekf_wrapper.get_state_covariance();
  map_to_odom = ekf_wrapper.get_map_to_odom();

  // Stop acceleration
  imu_values = {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};
  imu_input.set(imu_values);
  seconds = 1.0;
  dt = 1.0 / 200.0;
  steps = seconds / dt;
  std::cout << "Predicting for " << steps << " steps with dt = " << dt << ". Total of " <<
    seconds << " seconds." << std::endl;
  for (int i = 0; i < steps; ++i) {
    ekf_wrapper.predict(imu_input, dt);
  }
  state = ekf_wrapper.get_state();
  covariance = ekf_wrapper.get_state_covariance();
  map_to_odom = ekf_wrapper.get_map_to_odom();
  std::cout << "State after acceleration: \n" << state.to_string() << std::endl;
  std::cout << "Covariance after acceleration: \n" <<
    covariance.to_string() << std::endl;
  // Print the map to odom transformation
  std::cout << "Map to Odom Transformation: \n" << map_to_odom << std::endl;

  // // Turn in yaw a full circle in 5 seconds
  // imu_values = {0.0, 0.0, 9.81, 0.0, 0.0, 2.0 * M_PI};
  // imu_input.set(imu_values);
  // seconds = 5.0;
  // dt = 1.0 / 200.0;
  // steps = seconds / dt;
  // std::cout << "Predicting for " << steps << " steps with dt = " << dt << ". Total of " <<
  //   seconds << " seconds." << std::endl;
  // for (int i = 0; i < steps; ++i) {
  //   ekf_wrapper.predict(imu_input, dt);
  //   // Print every second
  //   if ((i + 1) % 200 == 0) {
  //     std::cout << "Step " << i + 1 << " of " << steps << std::endl;
  //     state = ekf_wrapper.get_state();
  //     std::cout << "State: \n" << state.to_string() << std::endl;
  //   }
  // }
  // state = ekf_wrapper.get_state();
  // covariance = ekf_wrapper.get_state_covariance();
  // map_to_odom = ekf_wrapper.get_map_to_odom();

  // std::cout << "State after yaw rotation: \n" << state.to_string() << std::endl;
  // std::cout << "Covariance after yaw rotation: \n" <<
  //   covariance.to_string() << std::endl;
  // // Print the map to odom transformation
  // std::cout << "Map to Odom Transformation: \n" << map_to_odom << std::endl;

  // // Test compute_map_to_odom
  // ekf::State new_state = ekf_wrapper.get_state();
  // Eigen::Matrix4d prev_map_to_odom = Eigen::Matrix4d::Identity();
  // prev_map_to_odom.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // prev_map_to_odom(0, 3) = 0.0;
  // prev_map_to_odom(1, 3) = 0.0;
  // prev_map_to_odom(2, 3) = 0.0;
  // Eigen::Matrix4d new_map_to_odom = ekf::EKFWrapper::compute_map_to_odom(
  //   prev_state, new_state,
  //   prev_map_to_odom);
  // std::cout << "Map to Odom Transformation: \n" << new_map_to_odom << std::endl;

  return 0;
}
