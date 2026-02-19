#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""EKF definition."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import numpy as np
from ekf_definition.ekf import EKF

from ekf_definition.transform_utils import *


class EKFWrapper:
    """
    Wrapper class for the EKF.
    """

    def __init__(self,
                 initial_state: np.ndarray,
                 initial_covariance: np.ndarray,
                 accelerometer_noise_density: float,
                 gyroscope_noise_density: float,
                 accelerometer_random_walk: float,
                 gyroscope_random_walk: float):
        """
        Initialize the EKF.

        :param initial_state (np.ndarray): The initial state vector.
        :param initial_covariance (np.ndarray): The initial covariance.
        :param accelerometer_noise_density (float): The accelerometer noise density.
        :param gyroscope_noise_density (float): The gyroscope noise density.
        :param accelerometer_random_walk (float): The accelerometer random walk.
        :param gyroscope_random_walk (float): The gyroscope random walk.
        """
        self.ekf = EKF()
        self.state = initial_state.reshape((-1, 1))
        self.state_covariance = initial_covariance
        # Initialize map to odom as identity matrix
        self.map_to_odom = np.eye(4)
        self.imu_noise = np.zeros((6, 1))  # IMU noise vector
        self.accelerometer_noise_density = accelerometer_noise_density
        self.gyroscope_noise_density = gyroscope_noise_density
        self.accelerometer_random_walk = accelerometer_random_walk
        self.gyroscope_random_walk = gyroscope_random_walk
        # self.imu_noise = np.array([
        #     # Accelerometer noise
        #     accelerometer_noise_density, accelerometer_noise_density, accelerometer_noise_density,
        #     # Gyroscope noise
        #     gyroscope_noise_density, gyroscope_noise_density, gyroscope_noise_density
        # ])
        # self.process_noise_covariance = np.array([
        #     # Accelerometer noise covariance
        #     accelerometer_noise_density ** 2, accelerometer_noise_density ** 2, accelerometer_noise_density ** 2,
        #     # Gyroscope noise covariance
        #     gyroscope_noise_density ** 2, gyroscope_noise_density ** 2, gyroscope_noise_density ** 2
        # ])
        # self.random_walk = np.array([
        #     # Accelerometer random walk
        #     accelerometer_random_walk, accelerometer_random_walk, accelerometer_random_walk,
        #     # Gyroscope random walk
        #     gyroscope_random_walk, gyroscope_random_walk, gyroscope_random_walk
        # ])
        # self.random_walk_covariance = np.array([
        #     # Accelerometer random walk covariance
        #     accelerometer_random_walk ** 2, accelerometer_random_walk ** 2, accelerometer_random_walk ** 2,
        #     # Gyroscope random walk covariance
        #     gyroscope_random_walk ** 2, gyroscope_random_walk ** 2, gyroscope_random_walk ** 2
        # ])
        self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity vector in m/s^2

    def reset(self,
              initial_state: np.ndarray,
              initial_covariance: np.ndarray):
        """
        Reset the EKF with a new state and covariance.

        :param initial_state (np.ndarray): The new initial state vector.
        :param initial_covariance (np.ndarray): The new initial covariance.
        """
        self.state = initial_state
        self.state_covariance = initial_covariance

    def get_state(self) -> np.ndarray:
        """
        Get the current state.

        :return: The current state vector.
        """
        # return self.state
        return np.array(self.state, dtype=np.float64)

    def get_state_covariance(self) -> np.ndarray:
        """
        Get the current state covariance.

        :return: The current state covariance matrix.
        """
        return np.array(self.state_covariance, dtype=np.float64)

    def get_map_to_odom(self) -> np.ndarray:
        """
        Get the current map to odom transformation.

        :return: The current map to odom transformation matrix.
        """
        return np.array(self.map_to_odom, dtype=np.float64)

    def compute_process_noise_covariance(self, dt: float) -> np.ndarray:
        """
        Compute the process noise covariance matrix.

        :param dt (float): The time step.
        :return: The process noise covariance matrix.
        """
        q_pp = (self.accelerometer_noise_density **
                2 * dt ** 3) / 3.0 * np.eye(3)
        q_pv = (self.accelerometer_noise_density **
                2 * dt ** 2) / 2.0 * np.eye(3)
        q_vv = (self.accelerometer_noise_density ** 2 * dt) * np.eye(3)
        q_ww = (self.gyroscope_noise_density ** 2 * dt) * np.eye(3)
        q_baba = (self.accelerometer_random_walk ** 2 * dt) * np.eye(3)
        q_bwbw = (self.gyroscope_random_walk ** 2 * dt) * np.eye(3)

        process_noise_covariance = np.zeros((15, 15))
        process_noise_covariance[0:3, 0:3] = q_pp
        process_noise_covariance[0:3, 3:6] = q_pv
        process_noise_covariance[3:6, 0:3] = q_pv
        process_noise_covariance[3:6, 3:6] = q_vv
        process_noise_covariance[6:9, 6:9] = q_ww
        process_noise_covariance[9:12, 9:12] = q_baba
        process_noise_covariance[12:15, 12:15] = q_bwbw
        return process_noise_covariance

    def predict(self,
                imu_measurement: np.ndarray,
                dt: float):
        """
        Predict the next state.

        :param imu_measurement (np.ndarray): The IMU measurement vector.
        :param dt (float): The time step.
        """
        # imu_noise = self.imu_noise + self.random_walk * dt
        # process_noise_covariance = self.process_noise_covariance + \
        #     self.random_walk_covariance * dt
        # print("imu_noise:", imu_noise)
        # print("process_noise_covariance:", process_noise_covariance)
        imu_noise = self.imu_noise
        process_noise_covariance = self.compute_process_noise_covariance(dt)
        X_new, P_new, acc_in_world = self.ekf.predict_function(
            self.state,
            imu_measurement,
            imu_noise,
            dt,
            self.state_covariance,
            process_noise_covariance,
            self.gravity,
        )
        self.state = X_new
        self.state_covariance = P_new

        return acc_in_world

        # return F, L

    def update_pose(self,
                    z: np.ndarray,
                    measurement_noise_covariance: np.ndarray):
        """
        Update the state with a new pose measurement.

        :param z (np.ndarray): The measurement (pose) vector.
        :param measurement_noise_covariance (np.ndarray): The measurement noise covariance matrix.
        """
        X_new, P_new = self.ekf.update_pose_function(
            self.state,
            self.imu_noise,
            z,
            self.state_covariance,
            measurement_noise_covariance,
        )
        self.map_to_odom = compute_map_to_odom(
            self.state,
            X_new,
            self.map_to_odom,
        )
        self.state = X_new
        self.state_covariance = P_new

    def update_pose_velocity(self,
                             z: np.ndarray,
                             measurement_noise_covariance: np.ndarray):
        """
        Update the state with a new pose and velocity measurement.

        :param z (np.ndarray): The measurement (pose + velocity) vector.
        :param measurement_noise_covariance (np.ndarray): The measurement noise covariance matrix.
        """
        X_new, P_new = self.ekf.update_pose_velocity_function(
            self.state,
            self.imu_noise,
            z,
            self.state_covariance,
            measurement_noise_covariance,
        )
        self.map_to_odom = compute_map_to_odom(
            self.state,
            X_new,
            self.map_to_odom,
        )
        self.state = X_new
        self.state_covariance = P_new
