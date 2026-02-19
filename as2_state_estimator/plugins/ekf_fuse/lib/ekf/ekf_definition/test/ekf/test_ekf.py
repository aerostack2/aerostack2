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


from re import A
import unittest

import numpy as np

from ekf_definition.ekf_wrapper import EKFWrapper


class TestEKF_Utils():

    @staticmethod
    def read_imu_for_seconds(ekf_wrapper: EKFWrapper,
                             imu_measurement: np.ndarray,
                             dt: float,
                             seconds: float,
                             verbose: bool = False):
        """
        Read IMU data for a given number of seconds.
        :param imu_measurement (np.ndarray): The IMU measurement.
        :param dt (float): The time step.
        :param seconds (float): The number of seconds to read.
        """
        steps = int(seconds / dt)

        # Predict the state using the IMU Measurement
        for step_n in range(steps):
            if verbose:
                if step_n % int(steps/10) == 0:
                    print('State:')
                    print(ekf_wrapper.get_state()[0:9, 0:1])
                    print('Covariance:')
                    print(ekf_wrapper.get_state_covariance()[0:6, 0:6])
            ekf_wrapper.predict(imu_measurement, dt)
        if verbose:
            print('Final State:')
            print(ekf_wrapper.get_state()[0:9, 0:1])
            print('Final Covariance:')
            print(ekf_wrapper.get_state_covariance()[0:6, 0:6])

    @staticmethod
    def compute_real_linear_movement(
            initial_state: np.ndarray,
            imu_measurement: np.ndarray,
            seconds: float):
        """
        Compute the real movement of the system.
        :param initial_state (np.ndarray): The initial state.
        :param imu_measurement (np.ndarray): The IMU measurement.
        :param seconds (float): The number of seconds to read.
        :return: The computed movement.
        """

        position = initial_state[0:3, 0] + \
            initial_state[3:6, 0] * seconds + \
            0.5 * (imu_measurement[0:3] -
                   np.array([0.0, 0.0, 9.81])) * seconds ** 2

        velocity = initial_state[3:6, 0] + \
            (imu_measurement[0:3] - np.array([0.0, 0.0, 9.81])) * seconds

        movement = np.array([
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ])
        movement = np.atleast_2d(movement).T

        return movement


class TestEKF(unittest.TestCase):
    def setUp(self):
        # Example parameters
        self.initial_state = np.array([
            0.0, 0.0, 0.0,  # Position (x, y, z)
            0.0, 0.0, 0.0,  # Velocity (vx, vy, vz)
            0.0, 0.0, 0.0,  # Orientation (roll, pitch, yaw)
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ])
        self.initial_state = np.atleast_2d(self.initial_state).T
        # print(self.initial_state)
        self.initial_covariance = np.eye(15) * 0.0
        # self.initial_covariance = np.diag([
        #     1e-2, 1e-2, 1e-2,  # Position covariance
        #     1e-3, 1e-3, 1e-3,  # Velocity covariance
        #     1e-4, 1e-4, 1e-4,  # Orientation covariance
        #     1e-4, 1e-4, 1e-4,
        #     1e-6, 1e-6, 1e-6
        # ])

        self.accelerometer_noise_density = 1e-2
        self.accelerometer_random_walk = 1e-4
        self.gyroscope_noise_density = 1e-4
        self.gyroscope_random_walk = 1e-6
        # self.accelerometer_noise_density = 0.0
        # self.accelerometer_random_walk = 0.0
        # self.gyroscope_noise_density = 0.0
        # self.gyroscope_random_walk = 0.0

        self.ekf_wrapper = EKFWrapper(
            self.initial_state,
            self.initial_covariance,
            self.accelerometer_noise_density,
            self.gyroscope_noise_density,
            self.accelerometer_random_walk,
            self.gyroscope_random_walk)

    def test_basics(self):
        """
        Test basic functionality of the EKF.
        """
        print("Testing shapes of state and covariance.")

        self.assertIsNotNone(
            self.ekf_wrapper, "EKFWrapper should be initialized.")
        self.assertEqual(self.ekf_wrapper.state.shape,
                         (15, 1),
                         "State shape should be (15,1).")
        self.assertEqual(self.ekf_wrapper.state_covariance.shape,
                         (15, 15),
                         "State covariance shape should be (15, 15).")
        self.assertEqual(self.ekf_wrapper.imu_noise.shape,
                         (6, 1),
                         "IMU noise shape should be (6,1).")
        # self.assertEqual(self.ekf_wrapper.process_noise_covariance.shape,
        #                  (15,15),
        #                  "Process noise covariance shape should be (6,).")

    def test_wrapper_init(self):
        """
        Test EKF wrapper initialization.
        """
        print("Testing EKF wrapper initialization.")

        self.test_basics()

        np.testing.assert_almost_equal(self.ekf_wrapper.get_state(),
                                       self.initial_state,
                                       err_msg="Initial state should match.",
                                       verbose=True)
        np.testing.assert_almost_equal(self.ekf_wrapper.get_state_covariance(),
                                       self.initial_covariance,
                                       err_msg="Initial covariance should match.",
                                       verbose=True)
        # np.testing.assert_almost_equal(self.ekf_wrapper.random_walk,
        #                                np.array([self.accelerometer_random_walk, self.accelerometer_random_walk, self.accelerometer_random_walk,
        #                                          self.gyroscope_random_walk, self.gyroscope_random_walk, self.gyroscope_random_walk]),
        #                                err_msg="Random walk should match.",
        #                                verbose=True)
        # np.testing.assert_almost_equal(self.ekf_wrapper.process_noise_covariance,
        #                                np.array([self.accelerometer_noise_density ** 2, self.accelerometer_noise_density ** 2, self.accelerometer_noise_density ** 2,
        #                                          self.gyroscope_noise_density ** 2, self.gyroscope_noise_density ** 2, self.gyroscope_noise_density ** 2]),
        #                                err_msg="Process noise covariance should match.",
        #                                verbose=True)

    def test_predict_1(self):
        """
        Test EKF with no movement.
        """
        print("Test 1 - No movement")

        # Reset EKF
        self.ekf_wrapper.reset(self.initial_state,
                               self.initial_covariance)

        # Example IMU Measurement
        imu_measurement = np.array([
            0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
            0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
        ])
        dt = 1/200
        seconds = 1

        TestEKF_Utils.read_imu_for_seconds(
            self.ekf_wrapper, imu_measurement, dt, seconds)

        np.testing.assert_almost_equal(self.ekf_wrapper.get_state(),
                                       self.initial_state,
                                       err_msg="State should match initial state.",
                                       verbose=True)

        process_noise_covariance = self.ekf_wrapper.compute_process_noise_covariance(dt)
        print('Process Noise Covariance:')
        print(process_noise_covariance)

        print('Final State:')
        print(self.ekf_wrapper.get_state()[0:6, 0:1])
        print('Final Covariance:')
        print(self.ekf_wrapper.get_state_covariance()[0:, 0:])

    # def test_predict_2(self):
    #     """
    #     Test EKF with 1m/s² and 100m/s² in all axis.
    #     """
    #     print("Test 2 - 1m/s² and 100m/s² in all axis.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200
    #     seconds = 1
    #     values = [1.0, -1.0, 100.0, -100.0]
    #     axis_names = ['x', 'y', 'z']
    #     for value in values:
    #         for axis in range(len(axis_names)):
    #             # Reset EKF
    #             self.ekf_wrapper.reset(self.initial_state,
    #                                    self.initial_covariance)
    #             imu_measurement = base_measurement.copy()
    #             imu_measurement[axis] += value
    #             print(f"Testing with {value} m/s² in {axis_names[axis]} axis")
    #             TestEKF_Utils.read_imu_for_seconds(
    #                 self.ekf_wrapper, imu_measurement, dt, seconds)
    #             movement = TestEKF_Utils.compute_real_linear_movement(
    #                 self.initial_state, imu_measurement, seconds)
    #             np.testing.assert_almost_equal(self.ekf_wrapper.get_state(),
    #                                            movement,
    #                                            err_msg=f"State should match initial state with {value} m/s² in {axis_names[axis]} axis.",
    #                                            verbose=True)
    #             print('Final State:')
    #             print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #             print('Final Covariance:')
    #             print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    # def test_predict_3(self):
    #     """
    #     Test EKF with 1m/s² and 100m/s² and stop in all axis.
    #     """
    #     print("Test 3 - 1m/s² and 100m/s² and stop in all axis.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200
    #     seconds = 1
    #     values = [1.0, -1.0, 100.0, -100.0]
    #     axis_names = ['x', 'y', 'z']
    #     for value in values:
    #         for axis in range(len(axis_names)):
    #             # Reset EKF
    #             self.ekf_wrapper.reset(self.initial_state,
    #                                    self.initial_covariance)
    #             imu_measurement = base_measurement.copy()
    #             imu_measurement[axis] += value
    #             print(
    #                 f"Testing with {value} m/s² and stop in {axis_names[axis]} axis")
    #             TestEKF_Utils.read_imu_for_seconds(
    #                 self.ekf_wrapper, imu_measurement, dt, seconds)
    #             movement = TestEKF_Utils.compute_real_linear_movement(
    #                 self.initial_state, imu_measurement, seconds)
    #             # Stop the movement
    #             imu_measurement = base_measurement.copy()
    #             imu_measurement[axis] -= value
    #             TestEKF_Utils.read_imu_for_seconds(
    #                 self.ekf_wrapper, imu_measurement, dt, seconds)
    #             movement = TestEKF_Utils.compute_real_linear_movement(
    #                 movement, imu_measurement, seconds)
    #             np.testing.assert_almost_equal(self.ekf_wrapper.get_state(),
    #                                            movement,
    #                                            err_msg=f"State should match initial state with {value} m/s² and stop in {axis_names[axis]} axis.",
    #                                            verbose=True)
    #             print('Final State:')
    #             print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #             print('Final Covariance:')
    #             print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    # def test_predict_4(self):
    #     """
    #     Test EKF with 1,5708rad/s.
    #     """
    #     print("Test 4 - 1,5708rad/s.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200
    #     seconds = 1
    #     # Reset EKF
    #     self.ekf_wrapper.reset(self.initial_state,
    #                            self.initial_covariance)
    #     imu_measurement = base_measurement.copy()
    #     imu_measurement[5] = 1.5708  # 90 degrees in rad/s
    #     print("Testing with 1,5708 rad/s in z axis")
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds, verbose=False)
    #     imu_measurement = base_measurement.copy()
    #     imu_measurement[0] = 1.0
    #     seconds = 2
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds, verbose=False)
    #     print('Final State:')
    #     print(self.ekf_wrapper.get_state()[0:9, 0:1])
    #     print('Final Covariance:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:9, 0:9])

    # def test_predict_5(self):
    #     """
    #     Test EKF with rotation in Y axis.
    #     """
    #     print("Test 5 - Rotation in Y axis.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200
    #     # Reset EKF
    #     self.ekf_wrapper.reset(self.initial_state,
    #                            self.initial_covariance)
    #     imu_measurement = base_measurement.copy()
    #     imu_measurement[4] = 1.5708
    #     seconds = 0.5
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds, verbose=True)
    #     imu_measurement = base_measurement.copy()
    #     imu_measurement[4] = -1.5708
    #     seconds = 0.5
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds, verbose=True)

    # def test_update_1(self):
    #     """
    #     Test EKF update with pose measurement.
    #     """
    #     print("Test 6 - Update with pose measurement.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200

    #     # Reset EKF
    #     self.ekf_wrapper.reset(self.initial_state,
    #                            self.initial_covariance)

    #     # Simulate a pose measurement
    #     pose_measurement = np.array([
    #         1.0, 1.0, 1.0,  # Position (x, y, z)
    #         0.0, 0.0, 0.0,  # Orientation (roll, pitch, yaw)
    #     ])

    #     print('State before predict:')
    #     print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #     print('Covariance before predict:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #     imu_measurement = base_measurement.copy()
    #     # imu_measurement[0] = 1.0

    #     print('IMU measurement:')
    #     print(imu_measurement)

    #     seconds = 1
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds)

    #     print('State before update:')
    #     print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #     print('Covariance before update:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #     self.ekf_wrapper.update_pose(pose_measurement, np.ones(6) * 1e-12)

    #     print('State after pose update:')
    #     print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #     print('Covariance after pose update:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #     np.testing.assert_almost_equal(
    #         self.ekf_wrapper.get_state()[0:3, 0],
    #         np.array([1.0, 1.0, 1.0]),
    #         decimal=4,
    #         err_msg="Position should match.",
    #         verbose=True)

    # def test_update_2(self):
    #     """
    #     Test EKF update with pose + velocity measurement.
    #     """
    #     print("Test 7 - Update with pose measurement.")
    #     # Base
    #     base_measurement = np.array([
    #         0.0, 0.0, 9.81,  # Accelerometer (ax, ay, az)
    #         0.0, 0.0, 0.0  # Gyroscope (gx, gy, gz)
    #     ])
    #     dt = 1/200

    #     # Reset EKF
    #     self.ekf_wrapper.reset(self.initial_state,
    #                            self.initial_covariance)

    #     # Simulate a pose measurement
    #     pose_measurement = np.array([
    #         0.0, 0.0, 0.0,  # Position (x, y, z)
    #         0.0, 0.0, 0.0,  # Orientation (roll, pitch, yaw)
    #         1.0, 0.0, 0.0,  # Velocity (vx, vy, vz)
    #     ])

    #     print('State before update:')
    #     print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #     print('Covariance before update:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #     imu_measurement = base_measurement.copy()
    #     # imu_measurement[0] = 1.0

    #     print('IMU measurement:')
    #     print(imu_measurement)

    #     for i in range(1):

    #         seconds = 1
    #         TestEKF_Utils.read_imu_for_seconds(
    #             self.ekf_wrapper, imu_measurement, dt, seconds)

    #         # print('State after update:')
    #         # print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #         # print('Covariance after update:')
    #         # print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #         self.ekf_wrapper.update_pose_velocity(
    #             pose_measurement, np.ones(9) * 1e-12)

    #     seconds = 1
    #     TestEKF_Utils.read_imu_for_seconds(
    #         self.ekf_wrapper, imu_measurement, dt, seconds)

    #     print('State after pose update:')
    #     print(self.ekf_wrapper.get_state()[0:6, 0:1])
    #     print('Covariance after pose update:')
    #     print(self.ekf_wrapper.get_state_covariance()[0:6, 0:6])

    #     np.testing.assert_almost_equal(
    #         self.ekf_wrapper.get_state()[0:3, 0],
    #         np.array([1.0, 0.0, 0.0]),
    #         decimal=4,
    #         err_msg="Position should match.",
    #         verbose=True)


if __name__ == '__main__':
    unittest.main()
