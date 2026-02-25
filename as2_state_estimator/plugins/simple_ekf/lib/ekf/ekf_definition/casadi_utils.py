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


import casadi as ca


class Utils():
    """
    Utility functions for the EKF.
    """

    @staticmethod
    def quaternion_multiply(q1: ca.SX, q2: ca.SX) -> ca.SX:
        """
        Multiply two quaternions.

        q = q1 x q2 = [qw1, qx1, qy1, qz1] x [qw2, qx2, qy2, qz2]

        :param q1 (ca.SX): The first quaternion [qw1, qx1, qy1, qz1].
        :param q2 (ca.SX): The second quaternion [qw2, qx2, qy2, qz2].

        :return (ca.SX): The resulting quaternion [qw, qx, qy, qz].
        """
        qw1 = q1[0]
        qx1 = q1[1]
        qy1 = q1[2]
        qz1 = q1[3]

        qw2 = q2[0]
        qx2 = q2[1]
        qy2 = q2[2]
        qz2 = q2[3]

        qw = qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2
        qx = qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2
        qy = qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2
        qz = qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2

        return ca.vertcat(qw, qx, qy, qz)

    @staticmethod
    def apply_rotation(q: ca.SX, v: ca.SX) -> ca.SX:
        """
        Apply a rotation to a vector.

        v_rotated = q x v x q_conj

        :param q (ca.SX): The quaternion [qw, qx, qy, qz].
        :param v (ca.SX): The vector [vx, vy, vz].

        :return (ca.SX): The rotated vector [vx_rotated, vy_rotated, vz_rotated].
        """
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        vx = v[0]
        vy = v[1]
        vz = v[2]

        q_conj = ca.vertcat(qw, -qx, -qy, -qz)

        v_rotated = Utils.quaternion_multiply(
            Utils.quaternion_multiply(q, ca.vertcat(0, vx, vy, vz)),
            q_conj)

        return ca.vertcat(v_rotated[1], v_rotated[2], v_rotated[3])

    @staticmethod
    def normalize_quaternion(q: ca.SX) -> ca.SX:
        """
        Normalize a quaternion.

        :param q (ca.SX): The quaternion to normalize.

        :return (ca.SX): The normalized quaternion.
        """
        q_norm = ca.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
        return q / q_norm

    @staticmethod
    def quaternion_derivate(quaternion: ca.SX, angular_velocity: ca.SX) -> ca.SX:
        """
        Compute the quaternion derivative.

        q_dot = 0.5 * q x omega = 0.5 * [qw, qx, qy, qz] * [0, ,wx, wy, wz]

        :param quaternion (ca.SX): The quaternion [qw, qx, qy, qz].
        :param angular_velocity (ca.SX): The angular velocity [wx, wy, wz].

        :return (ca.SX): The quaternion derivative [qw_dot, qx_dot, qy_dot, qz_dot].
        """
        w_qx = angular_velocity[0]
        w_qy = angular_velocity[1]
        w_qz = angular_velocity[2]
        w_q = ca.vertcat(
            0.0,
            w_qx,
            w_qy,
            w_qz)

        return 0.5 * Utils.quaternion_multiply(
            Utils.normalize_quaternion(quaternion), w_q)

    @staticmethod
    def euler_apply_rotation(
            state_orientation: ca.SX,
            input_acceleration: ca.SX) -> ca.SX:
        """
        Apply a rotation to a vector using Euler angles.

        :param state_orientation (ca.SX): The state orientation [roll, pitch, yaw].
        :param input_acceleration (ca.SX): The acceleration [ax, ay, az].

        :return (ca.SX): The rotated vector [vx_rotated, vy_rotated, vz_rotated].
        """
        roll, pitch, yaw = ca.vertsplit(state_orientation)
        ax, ay, az = ca.vertsplit(input_acceleration)

        # Rotation matrix
        row1 = ca.horzcat(
            ca.cos(yaw) * ca.cos(pitch),
            ca.sin(roll) * ca.sin(pitch) * ca.cos(yaw) -
            ca.cos(roll) * ca.sin(yaw),
            ca.cos(roll) * ca.sin(pitch) * ca.cos(yaw) +
            ca.sin(roll) * ca.sin(yaw)
        )
        row2 = ca.horzcat(
            ca.sin(yaw) * ca.cos(pitch),
            ca.sin(roll) * ca.sin(pitch) * ca.sin(yaw) +
            ca.cos(roll) * ca.cos(yaw),
            ca.cos(roll) * ca.sin(pitch) * ca.sin(yaw) -
            ca.sin(roll) * ca.cos(yaw)
        )
        row3 = ca.horzcat(
            -ca.sin(pitch),
            ca.sin(roll) * ca.cos(pitch),
            ca.cos(roll) * ca.cos(pitch)
        )
        R = ca.vertcat(row1, row2, row3)

        return R @ input_acceleration

    @staticmethod
    def euler_rotation_derivative(
            state_orientation: ca.SX,
            input_angular_velocity: ca.SX) -> ca.SX:
        """
        Compute the Euler angles derivative.

        :param state_orientation (ca.SX): The state orientation [roll, pitch, yaw].
        :param input_angular_velocity (ca.SX): The angular velocity [wx, wy, wz].

        :return (ca.SX): The Euler angles derivative [roll_dot, pitch_dot, yaw_dot].
        """
        roll, pitch, yaw = ca.vertsplit(state_orientation)
        wx, wy, wz = ca.vertsplit(input_angular_velocity)

        roll_dot = wx + wy * \
            ca.sin(roll) * ca.tan(pitch) + wz * ca.cos(roll) * ca.tan(pitch)
        pitch_dot = wy * ca.cos(roll) - wz * ca.sin(roll)
        yaw_dot = wy * ca.sin(roll) / ca.cos(pitch) + \
            wz * ca.cos(roll) / ca.cos(pitch)

        return ca.vertcat(roll_dot, pitch_dot, yaw_dot)

    @staticmethod
    def velocity_derivative(
            state_orientation: ca.SX,
            input_acceleration: ca.SX,
            gravity: ca.SX) -> ca.SX:
        """
        Compute the velocity derivative.
        v_dot = q(input - noise) - g
        :return (ca.SX): The velocity derivative [vx_dot, vy_dot, vz_dot].
        """
        roll, pitch, yaw = ca.vertsplit(state_orientation)
        iax, iay, iaz = ca.vertsplit(input_acceleration)

        v_dot = Utils.euler_apply_rotation(
            state_orientation,
            input_acceleration
        )
        # v_dot = input_acceleration

        return ca.vertcat(
            v_dot[0] - gravity[0],
            v_dot[1] - gravity[1],
            v_dot[2] - gravity[2]
        )

    @staticmethod
    def state_quaternion_normalization(state: ca.SX) -> ca.SX:
        """
        Normalize the quaternion part of the state vector.

        :param state (ca.SX): The state vector [x, y, z, vx, vy, vz, qw, qx, qy, qz].

        :return (ca.SX): The normalized state vector.
        """
        q = state[6:10]
        q_norm = Utils.normalize_quaternion(q)

        return ca.vertcat(
            state[0:6],
            q_norm,
            state[10:16]
        )
