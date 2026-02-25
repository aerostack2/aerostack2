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

"""Utils definition."""

__authors__ = 'Rodrigo da Silva Gómez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion.

    :param q: The quaternion to normalize, shape (4,), [qw, qx, qy, qz].
    :type q: np.ndarray
    :return: The normalized quaternion.
    :rtype: np.ndarray
    """
    norm = np.linalg.norm(q)
    if norm == 0:
        norm = 1e-6
    return q / norm


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: The quaternion [qw, qx, qy, qz], shape (4,).
    :type q: np.ndarray
    :return: The Euler angles [roll, pitch, yaw], shape (3,).
    :rtype: np.ndarray
    """
    # First, normalize the quaternion to ensure a valid rotation
    q_normed = normalize_quaternion(q)
    qw, qx, qy, qz = q_normed

    # --- Roll (x-axis rotation) ---
    # sinr_cosp = 2 * (qw * qx + qy * qz)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    # cosr_cosp = 1 - 2 * (qx^2 + qy^2)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # --- Pitch (y-axis rotation) ---
    # sinp = 2 * (qw * qy - qz * qx)
    sinp = 2.0 * (qw * qy - qz * qx)
    # Clamp sinp to [-1, 1] to avoid invalid domain for arcsin
    sinp_clamped = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp_clamped)

    # --- Yaw (z-axis rotation) ---
    # siny_cosp = 2 * (qw * qz + qx * qy)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    # cosy_cosp = 1 - 2 * (qy^2 + qz^2)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def euler_to_quaternion(euler_angles: np.ndarray) -> np.ndarray:
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.

    :param euler_angles: The Euler angles [roll, pitch, yaw], shape (3,).
    :type euler_angles: np.ndarray
    :return: The quaternion [qw, qx, qy, qz], shape (4,).
    :rtype: np.ndarray
    """
    roll, pitch, yaw = euler_angles

    # Compute half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    # Compute quaternion components
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.array([qw, qx, qy, qz])


def transform_to_pose(transform: np.ndarray) -> np.ndarray:
    """
    Convert a 4×4 transformation matrix to a pose (position and quaternion).

    :param transform: The transformation matrix, shape (4, 4).
    :type transform: np.ndarray
    :return: The pose as a numpy array [x, y, z, qw, qx, qy, qz], shape (7,).
    :rtype: np.ndarray
    """
    if transform.shape != (4, 4):
        raise ValueError(f"Expected a 4×4 matrix, got shape {transform.shape}")

    # 1) translation
    x, y, z = transform[0:3, 3]

    # 2) rotation matrix
    R = transform[0:3, 0:3]
    t = np.trace(R)

    # 3) compute quaternion
    if t > 0.0:
        S = np.sqrt(t + 1.0) * 2.0  # S = 4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        # find the largest diagonal element among R[0,0], R[1,1], R[2,2]
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0  # S = 4*qx
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0  # S = 4*qy
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0  # S = 4*qz
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    return np.array([x, y, z, qw, qx, qy, qz])


def compute_tilt_and_euler_angles(g: np.ndarray,
                                  g_ref: np.ndarray = np.array([0., 0., 9.81])) -> tuple:
    """
    Compute the total tilt angle (from vertical) and the roll & pitch Euler angles
    from a measured gravity vector `g` (numpy array of shape (3,)).

    Parameters
    ----------
    g : np.ndarray
        Measured gravity vector [gx, gy, gz] in m/s².
    g_ref : np.ndarray, optional
        Reference gravity vector (default [0, 0, 9.81]).

    Returns
    -------
    tilt : float
        Total tilt angle from vertical (radians).
    roll : float
        Rotation about the x-axis (radians).
    pitch : float
        Rotation about the y-axis (radians).
    """
    # Normalize vectors
    g_norm = np.linalg.norm(g)
    g_unit = g / g_norm
    g_ref_unit = g_ref / np.linalg.norm(g_ref)

    # Total tilt via dot product
    # Clip the dot product for numerical stability
    cos_theta = np.clip(np.dot(g_unit, g_ref_unit), -1.0, 1.0)
    tilt = np.arccos(cos_theta)

    # Euler angles (roll, pitch)
    gx, gy, gz = g
    roll = np.arctan2(gy, gz)
    pitch = np.arctan2(-gx, np.sqrt(gy**2 + gz**2))

    return tilt, roll, pitch


def pose_to_transform(position, euler_rpy):
    """
    Build a 4×4 homogeneous transform from position (x,y,z)
    and Euler angles (roll, pitch, yaw) without any helper library.
    Uses:
      R_x = rotation about X (roll)
      R_y = rotation about Y (pitch)
      R_z = rotation about Z (yaw)
    and composes R = R_z @ R_y @ R_x.
    """
    roll, pitch, yaw = euler_rpy
    roll = float(roll)
    pitch = float(pitch)
    yaw = float(yaw)
    # print(roll, pitch, yaw)

    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    # print(cr, sr, cp, sp, cy, sy)

    # Rotation about X-axis (roll)
    R_x = np.array([[1,  0,   0],
                    [0, cr, -sr],
                    [0, sr,  cr]])

    # Rotation about Y-axis (pitch)
    R_y = np.array([[cp, 0, sp],
                    [0, 1,  0],
                    [-sp, 0, cp]])

    # Rotation about Z-axis (yaw)
    R_z = np.array([[cy, -sy, 0],
                    [sy,  cy, 0],
                    [0,   0, 1]])

    # Compose: first roll, then pitch, then yaw
    R = R_z @ R_y @ R_x

    # Build homogeneous 4×4
    T = np.eye(4)
    T[0:3, 0:3] = R
    # print(T[0:3, 3:4])
    # print("Position:", position)
    T[0:3, 3:4] = position.reshape((-1, 1))
    return T


def compute_map_to_odom(state, X_new, prev_map_to_odom):
    """
    state:            prev map→base pose as a 9-vector [x,y,z, ..., roll,pitch,yaw]
    X_new:            new  map→base pose in same format
    prev_map_to_odom: 4×4 homogeneous map→odom from last time
    """
    # 1) extract positions and RPY
    p_prev = state[0:3]
    r_prev = (state[6], state[7], state[8])  # roll, pitch, yaw
    p_new = X_new[0:3]
    r_new = (X_new[6], X_new[7], X_new[8])  # roll, pitch, yaw

    # 2) builds
    T_map_base_prev = pose_to_transform(p_prev, r_prev)
    T_map_base_new = pose_to_transform(p_new,  r_new)
    # print("T_map_base_prev:\n", T_map_base_prev)
    # print("T_map_base_new:\n", T_map_base_new)

    # 3) delta = how base moved in the map frame
    delta = T_map_base_new @ np.linalg.inv(T_map_base_prev)

    # 4) update map→odom by applying same delta
    T_map_odom_new = delta @ prev_map_to_odom

    return T_map_odom_new
