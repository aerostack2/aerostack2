#!/usr/bin/env python3

"""Collection of utility functions for working with motion references."""

# Copyright 2023 Universidad Politécnica de Madrid
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

__authors__ = ' Rafael Perez Seguí, Miguel Fernandez Cortizas, Pedro Arias Perez '
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'


from geometry_msgs.msg import Quaternion
from rclpy import logging
from rclpy.node import Node
from scipy.spatial.transform import Rotation


def get_quaternion_from_yaw_angle(yaw_angle: float):
    """Get quaternion from yaw angle."""
    rot = Rotation.from_euler(
        'xyz', [0.0, 0.0, yaw_angle], degrees=False)
    rot_quat = rot.as_quat()
    return Quaternion(
        x=rot_quat[0], y=rot_quat[1], z=rot_quat[2], w=rot_quat[3])


def generate_tf_name(namespace: str, frame_name: str):
    """Generate tf name."""
    if len(frame_name) == 0:
        raise RuntimeError('Empty frame name')
    if frame_name[0] == '/':
        return frame_name[1:]
    if len(namespace) == 0:
        logging.get_logger('tf_utils').warn(
            f'The frame name {frame_name} is not absolute and the node namespace is empty.\
            This could lead to conflicts.')
        return frame_name
    if namespace[0] == '/':
        namespace = namespace[1:]
    # If frame_name until first '/' is equal to namespace, then frame_name is absolute
    pos = frame_name.find('/')
    if pos == -1:
        return namespace + '/' + frame_name
    return frame_name


def get_tf_name(node: Node, frame_name: str):
    """Get tf name."""
    return generate_tf_name(node.get_namespace(), frame_name)
