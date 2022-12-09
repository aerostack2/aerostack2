"""
Collection of utility functions for working with motion references.
"""

from rclpy.node import Node
from rclpy import logging
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation


def get_quaternion_from_yaw_angle(yaw_angle: float):
    """ Get quaternion from yaw angle """
    rot = Rotation.from_euler(
        'xyz', [0.0, 0.0, yaw_angle], degrees=False)
    rot_quat = rot.as_quat()
    return Quaternion(
        x=rot_quat[0], y=rot_quat[1], z=rot_quat[2], w=rot_quat[3])


def generate_tf_name(namespace: str, frame_name: str):
    """ Generate tf name """
    if len(frame_name) == 0:
        raise RuntimeError("Empty frame name")
    if frame_name[0] == '/':
        return frame_name[1:]
    if len(namespace) == 0:
        logging.get_logger("tf_utils").warn(
            "The frame name [%s] is not absolute and the node namespace is empty. This could "
            "lead to conflicts.", frame_name)
        return frame_name
    if namespace[0] == '/':
        namespace = namespace[1:]
    # If frame_name until first '/' is equal to namespace, then frame_name is absolute
    pos = frame_name.find('/')
    if pos == -1:
        return namespace + "/" + frame_name
    return frame_name


def get_tf_name(node: Node, frame_name: str):
    """ Get tf name """
    return generate_tf_name(node.get_namespace(), frame_name)
