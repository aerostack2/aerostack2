#!/usr/bin/env python3

"""Implementation of a motion reference handler for speed in a plane motion."""

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

from typing import Union

from as2_motion_reference_handlers import utils
from as2_motion_reference_handlers.basic_motion_references import BasicMotionReferenceHandler
from as2_msgs.msg import ControlMode
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from rclpy.node import Node


class SpeedInAPlaneMotion(BasicMotionReferenceHandler):
    """
    Send SPEED_IN_A_PLANE references: horizontal velocity plus a held height.

    This mode splits the reference across the two messages: the twist carries the
    horizontal velocity and the pose carries the height to hold and the yaw. That is why
    both are published, each with its own frame.
    """

    def __init__(self, node: Node):
        """
        Create the handler and select the SPEED_IN_A_PLANE control mode.

        :param node: node the references are published from
        :type node: Node
        """
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.SPEED_IN_A_PLANE

    def __own_send_command(self, yaw_mode: int, pose_msg: PoseStamped,
                           twist_mgs: TwistStamped) -> bool:
        """
        Settle the yaw mode and publish both references.

        :param yaw_mode: ControlMode.YAW_ANGLE or ControlMode.YAW_SPEED
        :type yaw_mode: int
        :param pose_msg: pose carrying the height to hold, and the yaw in angle mode
        :type pose_msg: PoseStamped
        :param twist_mgs: horizontal velocity reference
        :type twist_mgs: TwistStamped
        :return: True if every reference was published
        :rtype: bool
        """
        self.desired_control_mode_.yaw_mode = yaw_mode

        self.command_pose_msg_ = pose_msg
        send_pose = self.send_pose_command()

        self.command_twist_msg_ = twist_mgs
        send_twist = self.send_twist_command()

        return send_pose and send_twist

    def __check_input_pose(self, pose: Union[PoseStamped, float],
                           pose_frame_id: str) -> Union[PoseStamped, None]:
        """
        Normalize the height argument into a PoseStamped, rejecting it if it has no frame.

        :param pose: height to hold, either a full pose or a z value
        :type pose: Union[PoseStamped, float]
        :param pose_frame_id: frame the height is expressed in
        :type pose_frame_id: str
        :return: the pose as a message, or None if the type is wrong or the frame is empty
        :rtype: Union[PoseStamped, None]
        """
        pose_msg = PoseStamped()
        if isinstance(pose, float):
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = float(pose)
            pose_msg.header.frame_id = pose_frame_id
        elif isinstance(pose, PoseStamped):
            pose_msg = pose
        else:
            self.node.get_logger().error('Height is not a float or PoseStamped')
            return None

        if pose_msg.header.frame_id == '':
            self.node.get_logger().error('Height frame id is not set')
            return None
        return pose_msg

    def __check_input_twist(self, twist: Union[TwistStamped, list],
                            twist_frame_id: str) -> Union[TwistStamped, None]:
        """
        Normalize the twist argument into a TwistStamped, rejecting it if it has no frame.

        :param twist: horizontal velocity, either a message or an [x, y] list
        :type twist: Union[TwistStamped, list]
        :param twist_frame_id: frame the list is expressed in, unused if twist is a message
        :type twist_frame_id: str
        :return: the twist as a message, or None if the type is wrong or the frame is empty
        :rtype: Union[TwistStamped, None]
        """
        twist_mgs = TwistStamped()

        if isinstance(twist, list):
            twist_mgs.twist.linear.x = float(twist[0])
            twist_mgs.twist.linear.y = float(twist[1])
            twist_mgs.twist.linear.z = 0.0
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, TwistStamped):
            twist_mgs = twist
        else:
            self.node.get_logger().error('Twist is not a list or TwistStamped')
            return None

        if twist_mgs.header.frame_id == '':
            self.node.get_logger().error('Twist frame id is not set')
            return None

        return twist_mgs

    def send_speed_in_a_plane_command_with_yaw_angle(
            self,
            twist: Union[TwistStamped, list],
            height: Union[PoseStamped, float],
            pose_frame_id: str = '',
            twist_frame_id: str = '',
            yaw_angle: Union[Quaternion, float, None] = None) -> bool:
        """
        Send a speed in a plane reference holding an absolute yaw angle.

        The yaw may come as a quaternion, as an angle, or already inside the height pose;
        in that last case yaw_angle is left unset.

        :param twist: horizontal velocity, either a message or an [x, y] list
        :type twist: Union[TwistStamped, list]
        :param height: height to hold, either a full pose or a z value
        :type height: Union[PoseStamped, float]
        :param pose_frame_id: frame of the height
        :type pose_frame_id: str, optional
        :param twist_frame_id: frame of the twist, when given as a list
        :type twist_frame_id: str, optional
        :param yaw_angle: desired yaw, as a quaternion or radians
        :type yaw_angle: Union[Quaternion, float, None], optional
        :return: True if the reference was published
        :rtype: bool
        """
        twist_msg = self.__check_input_twist(twist, twist_frame_id)
        pose_msg = self.__check_input_pose(height, pose_frame_id)

        if twist_msg is None or pose_msg is None:
            return False

        if isinstance(yaw_angle, Quaternion):
            pose_msg.pose.orientation = yaw_angle
        elif isinstance(yaw_angle, float):
            pose_msg.pose.orientation = utils.get_quaternion_from_yaw_angle(
                yaw_angle)
        elif yaw_angle is None and isinstance(height, PoseStamped):
            pass
        else:
            self.node.get_logger().error(
                'Yaw angle is not set')
            return False

        return self.__own_send_command(ControlMode.YAW_ANGLE, pose_msg, twist_msg)

    def send_speed_in_a_plane_command_with_yaw_speed(
            self,
            twist: Union[TwistStamped, list],
            height: Union[PoseStamped, float],
            pose_frame_id: str = '',
            twist_frame_id: str = '',
            yaw_speed: Union[float, None] = None) -> bool:
        """
        Send a speed in a plane reference holding a yaw rate.

        :param twist: horizontal velocity, either a message or an [x, y] list
        :type twist: Union[TwistStamped, list]
        :param height: height to hold, either a full pose or a z value
        :type height: Union[PoseStamped, float]
        :param pose_frame_id: frame of the height
        :type pose_frame_id: str, optional
        :param twist_frame_id: frame of the twist, when given as a list
        :type twist_frame_id: str, optional
        :param yaw_speed: desired yaw rate in rad/s
        :type yaw_speed: Union[float, None], optional
        :return: True if the reference was published
        :rtype: bool
        """
        twist_msg = self.__check_input_twist(twist, twist_frame_id)
        pose_msg = self.__check_input_pose(height, pose_frame_id)

        if twist_msg is None or pose_msg is None:
            return False

        if isinstance(yaw_speed, float):
            twist_msg.twist.angular.z = yaw_speed
        elif yaw_speed is None and isinstance(twist, TwistStamped):
            pass
        else:
            self.node.get_logger().error(
                'Yaw speed is not set')
            return False

        return self.__own_send_command(ControlMode.YAW_SPEED, pose_msg, twist_msg)
