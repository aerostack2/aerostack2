#!/usr/bin/env python3

"""Implementation of a motion reference handler for speed motion."""

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
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node


class SpeedMotion(BasicMotionReferenceHandler):
    """
    Send SPEED references: a linear velocity plus a yaw angle or a yaw rate.

    The twist carries the linear velocity and must always name the frame it is expressed
    in. Yaw is commanded either as an absolute angle, through a pose, or as a rate, in the
    angular z component of the twist.
    """

    def __init__(self, node: Node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.SPEED

    def __own_send_command(self, yaw_mode: int, twist_mgs: TwistStamped,
                           pose_msg: Union[PoseStamped, None] = None) -> bool:
        """
        Settle the yaw mode and publish the references.

        :param yaw_mode: ControlMode.YAW_ANGLE or ControlMode.YAW_SPEED
        :type yaw_mode: int
        :param twist_mgs: linear velocity reference
        :type twist_mgs: TwistStamped
        :param pose_msg: pose carrying the yaw angle, only for YAW_ANGLE
        :type pose_msg: Union[PoseStamped, None], optional
        :return: True if every reference was published
        :rtype: bool
        """
        self.desired_control_mode_.yaw_mode = yaw_mode

        send_pose = True
        if pose_msg is not None:
            self.command_pose_msg_ = pose_msg
            send_pose = self.send_pose_command()

        self.command_twist_msg_ = twist_mgs
        send_twist = self.send_twist_command()
        return send_pose and send_twist

    def __check_input_twist(self, twist: Union[TwistStamped, list],
                            twist_frame_id: str) -> Union[TwistStamped, None]:
        """
        Normalize the twist argument into a TwistStamped, rejecting it if it has no frame.

        A reference without a frame id cannot be converted, and whoever receives it would
        act on it as if it were already in its own frame, so it is refused here.

        :param twist: linear velocity, either a message or a [x, y, z] list
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
            twist_mgs.twist.linear.z = float(twist[2])
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

    def send_speed_command_with_yaw_angle(self, twist: Union[TwistStamped, list],
                                          pose: Union[PoseStamped,
                                                      None] = None,
                                          twist_frame_id: str = '',
                                          yaw_angle: Union[float, None] = None,
                                          pose_frame_id: str = '') -> bool:
        """
        Send a speed reference holding an absolute yaw angle.

        The yaw comes either from a full pose or from an angle, in which case the pose is
        built from it. Exactly one of the two must be given.

        :param twist: linear velocity, either a message or a [x, y, z] list
        :type twist: Union[TwistStamped, list]
        :param pose: pose whose orientation holds the desired yaw
        :type pose: Union[PoseStamped, None], optional
        :param twist_frame_id: frame of the twist, when given as a list
        :type twist_frame_id: str, optional
        :param yaw_angle: desired yaw in radians, used when no pose is given
        :type yaw_angle: Union[float, None], optional
        :param pose_frame_id: frame the yaw angle refers to
        :type pose_frame_id: str, optional
        :return: True if the reference was published
        :rtype: bool
        """
        twist_msg = self.__check_input_twist(twist, twist_frame_id)

        if twist_msg is None:
            return False

        pose_msg = PoseStamped()
        if isinstance(pose, PoseStamped):
            pose_msg = pose
        elif isinstance(yaw_angle, float):
            pose_msg.header.frame_id = pose_frame_id
            pose_msg.pose.orientation = utils.get_quaternion_from_yaw_angle(
                yaw_angle)
        else:
            self.node.get_logger().error(
                'Yaw angle is not set')
            return False

        return self.__own_send_command(ControlMode.YAW_ANGLE, twist_msg, pose_msg)

    def send_speed_command_with_yaw_speed(self, twist: Union[TwistStamped, list],
                                          twist_frame_id: str = '',
                                          yaw_speed: Union[float, None] = None) -> bool:
        """
        Send a speed reference holding a yaw rate.

        When the twist is given as a message its angular z is used as the yaw rate; when
        it is given as a list, yaw_speed is required.

        :param twist: linear velocity, either a message or a [x, y, z] list
        :type twist: Union[TwistStamped, list]
        :param twist_frame_id: frame of the twist, when given as a list
        :type twist_frame_id: str, optional
        :param yaw_speed: desired yaw rate in rad/s, required when twist is a list
        :type yaw_speed: Union[float, None], optional
        :return: True if the reference was published
        :rtype: bool
        """
        twist_msg = self.__check_input_twist(twist, twist_frame_id)

        if twist_msg is None:
            return False

        if isinstance(twist, list):
            if not isinstance(yaw_speed, float):
                self.node.get_logger().error(
                    'Yaw speed is not set')
                return False
            twist_msg.twist.angular.z = yaw_speed

        return self.__own_send_command(ControlMode.YAW_SPEED, twist_msg)
