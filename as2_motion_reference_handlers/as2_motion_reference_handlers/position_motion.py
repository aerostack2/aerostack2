#!/usr/bin/env python3

"""Implementation of a motion reference handler for position motion."""
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


class PositionMotion(BasicMotionReferenceHandler):
    """
    Send POSITION references: a target pose plus an optional speed limit.

    The pose carries the position and, in yaw angle mode, the desired yaw. The twist is
    not a velocity reference here but a limit on how fast the vehicle may approach the
    target; left unset, no limit is imposed.
    """

    def __init__(self, node: Node):
        """
        Create the handler and select the POSITION control mode.

        :param node: node the references are published from
        :type node: Node
        """
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.POSITION

    def __own_send_command(self, yaw_mode: int,
                           pose_msg: PoseStamped,
                           twist_mgs: TwistStamped) -> bool:
        """
        Settle the yaw mode and publish both references.

        :param yaw_mode: ControlMode.YAW_ANGLE or ControlMode.YAW_SPEED
        :type yaw_mode: int
        :param pose_msg: target pose
        :type pose_msg: PoseStamped
        :param twist_mgs: speed limit
        :type twist_mgs: TwistStamped
        :return: True if every reference was published
        :rtype: bool
        """
        self.desired_control_mode_.yaw_mode = yaw_mode
        self.command_pose_msg_ = pose_msg
        self.command_twist_msg_ = twist_mgs
        send_pose = self.send_pose_command()
        send_twist = self.send_twist_command()
        return send_pose and send_twist

    def __check_input_pose(self, pose: Union[PoseStamped, list],
                           pose_frame_id: str) -> Union[PoseStamped, None]:
        """
        Normalize the pose argument into a PoseStamped, rejecting it if it has no frame.

        :param pose: target position, either a message or an [x, y, z] list
        :type pose: Union[PoseStamped, list]
        :param pose_frame_id: frame the list is expressed in, unused if pose is a message
        :type pose_frame_id: str
        :return: the pose as a message, or None if the type is wrong or the frame is empty
        :rtype: Union[PoseStamped, None]
        """
        pose_msg = PoseStamped()
        if isinstance(pose, list):
            pose_msg.pose.position.x = float(pose[0])
            pose_msg.pose.position.y = float(pose[1])
            pose_msg.pose.position.z = float(pose[2])
            pose_msg.header.frame_id = pose_frame_id
        elif isinstance(pose, PoseStamped):
            pose_msg = pose
        else:
            self.node.get_logger().error('Pose is not a list or PoseStamped')
            return None

        if pose_msg.header.frame_id == '':
            self.node.get_logger().error('Pose frame id is not set')
            return None
        return pose_msg

    def __check_input_twist(self,
                            twist: Union[float, list, TwistStamped, None],
                            twist_frame_id: str) -> Union[TwistStamped, None]:
        """
        Normalize the speed limit into a TwistStamped, rejecting it if it has no frame.

        A float applies the same limit to the three axes, a list gives one per axis, and
        None means no limit. The frame is required in every case, since a limit without a
        frame cannot be converted.

        :param twist: speed limit, as a single value, a per axis list, a message or None
        :type twist: Union[float, list, TwistStamped, None]
        :param twist_frame_id: frame the limit is expressed in
        :type twist_frame_id: str
        :return: the limit as a message, or None if the type is wrong or the frame is empty
        :rtype: Union[TwistStamped, None]
        """
        twist_mgs = TwistStamped()

        if isinstance(twist, float):
            twist_mgs.twist.linear.x = twist
            twist_mgs.twist.linear.y = twist
            twist_mgs.twist.linear.z = twist
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, list):
            twist_mgs.twist.linear.x = float(twist[0])
            twist_mgs.twist.linear.y = float(twist[1])
            twist_mgs.twist.linear.z = float(twist[2])
            twist_mgs.header.frame_id = twist_frame_id
        elif isinstance(twist, TwistStamped):
            twist_mgs = twist
        elif twist is None:
            # Default value -> no limit
            twist_mgs.header.frame_id = twist_frame_id
        else:
            self.node.get_logger().error('Twist is not a float, list or TwistStamped')
            return None

        if twist_mgs.header.frame_id == '':
            self.node.get_logger().error('Twist frame id is not set')
            return None

        return twist_mgs

    def send_position_command_with_yaw_angle(self,
                                             pose: Union[list, PoseStamped],
                                             twist_limit: Union[float, list,
                                                                TwistStamped, None] = None,
                                             pose_frame_id: str = '',
                                             twist_frame_id: str = '',
                                             yaw_angle: Union[float, None] = None) -> bool:
        """
        Send a position reference holding an absolute yaw angle.

        When the pose is given as a list, yaw_angle is required, since the list carries no
        orientation.

        :param pose: target position, either a message or an [x, y, z] list
        :type pose: Union[list, PoseStamped]
        :param twist_limit: speed limit for the approach, None for no limit
        :type twist_limit: Union[float, list, TwistStamped, None], optional
        :param pose_frame_id: frame of the pose, when given as a list
        :type pose_frame_id: str, optional
        :param twist_frame_id: frame of the speed limit
        :type twist_frame_id: str, optional
        :param yaw_angle: desired yaw in radians, required when pose is a list
        :type yaw_angle: Union[float, None], optional
        :return: True if the reference was published
        :rtype: bool
        """
        pose_msg = self.__check_input_pose(pose, pose_frame_id)
        twist_msg = self.__check_input_twist(twist_limit, twist_frame_id)

        if twist_msg is None or pose_msg is None:
            return False

        if isinstance(pose, list):
            if not isinstance(yaw_angle, float):
                self.node.get_logger().error(
                    'Yaw angle is not set')
                return False
            pose_msg.pose.orientation = utils.get_quaternion_from_yaw_angle(
                yaw_angle)

        return self.__own_send_command(ControlMode.YAW_ANGLE, pose_msg, twist_msg)

    def send_position_command_with_yaw_speed(self,
                                             pose: Union[list, PoseStamped],
                                             twist_limit: Union[float, list,
                                                                TwistStamped, None] = None,
                                             pose_frame_id: str = '',
                                             twist_frame_id: str = '',
                                             yaw_speed: Union[float, None] = None) -> bool:
        """
        Send a position reference holding a yaw rate.

        The yaw rate travels in the angular z of the speed limit message.

        :param pose: target position, either a message or an [x, y, z] list
        :type pose: Union[list, PoseStamped]
        :param twist_limit: speed limit for the approach, None for no limit
        :type twist_limit: Union[float, list, TwistStamped, None], optional
        :param pose_frame_id: frame of the pose, when given as a list
        :type pose_frame_id: str, optional
        :param twist_frame_id: frame of the speed limit
        :type twist_frame_id: str, optional
        :param yaw_speed: desired yaw rate in rad/s
        :type yaw_speed: Union[float, None], optional
        :return: True if the reference was published
        :rtype: bool
        """
        pose_msg = self.__check_input_pose(pose, pose_frame_id)
        twist_msg = self.__check_input_twist(twist_limit, twist_frame_id)

        if twist_msg is None or pose_msg is None:
            return False

        if isinstance(twist_limit, list):
            if not isinstance(yaw_speed, float):
                self.node.get_logger().error(
                    'Yaw speed is not set')
                return False

        if yaw_speed is not None:
            twist_msg.twist.angular.z = yaw_speed

        return self.__own_send_command(ControlMode.YAW_SPEED, pose_msg, twist_msg)
