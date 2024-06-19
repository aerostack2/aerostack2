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
    """Send speed motion command."""

    def __init__(self, node: Node):
        super().__init__(node)
        self.desired_control_mode_.yaw_mode = ControlMode.NONE
        self.desired_control_mode_.control_mode = ControlMode.SPEED
        self.desired_control_mode_.reference_frame = ControlMode.UNDEFINED_FRAME

    def __own_send_command(self, yaw_mode: int, twist_mgs: TwistStamped,
                           pose_msg: Union[PoseStamped, None] = None) -> bool:
        """Send speed command."""
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
        """Check if the input twist is valid and return a TwistStamped message."""
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
                                          pose_frame_id: str = 'earth') -> bool:
        """Send speed command with yaw angle."""
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
        """Send speed command with yaw speed."""
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
