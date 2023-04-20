"""
follow_reference_behavior.py
"""

# Copyright 2022 Universidad Politécnica de Madrid
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
#    * Neither the name of the the copyright holder nor the names of its
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


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí, Javier Melero Deza"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import typing
from typing import Tuple

from as2_msgs.action import FollowReference
from geometry_msgs.msg import PoseStamped, Pose

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class FollowReferenceBehavior(BehaviorHandler):
    """FollowReference Behavior"""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, FollowReference, 'FollowReferenceBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, pose: Tuple[Pose, PoseStamped], frame_id: str,
              speed_x: float, speed_y: float, speed_z: float, yaw_mode: int, 
              yaw_angle: float, wait_result: bool = False) -> bool:
        goal_msg = FollowReference.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.target_pose.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = frame_id
        goal_msg.target_pose.point.x = pose_stamped.position.x
        goal_msg.target_pose.point.y = pose_stamped.position.y
        goal_msg.target_pose.point.z = pose_stamped.position.z

        goal_msg.max_speed_x = speed_x
        goal_msg.max_speed_y = speed_y
        goal_msg.max_speed_z = speed_z

        goal_msg.yaw.mode = yaw_mode
        if yaw_angle:
            goal_msg.yaw.angle = yaw_angle

        return super().start(goal_msg, wait_result)

    def modify(self, pose: Tuple[Pose, PoseStamped], frame_id: str,
              speed_x: float, speed_y: float, speed_z: float, yaw_mode: int, 
              yaw_angle: float):
        goal_msg = FollowReference.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.target_pose.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = frame_id  # TODO
        goal_msg.target_pose.point.x = pose_stamped.position.x
        goal_msg.target_pose.point.y = pose_stamped.position.y
        goal_msg.target_pose.point.z = pose_stamped.position.z

        goal_msg.max_speed_x = speed_x
        goal_msg.max_speed_y = speed_y
        goal_msg.max_speed_z = speed_z
        
        goal_msg.yaw.mode = yaw_mode
        if yaw_angle:
            goal_msg.yaw.angle = yaw_angle
        return super().modify(goal_msg)

    def __get_pose(self, pose: Tuple[Pose, PoseStamped]):
        """get pose msg"""
        if isinstance(pose, Pose):
            return pose
        if isinstance(pose, PoseStamped):
            return pose.pose

        raise self.GoalRejected("Goal format invalid")
