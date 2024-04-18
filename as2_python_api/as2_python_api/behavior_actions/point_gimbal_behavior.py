"""
point_gimbal_behavior.py
"""

# Copyright 2024 Universidad Politécnica de Madrid
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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"


import typing
from typing import Tuple

from geometry_msgs.msg import PoseStamped, Pose
from as2_msgs.action import PointGimbal
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

if typing.TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class PointGimbalBehavior(BehaviorHandler):
    """PointGimbal Behavior"""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, PointGimbal, 'PointGimbalBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, pose: Tuple[Pose, PoseStamped], frame_id: str,
              wait_result: bool = False) -> bool:
        goal_msg = PointGimbal.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.control.target.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.control.target.header.frame_id = frame_id
        goal_msg.control.target.vector.x = pose_stamped.position.x
        goal_msg.control.target.vector.y = pose_stamped.position.y
        goal_msg.control.target.vector.z = pose_stamped.position.z

        return super().start(goal_msg, wait_result)

    def modify(self, pose: Tuple[Pose, PoseStamped], frame_id: str):
        goal_msg = PointGimbal.Goal()
        pose_stamped = self.__get_pose(pose)
        goal_msg.control.target.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.control.target.header.frame_id = frame_id  # TODO
        goal_msg.control.target.vector.x = pose_stamped.position.x
        goal_msg.control.target.vector.y = pose_stamped.position.y
        goal_msg.control.target.vector.z = pose_stamped.position.z
        return super().modify(goal_msg)

    def __get_pose(self, pose: Tuple[Pose, PoseStamped]):
        """get pose msg"""
        if isinstance(pose, Pose):
            return pose
        if isinstance(pose, PoseStamped):
            return pose.pose

        raise self.GoalRejected("Goal format invalid")
