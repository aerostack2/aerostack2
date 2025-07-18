"""Trajectory Generation Behavior."""

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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import TYPE_CHECKING, Union

from as2_msgs.action import GeneratePolynomialTrajectory
from as2_msgs.msg import PoseStampedWithID, PoseWithID, YawMode
from nav_msgs.msg import Path

from .behavior_handler import BehaviorHandler
from ..tools.utils import path_to_list

if TYPE_CHECKING:
    from ..drone_interface_base import DroneInterfaceBase


class TrajectoryGenerationBehavior(BehaviorHandler):
    """Trajectory Generation Behavior."""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, GeneratePolynomialTrajectory, 'TrajectoryGeneratorBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

    def start(self, path: Union[list, tuple, Path, PoseWithID, PoseStampedWithID],
              speed: float, yaw_mode: int, yaw_angle: float, frame_id: str = 'earth',
              wait_result: bool = True) -> bool:
        """Start behavior."""
        goal_msg = GeneratePolynomialTrajectory.Goal()
        goal_msg.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.path = self.__get_path(path, frame_id)
        yaw_msg = YawMode()
        yaw_msg.angle = yaw_angle
        yaw_msg.mode = yaw_mode
        goal_msg.yaw = yaw_msg
        goal_msg.max_speed = speed
        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__drone.get_logger().warn(str(err))
        return False

    def modify(self, path: Union[list, tuple, Path, PoseWithID, PoseStampedWithID],
               speed: float, yaw_mode: int, yaw_angle: float, frame_id: str = 'earth'):
        """Modify behavior."""
        goal_msg = GeneratePolynomialTrajectory.Goal()
        goal_msg.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.path = self.__get_path(path, frame_id)
        yaw_msg = YawMode()
        yaw_msg.angle = yaw_angle
        yaw_msg.mode = yaw_mode
        goal_msg.yaw = yaw_msg
        goal_msg.max_speed = speed
        return super().modify(goal_msg)

    def __get_pose_stamped_with_id(
            self, pose: Union[list, tuple, PoseWithID], frame_id: str) -> PoseStampedWithID:
        """Get pose stamped with id."""
        pose_stamped = PoseStampedWithID()
        pose_stamped.pose.header.frame_id = frame_id
        pose_stamped.pose.header.stamp = self.__drone.get_clock().now().to_msg()
        if isinstance(pose, PoseWithID):
            pose_stamped.pose = pose.pose
            pose_stamped.id = pose.id
        elif isinstance(pose, tuple):
            pose_stamped.pose.pose.position.x = float(pose[0])
            pose_stamped.pose.pose.position.y = float(pose[1])
            pose_stamped.pose.pose.position.z = float(pose[2])
        elif isinstance(pose, list):
            pose_stamped.pose.pose.position.x = float(pose[0])
            pose_stamped.pose.pose.position.y = float(pose[1])
            pose_stamped.pose.pose.position.z = float(pose[2])
        return pose_stamped

    def __list_to_pose_stamped_with_id(
            self, path: Union[list, tuple, PoseWithID], frame_id: str) -> list[PoseStampedWithID]:
        """Convert list to pose stamped with id."""
        point_list = []
        waypoint_id = 0
        for waypoint in path:
            pose_stamped = self.__get_pose_stamped_with_id(waypoint, frame_id)
            pose_stamped.id = str(waypoint_id)
            point_list.append(pose_stamped)
            waypoint_id += 1
        return point_list

    def __get_path(
            self, path: Union[list, tuple, Path, PoseWithID, PoseStampedWithID], frame_id: str):
        """Get trajectory msg."""
        # If is a list
        if isinstance(path, list):
            point_list = []
            if len(path) == 0:
                raise self.GoalRejected('Goal format invalid')

            # Waypoints is a list of 3 elements
            if isinstance(path[0], float):
                if len(path) != 3:
                    raise self.GoalRejected('Goal format invalid')
                return self.__list_to_pose_stamped_with_id([path], frame_id)
            # Waypoints is a list of lists, each list with 3 elements
            if isinstance(path[0], list):
                return self.__list_to_pose_stamped_with_id(path, frame_id)
            # Waypoints is list of PoseWithID
            elif isinstance(path[0], PoseWithID):
                return self.__list_to_pose_stamped_with_id(path, frame_id)
            # If is a list of PoseStampedWithID
            elif isinstance(path[0], PoseStampedWithID):
                return path
            else:
                raise self.GoalRejected('Goal format invalid')

        # If is a tuple
        elif isinstance(path, tuple):
            point_list = [list(path)]
            return self.__get_path(point_list, frame_id)
        # If is a Path
        elif isinstance(path, Path):
            point_list = path_to_list(path)
            return self.__get_path(point_list, frame_id)
        # If is a PoseWithID
        elif isinstance(path, PoseWithID):
            return self.__get_path([path], frame_id)
        # If is a PoseStampedWithID
        elif isinstance(path, PoseStampedWithID):
            return [path]
        else:
            raise self.GoalRejected('Goal format invalid')
