"""Goto action handler"""

# Copyright (c) 2022 Universidad Politécnica de Madrid
# All Rights Reserved
#
# Licensed under the BSD-3-Clause (the "License");
# you may not use this file except in compliance with the License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import typing
from typing import Tuple

from rclpy.action import ActionClient

from as2_msgs.action import GoToWaypoint
from as2_msgs.srv import GeopathToPath
from as2_msgs.msg import YawMode
from geometry_msgs.msg import PoseStamped, Pose
from geographic_msgs.msg import GeoPoseStamped, GeoPose

from python_interface.behaviour_actions.action_handler import ActionHandler

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class SendGoToWaypoint(ActionHandler):
    """Go to action"""

    def __init__(self, drone: 'DroneInterface',
                 pose: Tuple[Pose, PoseStamped, GeoPose, GeoPoseStamped],
                 speed: float, yaw_mode: int, yaw_angle: float) -> None:
        self._action_client = ActionClient(
            drone, GoToWaypoint, 'GoToWaypointBehaviour')

        self._drone = drone

        goal_msg = GoToWaypoint.Goal()
        pose_stamped = self.get_pose(pose)
        goal_msg.target_pose.header.stamp = self._drone.get_clock().now()
        goal_msg.target_pose.header.frame_id = "earth"  # TODO
        goal_msg.target_pose.point.x = pose_stamped.position.x
        goal_msg.target_pose.point.y = pose_stamped.position.y
        goal_msg.target_pose.point.z = pose_stamped.position.z

        goal_msg.max_speed = speed
        goal_msg.yaw.mode = yaw_mode
        if yaw_angle:
            goal_msg.yaw.angle = yaw_angle

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))

    def get_pose(self, pose: Tuple[Pose, PoseStamped, GeoPose, GeoPoseStamped]):
        """get pose msg"""
        if isinstance(pose, Pose):
            return pose
        if isinstance(pose, PoseStamped):
            return pose.pose
        if isinstance(pose, GeoPose):
            geopose = GeoPoseStamped()
            geopose.pose = pose
            return self.get_pose(geopose)
        if isinstance(pose, GeoPoseStamped):
            req = GeopathToPath.Request()
            req.geo_path.poses = [pose]
            resp = self._drone.global_to_local_cli_.call(req)
            if not resp.success:
                self._drone.get_logger().warn("Can't follow path since origin is not set")
                raise self.GoalFailed("GPS service not available")

            return resp.path.poses[0].pose

        raise self.GoalRejected("Goal format invalid")
