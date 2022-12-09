"""Follow path action handler"""

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
from dataclasses import dataclass
from typing import Any, Union

from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from as2_msgs.msg import TrajectoryWaypoints
from as2_msgs.srv import GeopathToPath
from as2_msgs.action import FollowPath

from ..behaviour_actions.action_handler import ActionHandler
from ..tools.utils import path_to_list

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class SendFollowPath(ActionHandler):
    """Send follow path action"""

    @dataclass
    class FollowPathData:
        """Follow Path Data"""
        path: Any
        speed: float = 1.0
        yaw_mode: TrajectoryWaypoints.yaw_mode = TrajectoryWaypoints.KEEP_YAW
        is_gps: bool = False

    def __init__(self, drone: 'DroneInterface',
                 path_data: Union[list, tuple, Path, GeoPath, TrajectoryWaypoints]) -> None:
        self._action_client = ActionClient(
            drone, FollowPath, 'FollowPathBehaviour')
        self._drone = drone

        goal_msg = FollowPath.Goal()
        goal_msg.trajectory_waypoints = self.get_traj(path_data)

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))

    def get_traj(self, path_data: Union[list, tuple, Path, GeoPath, TrajectoryWaypoints]):
        """get trajectory msg"""
        if isinstance(path_data.path, list):
            if not path_data.path:  # not empty
                raise self.GoalRejected("Goal format invalid")
            if isinstance(path_data.path[0], list):
                point_list = path_data.path
            else:
                point_list = [path_data.path]
        elif isinstance(path_data.path, tuple):
            point_list = [list(path_data.path)]
        elif isinstance(path_data.path, Path):
            point_list = path_to_list(path_data.path)
        elif isinstance(path_data.path, GeoPath):
            req = GeopathToPath.Request()
            req.geo_path = path_data.path
            resp = self._drone.global_to_local_cli_.call(req)
            if not resp.success:
                self._drone.get_logger().warn("Can't follow path since origin is not set")
                raise self.GoalFailed("GPS service not available")

            point_list = path_to_list(resp.path)
        elif isinstance(path_data.path, TrajectoryWaypoints):
            return path_data.path
        else:
            raise self.GoalRejected("Goal format invalid")

        if path_data.is_gps:
            geopath = GeoPath()
            geopath.header.stamp = self._drone.get_clock().now().to_msg()
            geopath.header.frame_id = "wgs84"
            for _wp in point_list:
                gps = GeoPoseStamped()
                gps.header.stamp = geopath.header.stamp
                gps.header.frame_id = geopath.header.frame_id
                gps.pose.position.latitude = float(_wp[0])
                gps.pose.position.longitude = float(_wp[1])
                gps.pose.position.altitude = float(_wp[2])
                geopath.poses.append(gps)

            path_data.path = geopath
            path_data.is_gps = False
            return self.get_traj(path_data)

        msg = TrajectoryWaypoints()
        msg.header.stamp = self._drone.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.yaw_mode = path_data.yaw_mode
        poses = []
        for point in point_list:
            pose = PoseStamped()
            _x, _y, _z = point
            pose.pose.position.x = (float)(_x)
            pose.pose.position.y = (float)(_y)
            pose.pose.position.z = (float)(_z)
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        msg.poses = poses
        msg.max_speed = (float)(path_data.speed)
        return msg
