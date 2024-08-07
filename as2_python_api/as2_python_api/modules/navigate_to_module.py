"""Navigate To Module."""

from __future__ import annotations

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

from typing import TYPE_CHECKING

from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.navigate_to_behavior import NavigateToBehavior
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class NavigateToModule(ModuleBase, NavigateToBehavior):
    """Navigate to Module."""

    __alias__ = 'navigate_to'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, x: float, y: float, z: float, speed: float,
                 yaw_mode: int = YawMode.FIXED_YAW, yaw_angle: float = None,
                 frame_id: str = 'earth', wait: bool = True) -> None:
        """
        Navigate to point (m) with speed (m/s).

        :type x: float
        :type y: float
        :type z: float
        :type speed: float
        :type yaw_mode: int
        :type yaw_angle: float
        :type wait: bool
        """
        self.__navigate_to(x, y, z, speed, yaw_mode, yaw_angle, frame_id, wait)

    def __navigate_to(self, x: float, y: float, z: float,
                      speed: float, yaw_mode: int, yaw_angle: float,
                      frame_id: str = 'earth', wait: bool = True) -> None:
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        self.start(msg, speed, yaw_mode, yaw_angle, frame_id, wait)

    # Method simplifications
    def navigate_to(self, x: float, y: float, z: float, speed: float,
                    frame_id: str = 'earth') -> None:
        """
        Navigate to point (m) with speed (m/s).

        :type x: float
        :type y: float
        :type z: float
        :type speed: float
        """
        self.__navigate_to(x, y, z, speed, yaw_mode=YawMode.KEEP_YAW,
                           yaw_angle=None, frame_id=frame_id)

    def navigate_to_with_yaw(self, x: float, y: float, z: float, speed: float, angle: float,
                             frame_id: str = 'earth') -> None:
        """
        Navigate to position with speed and yaw_angle.

        :type x: float
        :type y: float
        :type z: float
        :type speed: float
        :type yaw_angle: float
        """
        self.__navigate_to(x, y, z, speed, yaw_mode=YawMode.FIXED_YAW,
                           yaw_angle=angle, frame_id=frame_id)

    def navigate_to_path_facing(self, x: float, y: float, z: float, speed: float,
                                frame_id: str = 'earth') -> None:
        """
        Navigate to position facing goal with speed.

        :type x: float
        :type y: float
        :type z: float
        :type speed: float
        """
        self.__navigate_to(x, y, z, speed, yaw_mode=YawMode.PATH_FACING,
                           yaw_angle=None, frame_id=frame_id)

    def navigate_to_point(self, point: list[float], speed: float, frame_id: str = 'earth') -> None:
        """
        Navigate to point (m) with speed (m/s).

        :type point: List[float]
        :type speed: float
        """
        self.__navigate_to(point[0], point[1], point[2], speed,
                           yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, frame_id=frame_id)

    def navigate_to_point_with_yaw(self, point: list[float], speed: float, angle: float,
                                   frame_id: str = 'earth') -> None:
        """
        Navigate to point with speed and yaw_angle.

        :type point: List[float]
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__navigate_to(point[0], point[1], point[2], speed,
                           yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, frame_id=frame_id)

    def navigate_to_point_path_facing(self, point: list[float], speed: float,
                                      frame_id: str = 'earth') -> None:
        """
        Navigate to point facing goal with speed.

        :type point: List[float]
        :type speed: float
        """
        self.__navigate_to(point[0], point[1], point[2], speed,
                           yaw_mode=YawMode.PATH_FACING, frame_id=frame_id, yaw_angle=None)
