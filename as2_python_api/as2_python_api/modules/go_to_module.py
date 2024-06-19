"""Go to Module."""

from __future__ import annotations

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


__authors__ = 'Pedro Arias Pérez, Miguel Fernández Cortizas, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import TYPE_CHECKING

from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.go_to_behavior import GoToBehavior
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class GoToModule(ModuleBase, GoToBehavior):
    """Go to Module."""

    __alias__ = 'go_to'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, x: float, y: float, z: float, speed: float,
                 yaw_mode: int = YawMode.KEEP_YAW, yaw_angle: float = None,
                 frame_id: str = 'earth', wait: bool = True) -> bool:
        """
        Go to point.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(x, y, z, speed, yaw_mode, yaw_angle, frame_id, wait)

    def __go_to(self, x: float, y: float, z: float,
                speed: float, yaw_mode: int, yaw_angle: float,
                frame_id: str = 'earth', wait: bool = True) -> bool:
        """
        Go to point.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        msg = Pose()
        msg.position.x = (float)(x)
        msg.position.y = (float)(y)
        msg.position.z = (float)(z)
        return self.start(msg, speed, yaw_mode, yaw_angle, frame_id, wait)

    # Method simplifications
    def go_to(self, x: float, y: float, z: float, speed: float,
              frame_id: str = 'earth') -> bool:
        """
        Go to point, blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(x, y, z, speed,
                            yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, frame_id=frame_id)

    def go_to_with_yaw(self, x: float, y: float, z: float, speed: float, angle: float,
                       frame_id: str = 'earth') -> bool:
        """
        Go to point. With desired yaw angle (degrees). Blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: yaw angle
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(x, y, z, speed,
                            yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, frame_id=frame_id)

    def go_to_path_facing(self, x: float, y: float, z: float, speed: float,
                          frame_id: str = 'earth') -> bool:
        """
        Go to point. With path facing yaw mode. Blocking call.

        :param x: x coordinate (m) to go to
        :type x: float
        :param y: y coordinate (m) to go to
        :type y: float
        :param z: z coordinate (m) to go to
        :type z: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(x, y, z, speed,
                            yaw_mode=YawMode.PATH_FACING, yaw_angle=None, frame_id=frame_id)

    def go_to_point(self, point: list[float], speed: float, frame_id: str = 'earth') -> bool:
        """
        Go to point. With keep yaw mode. Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(point[0], point[1], point[2],
                            speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None, frame_id=frame_id)

    def go_to_point_with_yaw(self, point: list[float], speed: float, angle: float,
                             frame_id: str = 'earth') -> bool:
        """
        Go to point. With desired yaw angle (degrees). Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: yaw angle
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(point[0], point[1], point[2],
                            speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle, frame_id=frame_id)

    def go_to_point_path_facing(self, point: list[float], speed: float,
                                frame_id: str = 'earth') -> bool:
        """
        Go to point. With path facing yaw mode. Blocking call.

        :param point: [x, y, z] (m) coordinates to go to
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(point[0], point[1], point[2],
                            speed, yaw_mode=YawMode.PATH_FACING, frame_id=frame_id, yaw_angle=None)
