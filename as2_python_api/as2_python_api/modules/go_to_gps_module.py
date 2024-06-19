"""Go to GPS Module."""

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
__copyright__ = 'Copyright(c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD - 3 - Clause'

from typing import TYPE_CHECKING

from as2_msgs.msg import YawMode
from as2_python_api.behavior_actions.go_to_behavior import GoToBehavior
from as2_python_api.modules.module_base import ModuleBase
from geographic_msgs.msg import GeoPose

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class GoToGpsModule(ModuleBase, GoToBehavior):
    """Go to GPS Module."""

    __alias__ = 'go_to_gps'
    __deps__ = ['gps']

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, lat: float, lon: float, alt: float, speed: float,
                 yaw_mode: int = YawMode.KEEP_YAW,
                 yaw_angle: float = None, wait: bool = True) -> bool:
        """
        Go to GPS point.

        :param lat: latitude (deg) of the point
        :type lat: float
        :param lon: longitude (deg) of the point
        :type lon: float
        :param alt: altitude (m) of the point
        :type alt: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(lat, lon, alt, speed, yaw_mode, yaw_angle, wait)

    def __go_to(self, lat: float, lon: float, alt: float,
                speed: float, yaw_mode: int, yaw_angle: float, wait: bool = True) -> bool:
        """
        Go to GPS point.

        :param lat: latitude (deg) of the point
        :type lat: float
        :param lon: longitude (deg) of the point
        :type lon: float
        :param alt: altitude (m) of the point
        :type alt: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        msg = GeoPose()
        msg.position.latitude = (float)(lat)
        msg.position.longitude = (float)(lon)
        msg.position.altitude = (float)(alt)

        return self.start(pose=msg, speed=speed, yaw_mode=yaw_mode,
                          yaw_angle=yaw_angle, wait_result=wait)

    # Method simplications
    def go_to_gps(self, lat: float, lon: float, alt: float, speed: float) -> bool:
        """
        Go to GPS point. Blocking call.

        :param lat: latitude (deg) of the point
        :type lat: float
        :param lon: longitude (deg) of the point
        :type lon: float
        :param alt: altitude (m) of the point
        :type alt: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(lat, lon, alt, speed,
                            yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_with_yaw(self, lat: float, lon: float, alt: float,
                           speed: float, angle: float) -> bool:
        """
        Go to GPS point. With desired yaw angle. Blocking call.

        :param lat: latitude (deg) of the point
        :type lat: float
        :param lon: longitude (deg) of the point
        :type lon: float
        :param alt: altitude (m) of the point
        :type alt: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: go to desired yaw angle (rad)
        :type yaw_angle: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(lat, lon, alt, speed,
                            yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_path_facing(self, lat: float, lon: float, alt: float, speed: float) -> bool:
        """
        Go to GPS point. With desired path facing. Blocking call.

        :param lat: latitude (deg) of the point
        :type lat: float
        :param lon: longitude (deg) of the point
        :type lon: float
        :param alt: altitude (m) of the point
        :type alt: float
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(lat, lon, alt, speed,
                            yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    def go_to_gps_point(self, waypoint: list[float], speed: float) -> bool:
        """
        Go to GPS point. Blocking call.

        :param point: GPS point [lat, lon, alt]
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                            speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_point_with_yaw(self, waypoint: list[float], speed: float, angle: float) -> bool:
        """
        Go to GPS point. With desired yaw angle. Blocking call.

        :param point: GPS point [lat, lon, alt]
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :param yaw_angle: go to desired yaw angle (rad)
        :type yaw_angle: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                            speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_point_path_facing(self, waypoint: list[float], speed: float) -> bool:
        """
        Go to GPS point. With desired path facing. Blocking call.

        :param point: GPS point [lat, lon, alt]
        :type point: list[float]
        :param speed: speed (m/s) to go to the point
        :type speed: float
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                            speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)
